"""NTRIP worker. GGA must originate from a fresh checksum-validated receiver fix."""
import base64,socket,time,threading
from urllib.parse import quote
from .protocol import RtcmFramer,ChunkDecoder

def read_response(sock):
    def line():
        buf=bytearray()
        while not buf.endswith(b'\r\n'):
            b=sock.recv(1)
            if not b:raise ConnectionError('Caster closed during response')
            buf.extend(b)
            if len(buf)>8192:raise ValueError('Response line too long')
        return bytes(buf)
    status=line().decode('latin1').strip()
    if status.startswith('ICY 200'):
        # Some v1 casters send only this line; optional textual headers are discarded by RTCM framing.
        return False,status
    if not (status.startswith('HTTP/1.0 200 ') or status.startswith('HTTP/1.1 200 ')):
        raise ConnectionError('NTRIP rejected: '+status[:120])
    headers=bytearray()
    while True:
        x=line()
        if x==b'\r\n':break
        headers.extend(x)
        if len(headers)>16384:raise ValueError('Response headers too long')
    low=headers.lower()
    if b'sourcetable' in low or b'text/' in low:raise ValueError('Caster returned text/source table, not corrections')
    if b'content-encoding:' in low and b'content-encoding: identity' not in low:
        raise ValueError('Compressed stream unsupported')
    return b'transfer-encoding: chunked' in low,status

def request(config,password,gga):
    user=config['user'];host=config['host'];mount=config['mount'].strip('/')
    if not mount or any('\r' in str(v) or '\n' in str(v) for v in [user,host,mount]):
        raise ValueError('Invalid NTRIP configuration')
    credentials=base64.b64encode((user+':'+password).encode()).decode()
    return (f'GET /{quote(mount,safe="/")} HTTP/1.1\r\nHost: {host}:{config["port"]}\r\n'
        'Ntrip-Version: Ntrip/2.0\r\nUser-Agent: NTRIP G70Linux/2.0\r\n'
        f'Authorization: Basic {credentials}\r\nNtrip-GGA: {gga.decode().strip()}\r\n'
        'Connection: close\r\n\r\n').encode('ascii')

class NtripWorker:
    def __init__(self,config,password,get_gga,write_rtcm):
        self.config=config;self.password=password;self.get_gga=get_gga;self.write_rtcm=write_rtcm
        self.stop=threading.Event();self.lock=threading.Lock()
        self.data=dict(state='WAIT_GGA',rtcm_bytes=0,rtcm_frames=0,crc_errors=0,last_rtcm_monotonic=None,message_types={})
        self.thread=threading.Thread(target=self.run,daemon=True)
    def update(self,**kw):
        with self.lock:self.data.update(kw)
    def snapshot(self):
        with self.lock:return {**self.data,'message_types':dict(self.data['message_types'])}
    def run(self):
        if self.password is None:
            self.update(state='PASSWORD_NOT_CONFIGURED');return
        while not self.stop.is_set():
            gga=self.get_gga()
            if not gga:self.update(state='WAIT_FRESH_GGA');self.stop.wait(.5);continue
            try:
                self.update(state='CONNECTING')
                with socket.create_connection((self.config['host'],self.config['port']),timeout=5) as sock:
                    sock.settimeout(5);sock.sendall(request(self.config,self.password,gga))
                    # VRS casters can wait for a GGA sentence before delivering corrections.
                    sock.sendall(gga)
                    chunked,status=read_response(sock)
                    self.update(state='CONNECTED',response=status)
                    sock.settimeout(.5);framer=RtcmFramer();decoder=ChunkDecoder()
                    sent=time.monotonic();last_valid=time.monotonic()
                    while not self.stop.is_set():
                        now=time.monotonic()
                        current=self.get_gga()
                        if not current:raise ConnectionError('Receiver GGA stale or no fix')
                        if now-sent>=self.config.get('gga_interval',5):
                            sock.sendall(current);sent=now
                        if now-last_valid>self.config.get('stream_timeout',15):raise ConnectionError('No valid RTCM frames')
                        try:data=sock.recv(4096)
                        except socket.timeout:continue
                        if not data:raise ConnectionError('Correction stream closed')
                        if chunked:data=decoder.feed(data)
                        for frame in framer.feed(data):
                            self.write_rtcm(frame)
                            last_valid=time.monotonic();identity=(frame[3]<<4)|(frame[4]>>4)
                            with self.lock:
                                self.data['rtcm_bytes']+=len(frame);self.data['rtcm_frames']+=1
                                self.data['last_rtcm_monotonic']=last_valid
                                self.data['message_types'][str(identity)]=self.data['message_types'].get(str(identity),0)+1
                        self.update(crc_errors=framer.bad_crc,state='STREAMING' if self.snapshot()['last_rtcm_monotonic'] else 'CONNECTED')
                        if chunked and decoder.done:raise ConnectionError('End of chunked stream')
            except (OSError,ValueError) as e:
                if 'NTRIP rejected:' in str(e) and any(x in str(e) for x in ['401','403']):
                    self.update(state='AUTH_FAILED',error='Caster rejected credentials; restart after correction');return
                self.update(state='RECONNECT_WAIT',error=str(e)[:180]);self.stop.wait(5)
    def start(self):self.thread.start()
    def close(self):self.stop.set();self.thread.join(timeout=6)
