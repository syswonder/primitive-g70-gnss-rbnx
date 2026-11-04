"""Minimal checked NMEA GGA and RTCM3 framing. No device configuration writes."""
import math

QUALITY={0:'NO_FIX',1:'SINGLE',2:'DGPS',4:'RTK_FIXED',5:'RTK_FLOAT',6:'DEAD_RECKONING',7:'MANUAL',8:'SIMULATION'}

def checked_nmea(line):
    text=line.decode('ascii') if isinstance(line,bytes) else line
    text=text.strip()
    if not text.startswith('$') or '*' not in text:raise ValueError('NMEA framing')
    body,crc=text[1:].rsplit('*',1)
    check=0
    for c in body:check^=ord(c)
    if len(crc)!=2 or check!=int(crc,16):raise ValueError('NMEA checksum')
    return body.split(',')

def coordinate(value, hemisphere, latitude):
    v=float(value);degrees=int(v//100);minutes=v-degrees*100
    limit=90 if latitude else 180
    if not math.isfinite(v) or v<0 or not 0<=minutes<60 or degrees>limit or (degrees==limit and minutes!=0):
        raise ValueError('NMEA coordinate')
    if hemisphere not in ('NS' if latitude else 'EW'):raise ValueError('NMEA hemisphere')
    return (degrees+minutes/60)*(-1 if hemisphere in 'SW' else 1)

def gga(line):
    f=checked_nmea(line)
    if len(f[0])!=5 or f[0][-3:]!='GGA':return None
    if len(f)<15:raise ValueError('Short GGA')
    quality=int(f[6]);valid=quality in (1,2,4,5)
    if valid:
        utc=f[1]
        if len(utc)<6 or not utc[:6].isdigit() or not 0<=int(utc[:2])<24 or not 0<=int(utc[2:4])<60 or not 0<=float(utc[4:])<61:
            raise ValueError('Invalid GGA UTC time')
    lat=coordinate(f[2],f[3],True) if f[2] else math.nan
    lon=coordinate(f[4],f[5],False) if f[4] else math.nan
    valid=valid and math.isfinite(lat) and math.isfinite(lon)
    def number(s):
        if not s:return None
        value=float(s)
        if not math.isfinite(value):raise ValueError('Non-finite NMEA field')
        return value
    alt=number(f[9]);geoid=number(f[11])
    if alt is not None and f[10]!='M':raise ValueError('GGA altitude unit')
    if geoid is not None and f[12]!='M':raise ValueError('GGA geoid unit')
    return dict(quality=quality,state=QUALITY.get(quality,'UNKNOWN_QUALITY'),valid=bool(valid),
        latitude=lat,longitude=lon,altitude_msl=alt,geoid_separation=geoid,
        altitude_ellipsoid=(alt+geoid if alt is not None and geoid is not None else math.nan),
        satellites=int(f[7] or 0),hdop=number(f[8]),differential_age_s=number(f[13]),
        station_id=f[14],utc_time=f[1])

def crc24q(data):
    crc=0
    for byte in data:
        crc^=byte<<16
        for _ in range(8):
            crc<<=1
            if crc&0x1000000:crc^=0x1864CFB
    return crc&0xFFFFFF

class RtcmFramer:
    def __init__(self):self.buffer=bytearray();self.bad_crc=0
    def feed(self,data):
        self.buffer.extend(data);frames=[]
        while self.buffer:
            if self.buffer[0]!=0xD3:del self.buffer[0];continue
            if len(self.buffer)<3:break
            if self.buffer[1]&0xFC:del self.buffer[0];continue
            size=((self.buffer[1]&3)<<8)|self.buffer[2]
            if size<2:del self.buffer[0];continue
            end=size+6
            if len(self.buffer)<end:break
            packet=bytes(self.buffer[:end])
            if crc24q(packet[:-3])!=int.from_bytes(packet[-3:],'big'):
                self.bad_crc+=1;del self.buffer[0];continue
            del self.buffer[:end];frames.append(packet)
        return frames

class ChunkDecoder:
    def __init__(self):self.buffer=bytearray();self.size=None;self.done=False
    def feed(self,data):
        if self.done:return b''
        self.buffer.extend(data);out=bytearray()
        while True:
            if self.size is None:
                end=self.buffer.find(b'\r\n')
                if end<0:
                    if len(self.buffer)>1024:raise ValueError('chunk header too long')
                    break
                self.size=int(self.buffer[:end].split(b';')[0],16);del self.buffer[:end+2]
                if self.size==0:self.done=True;break
                if not 0<self.size<=1048576:raise ValueError('chunk too large')
            if len(self.buffer)<self.size+2:break
            if self.buffer[self.size:self.size+2]!=b'\r\n':raise ValueError('chunk terminator')
            out.extend(self.buffer[:self.size]);del self.buffer[:self.size+2];self.size=None
        return bytes(out)
