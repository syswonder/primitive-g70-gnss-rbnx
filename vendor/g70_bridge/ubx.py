"""Explicit NAV-PVT compatibility for a u-blox receiver; no device writes."""
import struct


def decode_pvt(payload):
    if len(payload)<92:raise ValueError('Short NAV-PVT')
    lon,lat,height,msl,hacc,vacc=struct.unpack_from('<iiiiII',payload,24)
    north,east,down=struct.unpack_from('<iii',payload,48)
    flags=payload[21];carrier=(flags>>6)&3
    valid=bool(flags&1) and payload[20] in (3,4)
    quality=4 if valid and carrier==2 else 5 if valid and carrier==1 else 1 if valid else 0
    return {'tow_ms':struct.unpack_from('<I',payload,0)[0],'quality':quality,
        'state':{0:'NO_FIX',1:'SINGLE',4:'RTK_FIXED',5:'RTK_FLOAT'}[quality],
        'valid':valid,'latitude':lat*1e-7,'longitude':lon*1e-7,
        'altitude_ellipsoid':height/1000,'altitude_msl':msl/1000,
        'hacc_m':hacc/1000,'vacc_m':vacc/1000,
        'east_mps':east/1000,'north_mps':north/1000,'down_mps':down/1000,
        'speed_accuracy_mps':struct.unpack_from('<I',payload,68)[0]/1000,
        'satellites':payload[23],'differential':bool(flags&2)}


class PvtDecoder:
    def __init__(self):self.buffer=bytearray();self.last_tow=None;self.bad_checksums=0
    def feed(self,data):
        self.buffer.extend(data);samples=[]
        while True:
            begin=self.buffer.find(b'\xb5\x62')
            if begin<0:
                self.buffer[:]=self.buffer[-1:];break
            if begin:del self.buffer[:begin]
            if len(self.buffer)<8:break
            size=int.from_bytes(self.buffer[4:6],'little')
            if size>4096:del self.buffer[0];continue
            end=size+8
            if len(self.buffer)<end:break
            a=b=0
            for value in self.buffer[2:end-2]:a=(a+value)&255;b=(b+a)&255
            if bytes((a,b))!=self.buffer[end-2:end]:
                self.bad_checksums+=1;del self.buffer[0];continue
            cls,ident=self.buffer[2:4];payload=bytes(self.buffer[6:end-2]);del self.buffer[:end]
            if (cls,ident)!=(1,7):continue
            sample=decode_pvt(payload)
            if sample['tow_ms']==self.last_tow:continue
            self.last_tow=sample['tow_ms'];samples.append(sample)
        return samples
