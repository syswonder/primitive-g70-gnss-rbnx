import struct,sys
from pathlib import Path
sys.path.insert(0,str(Path(__file__).parents[1]/'vendor'))
from g70_bridge.ubx import PvtDecoder,decode_pvt


def packet():
    p=bytearray(92);struct.pack_into('<I',p,0,1000);p[20]=3;p[21]=3|(2<<6);p[23]=20
    struct.pack_into('<iiiiII',p,24,1200000000,300000000,12000,10000,100,200)
    struct.pack_into('<iii',p,48,200,1000,0);struct.pack_into('<I',p,68,20)
    raw=bytes((1,7,92,0))+p;a=b=0
    for x in raw:a=(a+x)&255;b=(b+a)&255
    return b'\xb5\x62'+raw+bytes((a,b))


def test_pvt_units_quality_accuracy_and_split_stream():
    raw=packet();decoder=PvtDecoder();assert decoder.feed(raw[:15])==[]
    sample=decoder.feed(raw[15:])[0]
    assert sample['quality']==4 and sample['valid']
    assert sample['east_mps']==1. and sample['north_mps']==.2
    assert sample['speed_accuracy_mps']==.02 and sample['hacc_m']==.1
    assert sample['altitude_ellipsoid']==12. and sample['altitude_msl']==10.
    assert decoder.feed(raw)==[]


def test_invalid_checksum_is_not_a_measurement():
    raw=bytearray(packet());raw[-1]^=1;decoder=PvtDecoder()
    assert decoder.feed(raw)==[] and decoder.bad_checksums==1
