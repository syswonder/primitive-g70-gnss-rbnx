import math
import sys
from pathlib import Path
sys.path.insert(0,str(Path(__file__).parents[1]/'vendor'))
from g70_bridge.protocol import rmc,gst,MeasurementCache
import pytest


def nmea(body):
    checksum=0
    for byte in body.encode():checksum^=byte
    return f'${body}*{checksum:02X}'


def test_receiver_velocity_units_axes_and_unknown_accuracy():
    v=rmc(nmea('GNRMC,120000.00,A,3000.0,N,12000.0,E,1.94384449,90.0,051126,,,A'))
    assert v['east_mps']==pytest.approx(1.,abs=1e-7) and abs(v['north_mps'])<1e-6
    assert v['covariance_known'] is False
    assert rmc(nmea('GNRMC,120000.00,V,3000.0,N,12000.0,E,1,90,051126,,,N'))['valid'] is False


def test_replayed_measurement_is_not_fresh():
    cache=MeasurementCache();line=nmea('GNRMC,120000.00,A,3000,N,12000,E,0,,051126,,,A')
    assert cache.feed(line,10.,'stamp')=='velocity'
    assert cache.feed(line,11.,'new_stamp') is None
    assert cache.velocity_stamp=='stamp'
    assert cache.velocity_status(11.1,1.)['fresh'] is False


def test_gst_is_same_epoch_and_never_hdop_derived():
    cache=MeasurementCache();cache.feed(nmea('GNGST,120000.00,1,1,1,0,0.1,0.2,0.3'),10.,None)
    sigma=cache.position_sigma('120000.000',10.1,1.)
    assert sigma['longitude_sigma_m']==.2
    assert cache.position_sigma('120001',10.1,1.) is None
    assert cache.position_sigma('120000',11.1,1.) is None
    with pytest.raises(ValueError):gst(nmea('GNGST,120000,1,1,1,0,0,0.2,0.3'))


def test_bad_checksum_negative_speed_and_missing_moving_course():
    for body in ('GNRMC,120000,A,3000,N,12000,E,-1,90,051126,,,A',
                 'GNRMC,120000,A,3000,N,12000,E,1,,051126,,,A'):
        with pytest.raises(ValueError):rmc(nmea(body))
