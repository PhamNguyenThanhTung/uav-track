import struct
import pytest
from profile_perception_pipeline import cdr_stamp, frame_metrics, percentiles


def test_cdr_stamp_keeps_exact_nanoseconds_and_endianness():
    for order, flag in [('<',1),('>',0)]:
        assert cdr_stamp(bytes([0,flag,0,0])+struct.pack(order+'iI',104,987654321))==104_987_654_321
    with pytest.raises(ValueError):cdr_stamp(b'bad')


def test_performance_clock_domains_are_never_subtracted():
    row=dict(source_stamp_ns=100_000_000_000,receive_source_clock_ns=100_020_000_000,
             guard_decision_source_clock_ns=100_110_000_000,
             rmw_published_wall_ns=1_790_000_000_000_000_000,
             rmw_received_wall_ns=1_790_000_000_005_000_000,
             callback_wall_ns=1_790_000_000_009_000_000)
    measured=frame_metrics(row)
    assert measured['receive_age_source_ms']==20
    assert measured['decision_age_source_ms']==110
    assert measured['dds_transport_ms']==5
    assert measured['queue_wait_wall_ms']==4
    assert frame_metrics({'source_stamp_ns':100})['queue_wait_wall_ms'] is None
    assert percentiles([1,3,None,float('nan')])==dict(n=2,p50=2.,p95=2.9,max=3.)
