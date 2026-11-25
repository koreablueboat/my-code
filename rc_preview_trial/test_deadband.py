"""Offline regression for the live/offline hold criterion. No ROS or devices."""
from deadband import DeadbandHold


def feed(hold, start, end, *, steer=1030, output=0., armed=True):
    for tick in range(start,end):
        t=tick/20
        updates = {
            "/rc_channels":[steer,992,0,0,1792,997],
            "/rc_uplink_lq":95,
            "/boat/estop":False,
            "/boat/motor_status":{"backend":"preview","armed":armed},
            "/boat/cmd_vel":(0.,0.),
            "/boat/motor_preview":[output,0.,1500.,1500.],
        }
        for k,v in updates.items():
            hold.update(k,t,v)


def test_range_exit_restarts_hold():
    h=DeadbandHold()
    feed(h,0,80)
    feed(h,80,81,steer=1051)
    feed(h,81,181)
    assert h.result is None
    feed(h,181,182)
    assert h.result=="PASS"


def test_gap_and_unarmed_do_not_accumulate():
    h=DeadbandHold()
    feed(h,0,70,armed=False)
    feed(h,70,140)
    feed(h,180,250)
    assert h.result is None
    feed(h,250,282)
    assert h.result=="PASS"


def test_bad_output_is_not_retried_as_success():
    h=DeadbandHold()
    feed(h,0,80)
    feed(h,80,81,output=.01)
    feed(h,81,150)
    assert h.result=="FAIL"
