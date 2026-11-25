"""One continuous MANUAL deadband hold; shared by live progress and bag analysis."""
HOLD = 5.0
TARGET = "CH1 1020~1050 / CH2 980~1004"
AGES = {"/rc_channels": .30, "/rc_uplink_lq": 1., "/boat/motor_status": 1.,
        "/boat/estop": .15, "/boat/cmd_vel": .12, "/boat/motor_preview": .12}


class DeadbandHold:
    def __init__(self):
        self.latest = {}
        self.since = None
        self.elapsed = 0.
        self.samples = 0
        self.mismatch = False
        self.result = None

    def update(self, topic, now, value):
        if self.result:
            return
        if topic in self.latest and now - self.latest[topic][0] > AGES[topic]:
            self.since = None
        self.latest[topic] = now, value
        self.advance(now, topic)

    def advance(self, now, topic=None):
        if self.result:
            return
        fresh = all(k in self.latest and now-self.latest[k][0] <= limit for k,limit in AGES.items())
        if fresh:
            rc = self.latest["/rc_channels"][1]
            state = self.latest["/boat/motor_status"][1]
            fresh = (len(rc)>=6 and 1020<=rc[0]<=1050 and 980<=rc[1]<=1004
                     and rc[4]>1400 and 800<rc[5]<1200
                     and state.get("backend")=="preview" and state.get("armed") is True
                     and self.latest["/boat/estop"][1] is False
                     and 0<self.latest["/rc_uplink_lq"][1]<=100)
        if not fresh:
            self.since = None
            self.elapsed = 0.
            return
        if self.since is None:
            self.since = now
            self.samples = 0
            self.mismatch = False
        self.elapsed = now-self.since
        # Allow previous command/output to settle for one motor tick at entry.
        if self.elapsed >= .12:
            if topic == "/boat/motor_preview":
                self.samples += 1
                self.mismatch |= self.latest[topic][1] != [0.,0.,1500.,1500.]
            if topic == "/boat/cmd_vel":
                self.mismatch |= self.latest[topic][1] != (0.,0.)
        if topic == "/boat/motor_preview" and self.elapsed >= HOLD:
            self.result = "FAIL" if self.mismatch else "PASS"
