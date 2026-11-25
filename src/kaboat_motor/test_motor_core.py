import unittest
from reverse_motor_core import Controller as BidirectionalController
from motor_core import Controller,mix
from esc_config import pulse_us
import json
from pathlib import Path
PROFILE=json.loads(Path(__file__).with_name("preview_bidirectional.json").read_text())
def pwm(q):return pulse_us(q,PROFILE,"left")

class MotorTests(unittest.TestCase):
    def armed(self):
        c=Controller();c.command(0,0,0);self.assertTrue(c.arm(0)[0]);return c
    def test_forward(self):self.assertEqual(mix(.03,0),(.1,.1))
    def test_left_yaw(self):
        l,r=mix(0,.06);self.assertLess(l,0);self.assertGreater(r,0)
    def test_right_yaw(self):
        l,r=mix(0,-.06);self.assertGreater(l,0);self.assertLess(r,0)
    def test_limit(self):
        for v in (-100,-.2,0,.2,100):
            for w in (-100,0,100):self.assertLessEqual(max(map(abs,mix(v,w))),.15000001)
    def test_neutral_pwm(self):self.assertEqual(pwm(0),1500)
    def test_full_reference_limit(self):self.assertEqual([pwm(-.15),pwm(.15)],[1440,1560])
    def test_no_automatic_arm(self):
        c=Controller();c.command(.03,0,0);self.assertEqual(c.step(.1,.1),(0,0))
    def test_arm_requires_zero(self):
        c=Controller();c.command(.03,0,0);self.assertFalse(c.arm(.1)[0])
    def test_arm_requires_fresh(self):
        c=Controller();c.command(0,0,0);self.assertFalse(c.arm(1)[0])
    def test_ramp(self):
        c=self.armed();c.command(.3,0,.01);self.assertEqual(c.step(.1,.1),(.03,.03))
    def test_stop_immediate(self):
        c=self.armed();c.command(.3,0,.01);c.step(.1,.1);c.command(0,0,.11);self.assertEqual(c.step(.12,.02),(0,0))
    def test_timeout_no_resume(self):
        c=self.armed();c.command(.3,0,.1);c.step(.2,.1);self.assertEqual(c.step(.7,.1),(0,0));c.command(.3,0,.71);self.assertEqual(c.step(.72,.02),(0,0));self.assertFalse(c.armed)
    def test_timeout_in_either_callback_order(self):
        for controller in (Controller, BidirectionalController):
            for timer_first in (False, True):
                with self.subTest(controller=controller.__module__, timer_first=timer_first):
                    c = controller()
                    c.command(0, 0, 0)
                    self.assertTrue(c.arm(0)[0])
                    c.command(.03, 0, .01)
                    self.assertGreater(min(c.step(.05, .05)), 0)
                    if timer_first:
                        c.step(.80, .75)
                    c.command(.03, 0, .80)
                    self.assertEqual(c.step(.81, .01), (0, 0))
                    self.assertFalse(c.armed)
                    self.assertFalse(c.arm(.82)[0])
                    c.command(0, 0, .83)
                    self.assertTrue(c.arm(.84)[0])

    def test_estop_latched(self):
        c=self.armed();c.emergency(True);c.emergency(False);c.command(0,0,.1);self.assertFalse(c.arm(.1)[0]);self.assertTrue(c.reset());self.assertFalse(c.armed)
    def test_estop_cannot_reset_while_held(self):
        c=self.armed();c.emergency(True);self.assertFalse(c.reset())
    def test_nan_disarms(self):
        c=self.armed();c.command(float('nan'),0,.1);self.assertFalse(c.armed)
    def test_reversal_through_zero(self):
        c=self.armed();c.command(.3,0,.01);c.step(.1,.1);c.command(-.3,0,.11);self.assertEqual(c.step(.2,.1),(0,0));self.assertLess(c.step(.3,.1)[0],0)

if __name__=='__main__':unittest.main()
