import copy
import json
import sys
import types
import unittest
from pathlib import Path
from unittest.mock import patch
from esc_config import controller_for_profile, validate,pulse_us,duty_from_us
from motor_core import mix,Controller
from pca_backend import PCAOutput
ROOT=Path(__file__).parent
F=json.loads((ROOT/'preview_forward_only.json').read_text())
B=json.loads((ROOT/'preview_bidirectional.json').read_text())

class ConfigTests(unittest.TestCase):
    def test_forward_only_stop(self):self.assertEqual(pulse_us(0,F,'left'),1000)
    def test_bidirectional_stop(self):self.assertEqual(pulse_us(0,B,'left'),1500)
    def test_one_side_turn(self):self.assertEqual(mix(0,.06,'forward_only'),(0,.1))
    def test_no_reverse(self):
        with self.assertRaises(ValueError):mix(-.01,0,'forward_only')
        with self.assertRaises(ValueError):pulse_us(-.01,F,'left')
    def test_reverse_command_disarms(self):
        c=Controller('forward_only');c.command(0,0,0);c.arm(0);c.command(-.01,0,.1);self.assertFalse(c.armed)
    def test_sweep_nonnegative(self):
        for v in (0,.03,.3,10):
            for w in (-10,-.06,0,.06,10):
                l,r=mix(v,w,'forward_only');self.assertGreaterEqual(min(l,r),0);self.assertLessEqual(max(l,r),.1500001)
    def test_examples_refuse_hardware(self):
        for p in (F,B):
            validate(p)
            with self.assertRaises(ValueError):PCAOutput(p)
    def test_duplicate_channel(self):
        p=copy.deepcopy(F);p['right']['channel']=0
        with self.assertRaises(ValueError):validate(p)
    def test_bad_stop(self):
        p=copy.deepcopy(F);p['left']['neutral_us']=None
        with self.assertRaises(ValueError):validate(p)
    def test_bidirectional_sign(self):
        p=copy.deepcopy(B);p['left']['sign']=-1;self.assertEqual(pulse_us(.1,p,'left'),1460)
    def test_timing(self):self.assertEqual(duty_from_us(1500,50),4915)
    def test_invalid_timing(self):
        with self.assertRaises(ValueError):duty_from_us(float('nan'),50)

class DriverTests(unittest.TestCase):
    def make(self, profile=F):
        self.events=[];self.fail=False
        p=copy.deepcopy(profile)
        # Mock-only configuration, NOT a hardware certification.
        for k in ('esc_spec_verified','oe_default_disabled_verified','independent_stop_verified','signal_loss_stop_verified'):p[k]=True
        events=self.events;test=self
        class OE:
            def __init__(self,*a,**kw):events.append(('oe_init',kw['initial_value']))
            def on(self):events.append(('oe',1))
            def off(self):events.append(('oe',0))
            def close(self):events.append(('oe_close',))
        class Channel:
            def __init__(self,i):self.i=i;self._duty=0
            @property
            def duty_cycle(self):return self._duty
            @duty_cycle.setter
            def duty_cycle(self,v):
                if test.fail and self.i==1:raise OSError('mock I2C failure')
                self._duty=v;events.append(('pwm',self.i,v))
        class PCA:
            def __init__(self,*a,**kw):events.append(('pca_init',));self.channels=[Channel(i) for i in range(16)];self.frequency=50
            def deinit(self):events.append(('pca_close',))
        board=types.ModuleType('board');board.I2C=lambda:types.SimpleNamespace(deinit=lambda:events.append(('i2c_close',)))
        ad=types.ModuleType('adafruit_pca9685');ad.PCA9685=PCA
        gpio=types.ModuleType('gpiozero');gpio.DigitalOutputDevice=OE
        return p,{'board':board,'adafruit_pca9685':ad,'gpiozero':gpio}
    def test_disabled_until_neutral_written(self):
        p,mods=self.make()
        with patch.dict(sys.modules,mods):d=PCAOutput(p)
        self.assertEqual(self.events[0], ('oe_init', True))
        self.assertNotIn(('oe', 0), self.events)
        d.enable_neutral()
        enabled = self.events.index(('oe', 0))
        for channel in (0, 1):
            self.assertIn(('pwm', channel, duty_from_us(1000, 50)), self.events[:enabled])
        d.close()

    def test_bidirectional_profile_and_mock_pwm(self):
        profile, modules = self.make(B)
        profile.update(reverse_limit=.02, neutral_dwell_seconds=.60)
        preview = controller_for_profile(validate(profile))
        hardware = controller_for_profile(validate(profile, hardware=True))
        with patch.dict(sys.modules, modules):
            bridge = PCAOutput(profile)
        self.addCleanup(bridge.close)
        bridge.enable_neutral()
        for controller in (preview, hardware):
            controller.command(0, 0, 0)
            self.assertTrue(controller.arm(0)[0])
        for now, yaw, expected in ((.1, .06, (-.02, .02)), (.2, -.06, (0., 0.)),
                                   (.5, -.06, (0., 0.)), (.79, -.06, (0., 0.)),
                                   (.81, -.06, (.02, -.02))):
            for controller in (preview, hardware):
                controller.command(0, yaw, now)
                self.assertEqual(controller.step(now, .1), expected)
            bridge.write(*hardware.output)
            for index, side in enumerate(('left', 'right')):
                pulse = pulse_us(preview.output[index], profile, side)
                self.assertEqual(bridge.pca.channels[index].duty_cycle, duty_from_us(pulse, 50))

    def test_bidirectional_hardware_requires_explicit_limits(self):
        profile, _ = self.make(B)
        for key, invalid in [('reverse_limit', .05), ('neutral_dwell_seconds', .39)]:
            old = profile.copy()
            del old[key]
            validate(old)
            with self.assertRaises(ValueError):
                validate(old, hardware=True)
            with self.assertRaises(ValueError):
                validate(dict(profile, **{key: invalid}), hardware=True)

    def test_partial_write_fault_disables(self):
        p,mods=self.make()
        with patch.dict(sys.modules,mods):d=PCAOutput(p)
        self.fail=True
        with self.assertRaises(OSError):d.write(.1,.1)
        self.assertEqual(self.events[-1],('oe',1));self.assertTrue(d.failed)
        with self.assertRaises(RuntimeError):d.write(0,0)
        d.close()
    def test_cleanup_oe_before_i2c(self):
        p,mods=self.make()
        with patch.dict(sys.modules,mods):d=PCAOutput(p)
        self.events.clear();d.close();self.assertLess(self.events.index(('oe',1)),self.events.index(('pca_close',)))

if __name__=='__main__':unittest.main()
