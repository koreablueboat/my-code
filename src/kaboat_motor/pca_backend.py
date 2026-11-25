"""PCA9685 output adapter. Hardware dependencies loaded only on construction.
OE HIGH disables PWM. Independent stop is required: PCA can retain its last
PWM if the host crashes; GPIO cleanup is not an independent watchdog.
"""
import time
from esc_config import validate,pulse_us,duty_from_us


class PCAOutput:
    def __init__(self,profile):
        validate(profile,hardware=True)  # before importing/accessing hardware
        import board
        from adafruit_pca9685 import PCA9685
        from gpiozero import DigitalOutputDevice
        self.profile=profile
        self.oe=self.i2c=self.pca=None
        self.failed=False
        try:
            self.oe=DigitalOutputDevice(profile['oe_gpio_bcm'],active_high=True,initial_value=True)
            self.i2c=board.I2C()
            self.pca=PCA9685(self.i2c,address=profile['i2c_address'],reference_clock_speed=profile['reference_clock_hz'])
            # Dedicated board for ESCs: disable all channels before setting frequency.
            for channel in self.pca.channels:channel.duty_cycle=0
            self.pca.frequency=profile['frequency_hz']
            self.actual_frequency=float(self.pca.frequency)
            self.write(0.,0.)
            self.ready_at = float('inf')
        except Exception:
            self._release()
            raise

    def disable(self):
        self.oe.on()
        self.ready_at = float('inf')

    def enable_neutral(self):
        self.write(0., 0.)
        self.ready_at = time.monotonic() + self.profile['neutral_init_seconds']
        self.oe.off()

    def poll(self,now):
        return not self.failed and now>=self.ready_at

    def write(self,left,right):
        if self.failed:raise RuntimeError('PCA output fault latched; restart required')
        try:
            for side,value in (('left',left),('right',right)):
                us=pulse_us(value,self.profile,side)
                self.pca.channels[self.profile[side]['channel']].duty_cycle=duty_from_us(us,self.actual_frequency)
        except Exception:
            self.failed=True
            if self.oe is not None:self.oe.on()
            raise

    def _release(self):
        # Disable before I2C cleanup; even if I2C is broken OE remains independent.
        try:
            if self.oe is not None:self.oe.on()
        finally:
            try:
                if self.pca is not None:self.pca.deinit()
            finally:
                try:
                    if self.i2c is not None:self.i2c.deinit()
                finally:
                    if self.oe is not None:self.oe.close()  # verified external circuit keeps OE HIGH

    def close(self):
        self._release()
