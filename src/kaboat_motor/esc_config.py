"""Explicit ESC configuration. Examples are not hardware specifications."""
import math
from motor_core import Controller, LIMIT
from reverse_motor_core import Controller as BidirectionalController, NEUTRAL_DWELL, REVERSE_LIMIT


def number(value,low,high):
    return type(value) in (int,float) and math.isfinite(value) and low<=value<=high


def validate(p,hardware=False):
    if p.get('mode') not in ('forward_only','bidirectional'):
        raise ValueError('set mode from the actual ESC manual')
    if not number(p.get('frequency_hz'),25,400):raise ValueError('invalid PWM frequency')
    if type(p.get('i2c_address')) is not int or not 8<=p['i2c_address']<=119:raise ValueError('invalid I2C address')
    if type(p.get('reference_clock_hz')) is not int or not 20000000<=p['reference_clock_hz']<=30000000:raise ValueError('invalid reference clock')
    if not number(p.get('neutral_init_seconds'),0,10):raise ValueError('invalid initialization time')
    if type(p.get('oe_gpio_bcm')) is not int or p['oe_gpio_bcm'] not in (17,22,23,24,25,27):
        raise ValueError('choose an unused permitted OE GPIO')
    channels=[]
    for side in ('left','right'):
        x=p.get(side,{})
        ch=x.get('channel')
        if type(ch) is not int or not 0<=ch<=15:raise ValueError('invalid '+side+' channel')
        channels.append(ch)
        if type(x.get('sign')) is not int or x['sign'] not in (-1,1):raise ValueError('invalid sign')
        if p['mode']=='forward_only' and x['sign']!=1:raise ValueError('forward-only sign must be +1')
        n,f=x.get('neutral_us'),x.get('forward_us')
        if not number(n,500,2500) or not number(f,500,2500) or f<=n:
            raise ValueError('neutral/forward pulse widths must be verified and increasing')
        rev=x.get('reverse_us')
        if p['mode']=='bidirectional' and (not number(rev,500,2500) or rev>=n):
            raise ValueError('bidirectional reverse pulse must be below neutral')
        if p['mode']=='forward_only' and rev is not None:raise ValueError('no reverse pulse for forward-only')
        if f*p['frequency_hz']>=1000000:raise ValueError('pulse exceeds period')
    if channels[0]==channels[1]:raise ValueError('left/right channels must differ')
    if p['mode'] == 'bidirectional':
        # Older preview profiles used the built-in envelope. Hardware needs explicit values.
        limit = p.get('reverse_limit', None if hardware else REVERSE_LIMIT)
        dwell = p.get('neutral_dwell_seconds', None if hardware else NEUTRAL_DWELL)
        if not number(limit, 0, REVERSE_LIMIT) or limit == 0:
            raise ValueError('reverse_limit must be positive and within the preview envelope')
        if type(dwell) not in (int, float) or not math.isfinite(dwell) or dwell < NEUTRAL_DWELL:
            raise ValueError('neutral_dwell_seconds must meet the minimum neutral dwell')
    if hardware:
        for flag in ('esc_spec_verified','oe_default_disabled_verified','independent_stop_verified','signal_loss_stop_verified'):
            if p.get(flag) is not True:raise ValueError('hardware refused: '+flag+' is not true')
    return p


def controller_for_profile(profile):
    """Select a mixer from a validated ESC profile for either output backend."""
    if profile['mode'] == 'bidirectional':
        return BidirectionalController(
            reverse_limit=profile.get('reverse_limit', REVERSE_LIMIT),
            neutral_dwell=profile.get('neutral_dwell_seconds', NEUTRAL_DWELL),
        )
    return Controller(profile['mode'])


def pulse_us(value,p,side):
    if not math.isfinite(value) or abs(value)>LIMIT+1e-9:raise ValueError('output outside starter limit')
    x=p[side];q=value*x['sign']
    if p['mode']=='forward_only' and q<0:raise ValueError('reverse forbidden')
    if q>=0:return x['neutral_us']+q*(x['forward_us']-x['neutral_us'])
    return x['neutral_us']+(-q)*(x['reverse_us']-x['neutral_us'])


def duty_from_us(microseconds,frequency):
    if not number(microseconds,500,2500) or not number(frequency,25,400):raise ValueError('invalid timing')
    result=round(microseconds*frequency*65536/1000000)
    if not 0<result<65535:raise ValueError('pulse exceeds period')
    return result
