#!/usr/bin/env python
#-*- coding: utf-8 -*-

import subprocess

class CameraSettingValue:
    def __init__(self, brightness=90, exposure_auto=3, focus_auto=1, white_balance_temperature_auto=1, backlight_compensation=1):
        self.set_brightness(brightness)
        self.set_exposure_auto(exposure_auto)
        self.set_focus_auto(focus_auto)
        self.set_white_balance_temperature_auto(white_balance_temperature_auto)
        self.set_backlight_compensation(backlight_compensation)
    
    def set_brightness(self, value):
        subprocess.check_call(f"v4l2-ctl -d /dev/video0 -c brightness={value}", shell=True)
    
    def set_exposure_auto(self, value):
        subprocess.check_call(f"v4l2-ctl -d /dev/video0 -c exposure_auto={value}", shell=True)
    
    def set_focus_auto(self, value):
        subprocess.check_call(f"v4l2-ctl -d /dev/video0 -c focus_auto={value}", shell=True)
    
    def set_white_balance_temperature_auto(self, value):
        subprocess.check_call(f"v4l2-ctl -d /dev/video0 -c white_balance_temperature_auto={value}", shell=True)
    
    def set_backlight_compensation(self, value):
        subprocess.check_call(f"v4l2-ctl -d /dev/video0 -c backlight_compensation={value}", shell=True)

# 예시로 설정값 변경
#camera = CameraSettingValue(brightness=90, exposure_auto=3, focus_auto=1, white_balance_temperature_auto=1, backlight_compensation=1)
