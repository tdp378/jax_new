# /*****************************************************************************
# * | File        :   lcdconfig.py
# * | Function    :   Hardware underlying interface for Jax (ROS 2 Humble)
# ******************************************************************************/

import os
import sys
import time
import logging
import numpy as np

# Mocking logic for testing on non-Pi systems
try:
    import RPi.GPIO as GPIO
    import spidev
except ImportError:
    import types
    # Mock for RPi.GPIO
    class MockGPIO:
        BCM, OUT, IN, HIGH, LOW = 11, 1, 0, 1, 0
        def setmode(self, mode): pass
        def setwarnings(self, flag): pass
        def setup(self, pin, mode, initial=None): pass
        def output(self, pin, value): pass
        def input(self, pin): return 0
        def cleanup(self): pass
        def PWM(self, pin, freq):
            class MockPWM:
                def start(self, duty): pass
                def stop(self): pass
                def ChangeDutyCycle(self, duty): pass
                def ChangeFrequency(self, freq): pass
            return MockPWM()
    GPIO = MockGPIO()
    # Mock for spidev
    class MockSpi:
        def __init__(self, bus=0, device=0):
            self.max_speed_hz = 0
            self.mode = 0
        def open(self, bus, device): pass
        def writebytes(self, data): pass
        def close(self): pass
    spidev = types.ModuleType("spidev")
    spidev.SpiDev = MockSpi

class RaspberryPi:
    def __init__(self, spi=spidev.SpiDev(0,0), spi_freq=40000000, rst=27, dc=25, bl=18, bl_freq=1000):
        self.np = np
        self.RST_PIN, self.DC_PIN, self.BL_PIN = rst, dc, bl
        self.SPEED, self.BL_freq = spi_freq, bl_freq
        self.GPIO = GPIO
        self.GPIO.setmode(self.GPIO.BCM)
        self.GPIO.setwarnings(False)
        self.GPIO.setup(self.RST_PIN, self.GPIO.OUT)
        self.GPIO.setup(self.DC_PIN, self.GPIO.OUT)
        self.GPIO.setup(self.BL_PIN, self.GPIO.OUT)
        self.GPIO.output(self.BL_PIN, self.GPIO.HIGH)        
        self.SPI = spi
        if self.SPI is not None:
            self.SPI.max_speed_hz = spi_freq
            self.SPI.mode = 0b00

    def digital_write(self, pin, value): self.GPIO.output(pin, value)
    def digital_read(self, pin): return self.GPIO.input(pin)
    def delay_ms(self, delaytime): time.sleep(delaytime / 1000.0)
    def spi_writebyte(self, data):
        if self.SPI is not None: self.SPI.writebytes(data)

    def module_init(self):
        self.GPIO.setup(self.RST_PIN, self.GPIO.OUT)
        self.GPIO.setup(self.DC_PIN, self.GPIO.OUT)
        self.GPIO.setup(self.BL_PIN, self.GPIO.OUT)
        self._pwm = self.GPIO.PWM(self.BL_PIN, self.BL_freq)
        self._pwm.start(100)
        return 0

    def module_exit(self):
        if self.SPI is not None: self.SPI.close()
        self.GPIO.output(self.RST_PIN, 1)
        self.GPIO.output(self.DC_PIN, 0)        
        self._pwm.stop()
        self.GPIO.output(self.BL_PIN, 1)
        self.GPIO.cleanup()