#!/usr/bin/python
import sys
# GPIO


class ShunkMachineControlersReal(object):
    def __init__(self):

        try:
            import RPi.GPIO as GPIO
        except ImportError:
            print("This script must be run on a Raspberry Pi. (RPi.GPIO import failed)")
            sys.exit(1)

        self.setup()

    def setup(self):
        """Specifies pin numbering schema and sets up GPIO channels"""
        # Setup GPIO
        GPIO.setwarnings(False)
        mode = GPIO.getmode()
        if mode is None:
            GPIO.setmode(GPIO.BOARD)
        elif mode == GPIO.BCM:
            GPIO.setup([], GPIO.OUT)
            GPIO.cleanup()
            GPIO.setmode(GPIO.BOARD)

        GPIO.setup(37, GPIO.OUT, initial=1)
        GPIO.setup(40, GPIO.OUT, initial=0)

    def open_chuck(self):
        """Set Pi pins to open chuck."""
        GPIO.output(40, 0)
        GPIO.output(37, 1)

    def close_chuck(self):
        """Set Pi pins to close chuck."""
        GPIO.output(40, 1)
        GPIO.output(37, 0)

    def start_operation(self):
        pass

    def end_operation(self):
        pass