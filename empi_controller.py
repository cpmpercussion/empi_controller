"""EMPI Controller for Grove hardware """

import math
import time
import sys
import argparse
from threading import Thread
from grove.i2c import Bus
from grove.adc import ADC
import RPi.GPIO as IO
from numpy import interp

# SSD1306 OLED
import Adafruit_GPIO.SPI as SPI
import Adafruit_SSD1306
from PIL import Image
from PIL import ImageDraw
from PIL import ImageFont

# OSC
from pythonosc import dispatcher
from pythonosc import osc_server
from pythonosc import udp_client

POTENTIOMETER_PIN = 0
SERVO_PIN = 5


def display_text(display, line1, line2=""):
    # Set up image and draw some text.
    display.clear()
    width = display.width
    height = display.height
    image = Image.new('1', (width, height))
    draw = ImageDraw.Draw(image)
    draw.text((0, 0),  line1,  font=font, fill=255)
    if line2 is not None:
        draw.text((0, 20), line2, font=font, fill=255)
    # Display image.
    display.image(image)
    display.display()


class GroveRotaryAngleSensor(ADC):
    def __init__(self, channel):
        self.channel = channel
        self.adc = ADC()

    @property
    def value(self):
        return self.adc.read(self.channel)


class GroveServo:
    MIN_DEGREE = 0
    MAX_DEGREE = 180
    INIT_DUTY = 2.5

    def __init__(self, channel):
        IO.setup(channel,IO.OUT)
        self.pwm = IO.PWM(channel,50)
        self.pwm.start(GroveServo.INIT_DUTY)

    def __del__(self):
        self.pwm.stop()

    def setAngle(self, angle):
        # Map angle from range 0 ~ 180 to range 25 ~ 125
        angle = max(min(angle, GroveServo.MAX_DEGREE), GroveServo.MIN_DEGREE)
        tmp = interp(angle, [0, 180], [25, 125])
        self.pwm.ChangeDutyCycle(round(tmp/10.0, 1))


def process_angle_input(value):
    return interp(value, [0, 999], [0, 1])


def command_servo(input=0.5):
    """Send a command to the servo. Input is between 0, 1"""
    val = interp(input, [0, 1], [0, 180])
    servo.setAngle(val)


def read_lever():
    """Read a value from the lever and return as float."""
    val = knob.value()
    return interp(val, [0, 999], [0, 1])


# Init devices:
# 128x32 display with hardware I2C:
RST = 24
disp = Adafruit_SSD1306.SSD1306_128_32(rst=RST)  # , i2c_address=0x3C)
# font = ImageFont.truetype('Minecraftia.ttf', 8)
font = ImageFont.load_default()
# Initialize library.
disp.begin()
disp.clear()
disp.display()

servo = GroveServo(SERVO_PIN)
knob = GroveRotaryAngleSensor(POTENTIOMETER_PIN)

# Display startup message.
display_text(disp, "EMPI Booted", line2="Loading MDRNN.")

# Load some other stuff

def interaction_loop():
    input_value = read_lever()
    command_servo(input_value)
    display_text(disp, "Lever:", line2=str(input_value))

try:
    # user_thread.start()
    # rnn_thread.start()
    while True:
        interaction_loop()
        # if not args.nogui:
        #    root_window.update()
        # if args.callresponse:
        #    monitor_user_action()
except KeyboardInterrupt:
    print("\nCtrl-C received... exiting.")
    # user_thread.join(timeout=1)
    # rnn_thread.join(timeout=1)
    pass
finally:
    display_text(disp, "Shutting down.", line2="Bye.")
    print("\nDone, shutting down.")
