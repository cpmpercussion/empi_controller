"""EMPI Controller for Grove hardware """

import math
import time
import sys
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

servo = GroveServo
knob = GroveRotaryAngleSensor


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

# Display init
# 128x32 display with hardware I2C:
RST = 24
disp = Adafruit_SSD1306.SSD1306_128_32(rst=RST)  # , i2c_address=0x3C)
# font = ImageFont.truetype('Minecraftia.ttf', 8)
font = ImageFont.load_default()
# Initialize library.
disp.begin()
disp.clear()
disp.display()

display_text(disp, "EMPI Booted and Loading MDRNN.")

# # Set up image and draw some text.
# width = disp.width
# height = disp.height
# image = Image.new('1', (width, height))
# draw = ImageDraw.Draw(image)
# draw.text((0, 0),    'Hello',  font=font, fill=255)
# draw.text((0, 20), 'World!', font=font, fill=255)
# # Display image.
# disp.image(image)
# disp.display()
