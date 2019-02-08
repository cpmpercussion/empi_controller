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


# Argparse
parser = argparse.ArgumentParser(description='EMPI Controller')
# OSC addresses
parser.add_argument("--clientip", default="localhost", help="The address of output device.")
parser.add_argument("--clientport", type=int, default=5001, help="The port the output device is listening on.")
parser.add_argument("--serverip", default="127.0.0.1", help="The address of this server.")
parser.add_argument("--serverport", type=int, default=5000, help="The port this server should listen on.")
parser.add_argument("--synthip", default="localhost", help="The address of the synth.")
parser.add_argument("--synthport", type=int, default=3000, help="The port the synth.")
parser.add_argument('-d', '--direct', dest='direct_connect', action="store_true", help='Connect Servo to Input Directly.')
args = parser.parse_args()


POTENTIOMETER_PIN = 0
SERVO_PIN = 5
MIN_POT_CHANGE = 10
MIN_SERVO_CHANGE = 5

last_potentiometer_value = -100
last_servo_value = -100
last_rnn_value = 0


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
    """Send a command to the servo. Input is between 0, 1023"""
    global last_servo_value
    val = interp(input, [0, 1], [0, 180])
    # Only write significant changes to servo.
    if (abs(val - last_servo_value) > MIN_SERVO_CHANGE):
        last_servo_value = val
        servo.setAngle(val)


def read_lever():
    """Read a value from the lever and return as float."""
    global last_potentiometer_value
    input_val = knob.value
    if (abs(input_val - last_potentiometer_value) > MIN_POT_CHANGE):
        last_potentiometer_value = input_val
        return interp(input_val, [0, 999], [0, 1])
    else:
        return None



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


def send_sound_command(address, value):
    """Send a sound command back to the interface/synth"""
    osc_sound_client.send_message(address, value)


def send_interface_command(value):
    osc_prediction_client.send_message(INTERFACE_MESSAGE_ADDRESS, value)


def handle_prediction_message(address: str, *osc_arguments) -> None:
    """Handler for OSC messages from the interface"""
    global last_rnn_value
    int_input = osc_arguments[0]
    send_sound_command(PREDICTION_MESSAGE_ADDRESS, int_input) # send to synth
    command_servo(180 * input_value) # send to servo


# OSC Interaction:
INTERFACE_MESSAGE_ADDRESS = "/interface"
PREDICTION_MESSAGE_ADDRESS = "/prediction"


osc_prediction_client = udp_client.SimpleUDPClient(args.clientip, args.clientport)
osc_sound_client = udp_client.SimpleUDPClient(args.synthip, args.synthport)

dispatch = dispatcher.Dispatcher()
dispatch.map(PREDICTION_MESSAGE_ADDRESS, handle_prediction_message)
server = osc_server.ThreadingOSCUDPServer((args.serverip, args.serverport), dispatch)


def interaction_loop():
    input_value = read_lever()
    # Only react to real changes in the potentiometer.
    if input_value is not None:
        send_interface_command(input_value)
        send_sound_command(INTERFACE_MESSAGE_ADDRESS, input_value)
        if args.direct_connect:
            command_servo(input_value)

try:
    # user_thread.start()
    # rnn_thread.start()
    while True:
        interaction_loop()
        lever_text = "Lever: {:10.4f}".format(last_potentiometer_value)
        rnn_text = "RNN: {:10.4f}".format(last_rnn_value)
        display_text(disp, lever_text, line2=rnn_text)
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
