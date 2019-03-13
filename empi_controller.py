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
import numpy as np

IO.setwarnings(False)
IO.setmode(IO.BCM)

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

_COMMAND_MODE = 0x80
_DATA_MODE = 0x40
_NORMAL_DISPLAY = 0xA6

_DISPLAY_OFF = 0xAE
_DISPLAY_ON = 0xAF
_INVERSE_DISPLAY = 0xA7 
_SET_BRIGHTNESS = 0x81

BasicFont = []

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

class GroveOledDisplay128x64(object):
    HORIZONTAL = 0x00
    VERTICAL = 0x01
    PAGE = 0x02

    def __init__(self, bus=None, address=0x3C):
        self.bus = Bus(bus)
        self.address = address

        self.off()
        self.inverse = False
        self.mode = self.HORIZONTAL

        self.width = 128 # test
        self.height = 64 # test
        self._pages = self.height//8 # test
        self._buffer = [0]*(self.width*self._pages) # test

        self.clear()
        self.on()

    def on(self):
        self.send_command(_DISPLAY_ON)

    def off(self):
        self.send_command(_DISPLAY_OFF)

    def send_command(self, command):
        self.bus.write_byte_data(self.address, _COMMAND_MODE, command)

    def send_data(self, data):
        self.bus.write_byte_data(self.address, _DATA_MODE, data)

    def send_commands(self, commands):
        for c in commands:
            self.send_command(c)

    def clear(self):
        self.off()
        self._buffer = [0]*(self.width*self._pages)
        self.adf_display()
        self.on()
        self.set_cursor(0, 0)

    @property
    def inverse(self):
        return self._inverse

    @inverse.setter
    def inverse(self, enable):
        self.send_command(_INVERSE_DISPLAY if enable else _NORMAL_DISPLAY)
        self._inverse = enable

    @property
    def mode(self):
        return self._mode

    @mode.setter
    def mode(self, mode):
        self.send_command(0x20)
        self.send_command(mode)
        self._mode = mode

    def set_cursor(self, row, column):
        self.send_command(0xB0 + row)
        self.send_command(0x00 + (8*column & 0x0F))
        self.send_command(0x10 + ((8*column>>4)&0x0F))

    def putc(self, c):
        C_add = ord(c)
        if C_add < 32 or C_add > 127:     # Ignore non-printable ASCII characters
            c = ' '
            C_add = ord(c)

        for i in range(0, 8):
            self.send_data(BasicFont[C_add-32][i])

    def puts(self, text):
        for c in text:
            self.putc(c)

    def adf_image(self, image):
        """Set buffer to value of Python Imaging Library image.  The image should
        be in 1 bit mode and a size equal to the display size.
        """
        if image.mode != '1':
            raise ValueError('Image must be in mode 1.')
        imwidth, imheight = image.size
        if imwidth != 128 or imheight != 64:
            raise ValueError('Image must be same dimensions as display (128x64).')
        # Grab all the pixels from the image, faster than getpixel.
        pix = image.load()
        # Iterate through the memory pages
        index = 0
        for page in range(self._pages):
            # Iterate through all x axis columns.
            for x in range(self.width):
                # Set the bits for the column of pixels at the current position.
                bits = 0
                # Don't use range here as it's a bit slow
                for bit in [0, 1, 2, 3, 4, 5, 6, 7]:
                    bits = bits << 1
                    bits |= 0 if pix[(x, page*8+7-bit)] == 0 else 1
                # Update buffer byte and increment to next byte.
                self._buffer[index] = bits
                index += 1

    def adf_display(self):
        """Write display buffer to physical display."""
        # self.command(SSD1306_COLUMNADDR)
        # self.command(0)              # Column start address. (0 = reset)
        # self.command(self.width-1)   # Column end address.
        # self.command(SSD1306_PAGEADDR)
        # self.command(0)              # Page start address. (0 = reset)
        # self.command(self._pages-1)  # Page end address.
        # Write buffer data.
        for i in range(0, len(self._buffer), 16):
            control = 0x40   # Co = 0, DC = 0
            #self._i2c.writeList(control, self._buffer[i:i+16])
            #self._bus.write_i2c_block_data(self._address, register, data)
            #self.bus.write_byte_data(self.address, _DATA_MODE, data)
            self.bus.write_i2c_block_data(self.address, _DATA_MODE, self._buffer[i:i+16])


    def show_image(self, image):
        #im = image  #Image.open(image)
        #bw = im.convert('1')
        pixels = np.array(image.getdata())
        #pixels = image.load()
        page_size = 128 * 8
        self.set_cursor(0, 0)
        # iterate through pages
        for page in range(8):
            start = page_size * page
            end = start + page_size
            # iterate through x-axis columns
            for i in range(start, start + 128):
                data = np.packbits(pixels[i:end:128][::-1])[0]
                self.send_data(data)


def display_text(display, line1, line2=""):
    # Set up image and draw some text.
    #display.clear()
    # display.set_cursor(0, 0)
    # display.puts(line1)
    # display.set_cursor(1, 0)
    # display.puts(line2)
    #width = display.width
    #height = display.height
    image = Image.new('1', (128, 64))
    draw = ImageDraw.Draw(image)
    draw.text((0, 0),  line1,  font=font, fill=255)
    if line2 is not None:
        draw.text((0, 20), line2, font=font, fill=255)
    # Display image.
    #display.show_image(image)
    display.adf_image(image)
    display.adf_display()
    #display.display()


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
servo = GroveServo(SERVO_PIN)
knob = GroveRotaryAngleSensor(POTENTIOMETER_PIN)
disp = GroveOledDisplay128x64()
#disp.clear()
# font = ImageFont.truetype('Minecraftia.ttf', 8)
font = ImageFont.load_default()


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

time_since_last_screen_update = time.time()


def update_screen():
    """ Only update screen 20fps """
    global time_since_last_screen_update
    now = time.time()
    if (now - time_since_last_screen_update > 0.05):
        lever_text = "Lever: {:10.4f}".format(last_potentiometer_value)
        rnn_text = "RNN: {:10.4f}".format(last_rnn_value)
        display_text(disp, lever_text, line2=rnn_text)
        print("Updated Screen:", lever_text, "took:", time.time() - now)
        time_since_last_screen_update = time.time()


print("Starting EMPI Controller.")
# Display startup message.
display_text(disp, "EMPI Booted", line2="Loading MDRNN.")
time.sleep(0.5)
print("Starting Interaction Loop:")
display_text(disp, "Starting Interaction.", line2="")
time.sleep(0.5)

try:
    # user_thread.start()
    # rnn_thread.start()
    while True:
        interaction_loop()
        update_screen()
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
