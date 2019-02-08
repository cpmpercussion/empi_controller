import time

from grove.i2c import Bus

_COMMAND_MODE = 0x80
_DATA_MODE = 0x40
_NORMAL_DISPLAY = 0xA6

_DISPLAY_OFF = 0xAE
_DISPLAY_ON = 0xAF
_INVERSE_DISPLAY = 0xA7
_SET_BRIGHTNESS = 0x81


BasicFont = [[0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00],
             [0x00, 0x00, 0x5F, 0x00, 0x00, 0x00, 0x00, 0x00],
             [0x00, 0x00, 0x07, 0x00, 0x07, 0x00, 0x00, 0x00],
             [0x00, 0x14, 0x7F, 0x14, 0x7F, 0x14, 0x00, 0x00],
             [0x00, 0x24, 0x2A, 0x7F, 0x2A, 0x12, 0x00, 0x00],
             [0x00, 0x23, 0x13, 0x08, 0x64, 0x62, 0x00, 0x00],
             [0x00, 0x36, 0x49, 0x55, 0x22, 0x50, 0x00, 0x00],
             [0x00, 0x00, 0x05, 0x03, 0x00, 0x00, 0x00, 0x00],
             [0x00, 0x1C, 0x22, 0x41, 0x00, 0x00, 0x00, 0x00],
             [0x00, 0x41, 0x22, 0x1C, 0x00, 0x00, 0x00, 0x00],
             [0x00, 0x08, 0x2A, 0x1C, 0x2A, 0x08, 0x00, 0x00],
             [0x00, 0x08, 0x08, 0x3E, 0x08, 0x08, 0x00, 0x00],
             [0x00, 0xA0, 0x60, 0x00, 0x00, 0x00, 0x00, 0x00],
             [0x00, 0x08, 0x08, 0x08, 0x08, 0x08, 0x00, 0x00],
             [0x00, 0x60, 0x60, 0x00, 0x00, 0x00, 0x00, 0x00],
             [0x00, 0x20, 0x10, 0x08, 0x04, 0x02, 0x00, 0x00],
             [0x00, 0x3E, 0x51, 0x49, 0x45, 0x3E, 0x00, 0x00],
             [0x00, 0x00, 0x42, 0x7F, 0x40, 0x00, 0x00, 0x00],
             [0x00, 0x62, 0x51, 0x49, 0x49, 0x46, 0x00, 0x00],
             [0x00, 0x22, 0x41, 0x49, 0x49, 0x36, 0x00, 0x00],
             [0x00, 0x18, 0x14, 0x12, 0x7F, 0x10, 0x00, 0x00],
             [0x00, 0x27, 0x45, 0x45, 0x45, 0x39, 0x00, 0x00],
             [0x00, 0x3C, 0x4A, 0x49, 0x49, 0x30, 0x00, 0x00],
             [0x00, 0x01, 0x71, 0x09, 0x05, 0x03, 0x00, 0x00],
             [0x00, 0x36, 0x49, 0x49, 0x49, 0x36, 0x00, 0x00],
             [0x00, 0x06, 0x49, 0x49, 0x29, 0x1E, 0x00, 0x00],
             [0x00, 0x00, 0x36, 0x36, 0x00, 0x00, 0x00, 0x00],
             [0x00, 0x00, 0xAC, 0x6C, 0x00, 0x00, 0x00, 0x00],
             [0x00, 0x08, 0x14, 0x22, 0x41, 0x00, 0x00, 0x00],
             [0x00, 0x14, 0x14, 0x14, 0x14, 0x14, 0x00, 0x00],
             [0x00, 0x41, 0x22, 0x14, 0x08, 0x00, 0x00, 0x00],
             [0x00, 0x02, 0x01, 0x51, 0x09, 0x06, 0x00, 0x00],
             [0x00, 0x32, 0x49, 0x79, 0x41, 0x3E, 0x00, 0x00],
             [0x00, 0x7E, 0x09, 0x09, 0x09, 0x7E, 0x00, 0x00],
             [0x00, 0x7F, 0x49, 0x49, 0x49, 0x36, 0x00, 0x00],
             [0x00, 0x3E, 0x41, 0x41, 0x41, 0x22, 0x00, 0x00],
             [0x00, 0x7F, 0x41, 0x41, 0x22, 0x1C, 0x00, 0x00],
             [0x00, 0x7F, 0x49, 0x49, 0x49, 0x41, 0x00, 0x00],
             [0x00, 0x7F, 0x09, 0x09, 0x09, 0x01, 0x00, 0x00],
             [0x00, 0x3E, 0x41, 0x41, 0x51, 0x72, 0x00, 0x00],
             [0x00, 0x7F, 0x08, 0x08, 0x08, 0x7F, 0x00, 0x00],
             [0x00, 0x41, 0x7F, 0x41, 0x00, 0x00, 0x00, 0x00],
             [0x00, 0x20, 0x40, 0x41, 0x3F, 0x01, 0x00, 0x00],
             [0x00, 0x7F, 0x08, 0x14, 0x22, 0x41, 0x00, 0x00],
             [0x00, 0x7F, 0x40, 0x40, 0x40, 0x40, 0x00, 0x00],
             [0x00, 0x7F, 0x02, 0x0C, 0x02, 0x7F, 0x00, 0x00],
             [0x00, 0x7F, 0x04, 0x08, 0x10, 0x7F, 0x00, 0x00],
             [0x00, 0x3E, 0x41, 0x41, 0x41, 0x3E, 0x00, 0x00],
             [0x00, 0x7F, 0x09, 0x09, 0x09, 0x06, 0x00, 0x00],
             [0x00, 0x3E, 0x41, 0x51, 0x21, 0x5E, 0x00, 0x00],
             [0x00, 0x7F, 0x09, 0x19, 0x29, 0x46, 0x00, 0x00],
             [0x00, 0x26, 0x49, 0x49, 0x49, 0x32, 0x00, 0x00],
             [0x00, 0x01, 0x01, 0x7F, 0x01, 0x01, 0x00, 0x00],
             [0x00, 0x3F, 0x40, 0x40, 0x40, 0x3F, 0x00, 0x00],
             [0x00, 0x1F, 0x20, 0x40, 0x20, 0x1F, 0x00, 0x00],
             [0x00, 0x3F, 0x40, 0x38, 0x40, 0x3F, 0x00, 0x00],
             [0x00, 0x63, 0x14, 0x08, 0x14, 0x63, 0x00, 0x00],
             [0x00, 0x03, 0x04, 0x78, 0x04, 0x03, 0x00, 0x00],
             [0x00, 0x61, 0x51, 0x49, 0x45, 0x43, 0x00, 0x00],
             [0x00, 0x7F, 0x41, 0x41, 0x00, 0x00, 0x00, 0x00],
             [0x00, 0x02, 0x04, 0x08, 0x10, 0x20, 0x00, 0x00],
             [0x00, 0x41, 0x41, 0x7F, 0x00, 0x00, 0x00, 0x00],
             [0x00, 0x04, 0x02, 0x01, 0x02, 0x04, 0x00, 0x00],
             [0x00, 0x80, 0x80, 0x80, 0x80, 0x80, 0x00, 0x00],
             [0x00, 0x01, 0x02, 0x04, 0x00, 0x00, 0x00, 0x00],
             [0x00, 0x20, 0x54, 0x54, 0x54, 0x78, 0x00, 0x00],
             [0x00, 0x7F, 0x48, 0x44, 0x44, 0x38, 0x00, 0x00],
             [0x00, 0x38, 0x44, 0x44, 0x28, 0x00, 0x00, 0x00],
             [0x00, 0x38, 0x44, 0x44, 0x48, 0x7F, 0x00, 0x00],
             [0x00, 0x38, 0x54, 0x54, 0x54, 0x18, 0x00, 0x00],
             [0x00, 0x08, 0x7E, 0x09, 0x02, 0x00, 0x00, 0x00],
             [0x00, 0x18, 0xA4, 0xA4, 0xA4, 0x7C, 0x00, 0x00],
             [0x00, 0x7F, 0x08, 0x04, 0x04, 0x78, 0x00, 0x00],
             [0x00, 0x00, 0x7D, 0x00, 0x00, 0x00, 0x00, 0x00],
             [0x00, 0x80, 0x84, 0x7D, 0x00, 0x00, 0x00, 0x00],
             [0x00, 0x7F, 0x10, 0x28, 0x44, 0x00, 0x00, 0x00],
             [0x00, 0x41, 0x7F, 0x40, 0x00, 0x00, 0x00, 0x00],
             [0x00, 0x7C, 0x04, 0x18, 0x04, 0x78, 0x00, 0x00],
             [0x00, 0x7C, 0x08, 0x04, 0x7C, 0x00, 0x00, 0x00],
             [0x00, 0x38, 0x44, 0x44, 0x38, 0x00, 0x00, 0x00],
             [0x00, 0xFC, 0x24, 0x24, 0x18, 0x00, 0x00, 0x00],
             [0x00, 0x18, 0x24, 0x24, 0xFC, 0x00, 0x00, 0x00],
             [0x00, 0x00, 0x7C, 0x08, 0x04, 0x00, 0x00, 0x00],
             [0x00, 0x48, 0x54, 0x54, 0x24, 0x00, 0x00, 0x00],
             [0x00, 0x04, 0x7F, 0x44, 0x00, 0x00, 0x00, 0x00],
             [0x00, 0x3C, 0x40, 0x40, 0x7C, 0x00, 0x00, 0x00],
             [0x00, 0x1C, 0x20, 0x40, 0x20, 0x1C, 0x00, 0x00],
             [0x00, 0x3C, 0x40, 0x30, 0x40, 0x3C, 0x00, 0x00],
             [0x00, 0x44, 0x28, 0x10, 0x28, 0x44, 0x00, 0x00],
             [0x00, 0x1C, 0xA0, 0xA0, 0x7C, 0x00, 0x00, 0x00],
             [0x00, 0x44, 0x64, 0x54, 0x4C, 0x44, 0x00, 0x00],
             [0x00, 0x08, 0x36, 0x41, 0x00, 0x00, 0x00, 0x00],
             [0x00, 0x00, 0x7F, 0x00, 0x00, 0x00, 0x00, 0x00],
             [0x00, 0x41, 0x36, 0x08, 0x00, 0x00, 0x00, 0x00],
             [0x00, 0x02, 0x01, 0x01, 0x02, 0x01, 0x00, 0x00],
             [0x00, 0x02, 0x05, 0x05, 0x02, 0x00, 0x00, 0x00]]


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
        for i in range(8):
            self.set_cursor(i, 0)
            self.puts(' ' * 16)

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

    def show_image(self, image):
        from PIL import Image
        import numpy as np

        im = Image.open(image)

        bw = im.convert('1')
        pixels = np.array(bw.getdata())
        page_size = 128 * 8

        self.set_cursor(0, 0)
        for page in range(8):
            start = page_size * page
            end = start + page_size

            for i in range(start, start + 128):
                data = np.packbits(pixels[i:end:128][::-1])[0]
                self.send_data(data)


def main():
    display = GroveOledDisplay128x64()

    display.set_cursor(0, 0)
    display.puts('hello')
    display.set_cursor(1, 4)
    display.puts('world')

if __name__ == "__main__":
    main()