curl -sL https://github.com/Seeed-Studio/grove.py/raw/master/install.sh | sudo bash -s -
sudo apt-get install -y python-smbus
sudo apt-get install -y i2c-tools

# Adafruit small OLED driver:
sudo python -m pip install --upgrade pip setuptools wheel
git clone https://github.com/adafruit/Adafruit_Python_SSD1306.git
cd Adafruit_Python_SSD1306
sudo python setup.py install

# pip3 install Adafruit-SSD1306