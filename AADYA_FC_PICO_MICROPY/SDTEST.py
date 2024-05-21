# SD Card Testing
# Author: Ameya Marakarkandy
# Last Updated: 21/05/2024

# ISSUE:
# SPI PIN DEFINITIONS NOT FOLLOWED

from machine import Pin,I2C, UART, SPI, ADC, PWM
import uos, SDcard_lib
cs = Pin(9, Pin.OUT, value = 1)
spi = SPI(0,
          baudrate=1000000,
          polarity=0,
          phase=0,
          bits=8,
          firstbit=SPI.MSB,
          sck=Pin(6),
          mosi=Pin(7),
          miso=Pin(16))


sd = SDcard_lib.SDCard(spi, cs)
vfs = uos.VfsFat(sd)
uos.mount(vfs, "/sd")

