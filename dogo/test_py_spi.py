import spidev
import serial
import time
import binascii
import random
import struct
import threading
import math
import numpy as np
import sys
import RPi.GPIO as GPIO


spi0_tx = []

for i in range(0,132):
    spi0_tx.append(i)

print(spi0_tx)



spi = spidev.SpiDev()
spi.open(0, 0)
spi.max_speed_hz = 1000000;
checksum_spi0 = 0;
spi0_data=[];
spi0_rx=[];

spi1 = spidev.SpiDev()
spi1.open(0, 1)
spi1.max_speed_hz = 1000000;
checksum_spi1 = 0;

while True:
    
    spi0_tx[131] = random.randint(0, 99)
    # spi0_tx[131] = 55;
    # print(spi0_tx)
    spi0_rx = spi.xfer3(spi0_tx);
    print(spi0_rx)
    time.sleep(0.001)