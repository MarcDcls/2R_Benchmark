#!/bin/bash

# See:
# https://www.projectgus.com/2011/10/notes-on-ftdi-latency-with-arduino/

sudo tee /sys/bus/usb-serial/devices/ttyUSB0/latency_timer <<< 0


