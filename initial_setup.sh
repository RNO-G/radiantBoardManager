#! /bin/sh 
# Initial setup, assuming you have arduino-cli installed :) 


arduino-cli lib install PacketSerial
arduino-cli lib install "AdaFruit SPIFlash"
arduino-cli core install arduino:samd

#really should read this with arduino-cli config 
ARDUINO_DIR=$HOME/Arduino
mkdir -p ${ARDUINO_DIR}/hardware

git clone https://github.com/barawn/osu-boards ${ARDUINO_DIR}/hardware/osu-boards 


