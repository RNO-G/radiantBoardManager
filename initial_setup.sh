#! /bin/sh 
# Initial setup, at least on a Linux platform, if you don't want to point and click on the arduino GUI :)
 


echo -n "Looking for arduino-cli: " 
which arduino-cli
if [[ $? -eq 1 ]] ; then
  echo "no arduino-cli found, make sure it's in your PATH"
  echo "  instructions: https://arduino.github.io/arduino-cli" 
  exit 1
fi
echo Found `arduino-cli version`

ARDUINO_DIR=`arduino-cli config dump | grep user: | sed -s 's/.*://g' | awk '{$1=$1};1'`
echo "\$ARDUINO_DIR is $ARDUINO_DIR" 

echo -n "Looking for uf2conv.py: " 
which uf2conv.py
if [[ $? -eq 1 ]] ; then
  echo "no uf2conv.py found, make sure it's in your PATH"
  echo "  You can get it by downloading"
  echo "  https://raw.githubusercontent.com/microsoft/uf2/master/utils/uf2conv.py"
  echo "  AND"
  echo "  https://raw.githubusercontent.com/microsoft/uf2/master/utils/uf2families.json" 
  echo "  into your PATH."  
  echo "  Yes, you need to put the json file next to the python file."
  exit 1
fi



arduino-cli core update-index  
arduino-cli lib install PacketSerial
#arduino-cli lib install "AdaFruit SPIFlash"
arduino-cli core install arduino:samd

mkdir -p ${ARDUINO_DIR}/hardware
if [ ! -d ${ARDUINO_DIR}/hardware/osu-boards ] ; 
then 
  echo "Cloning osu-boards" 
  git clone https://github.com/barawn/osu-boards ${ARDUINO_DIR}/hardware/osu-boards 
else 
  echo "Checking for osu-boards update" 
  cd ${ARDUINO_DIR}/hardware/osu-boards && git pull 
fi
 


