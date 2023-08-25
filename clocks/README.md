# RADIANT clock file generation

RADIANT internal clocks are generated from the onboard clock generator
using the Clock Builder Pro software.

IMPORTANT NOTE: 
Since v2.16, the board manager no longer reads the clock configuration off SPI flash, instead embedding it in the binary. 
So the instructions here are bifurcated for changing the clock configuration on the board manager or on circuitpython.

To change the onboard clock generation:

1. Use the ClockBuilderPro software:

https://www.skyworksinc.com/en/application-pages/clockbuilder-pro-software
and open the "slabtimeproj" file. You may want to change its name for yourself.
Change the clock configuration however you want. CLK0/CLK1/CLK2 should all
be the same, and they are 1/128th of the LAB4D sampling speed. CLK3 should
be the same frequency as the input clock.

2. "Export" the configuration by finishing the configuration and clicking on
"Export", then going to the Register File tab, then export a C Code
Header File. Save that file to some header name.

FOR CHANGING THE CLOCK ON THE BOARD MANAGER (since v2.16, see below for legacy method): 

3. Edit generateClockFile.c to include the file you just saved. Compile that
file (just "make generateClockFile" should work, no makefile needed), and
run that executable, dumping its output to a file. Probably name it
something like clocks_25.h 

4. Modify radiantBoardManager.ino to include the file you want for the desired sample rate (look for things like #if SAMPLE_RATE=) 

5. If necessary, change the sample rate. 





FOR CHANGING THE CLOCK IN CIRCUITPYTHON (this used to have an effect on the board manager as well, but no longer). 

3. Edit generateClockFile.c to include the file you just saved. Compile that
file (just "make generateClockFile" should work, no makefile needed), and
run that executable, dumping its output to a file. Probably name it
"intclock25.dat".

4. Switch to the CircuitPython build to access the onboard filesystem of the
board manager. This can be done by kicking into the RADIANT bootloader:
connect the RADIANT via USB, double-press the reset button quickly, and
then dragging radpy_v2.uf2 onto the CIRCUITPY disk from here:

https://github.com/RNO-G/radiant-circuitpython/releases/tag/v0r1

and then resetting again. Maybe power cycle, but reset should work.

5. Open the RADIANT disk that should have showed up once you reset/cycle.
Replace "intclock25.dat" with your new clock configuration.

6. Before v2.16, the board manager (once loaded back on) would read this file off SPI flash, but no longer. 
