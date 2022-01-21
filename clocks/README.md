# RADIANT clock file generation

RADIANT internal clocks are generated from the onboard clock generator
using the Clock Builder Pro software.

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

6. Kick the RADIANT back to the bootloader (double-press reset), reload
the RADIANT board manager firmware, and your changes should be present.