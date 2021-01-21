# RADIANT Board Manager Arduino core

This is the Arduino firmware for the RADIANT board manager.
You need a few requirements:

* OSU Board Hardware repository ( https://github.com/barawn/osu-boards )
* PacketSerial library
* Adafruit_SPIFlash library

Note that this version accepts COBS-encoded packets on either
the raw Serial path (from the controller board) or the SerialUSB
path. However both the paths shouldn't be used at the same time
as return packets are broadcast to both (if SerialUSB is connected).

During debugging, if the *physical* double-tap reset is used
to kick into the bootloader, the board will end up in a "weird"
state, since the I2C devices will be configured, but because of
the prolonged reset, all of the supplies will have turned off.
Because the RADIANT uses the I2C devices to determine if it's
already done the initial power-on configuration, it'll skip
its initial configuration.

To *force* initial configuration while debugging, connect a
serial terminal to the USB port *within 2 seconds* of power-on.
In this configuration, the board will force a reinitialization
at startup no matter what.

Another possibility is to kick the board to the bootloader via
software using the secret key register (not currently implemented).

# Packet format

All packets are COBS-encoded (using the 'default' 0x00 delimiter)
and consist of requests and responses.

* 3-byte big-endian byte start address + write bit
* Data (if a write) or number of bytes requested (if a read)

The first 3 bytes are a big-endian address (most significant bytes
first). The "top bit" (bit 7 in byte 0) indicates a write (if 1)
or a read (if 0). Bit 6 splits up the address space into the
main FPGA (0) and board manager (1).

That is, the FPGA has address space 0x000000-0x3FFFFF and the
board manager has address space 0x400000-0x7FFFFF (again, the
top bit is read/write).

Note: for the board manager, reads/writes should be kept simple
for now - only read/write 32-bit values in full (so all writes
should have 4 bytes, all reads should request 4 bytes).

For the FPGA, there is no restriction: data can be requested/written
to at byte addresses of any length.

