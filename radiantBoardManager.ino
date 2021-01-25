#include <Adafruit_FlashCache.h>
#include <Adafruit_FlashTransport.h>
#include <Adafruit_SPIFlash.h>
#include <Adafruit_SPIFlashBase.h>
#include <flash_devices.h>
#include <PacketSerial.h>

#include <Wire.h>

#define VER_MAJOR 0
#define VER_MINOR 2
#define VER_REV   0
#define VER_ENC ( ((VER_MAJOR & 0xF) << 12) | ((VER_MINOR & 0xF) << 8) | (VER_REV & 0xFF))
// these need to be automated, but it's a pain in the ass
#define DATE_MONTH 1
#define DATE_DAY   25
#define DATE_YEAR  21
#define DATE_ENC (((DATE_YEAR & 0x7F) << 9) | ((DATE_MONTH & 0xF) << 5) | (DATE_DAY & 0x1F))

const uint32_t ident = 'RDBM';
const uint32_t ver = ((DATE_ENC << 16) | (VER_ENC));

#define RADIANTV2

#ifdef RADIANTV2
#define I2C_CLOCK 0x70
#else
#define I2C_CLOCK 0x71
#endif

#ifdef RADIANTV2
#define I2C_GPBASE 0x38
#else
#define I2C_GPBASE 0x20
#endif

#define I2C_DAC_BASE 0x60

const uint8_t i2c_gp[7] = {
    I2C_GPBASE | 0x0,
    I2C_GPBASE | 0x4,
    I2C_GPBASE | 0x2,
    I2C_GPBASE | 0x6,
    I2C_GPBASE | 0x1,
    I2C_GPBASE | 0x5,
    I2C_GPBASE | 0x3
};

// NOTE PACKET FORMAT
// the 'raw' output packet format is
// addr0 addr1 addr2 (len or data)
// addr0 bit 7 indicates write (if 1) or read (if 0)
// addr0 bit 6 indicates board manager (if 1) or FPGA (if 0)
//

uint32_t control_reg = 0;

#define DSTART()      while(!SerialUSB)
#define DPRINT(...)   SerialUSB.print( __VA_ARGS__ )
#define DPRINTLN(...) SerialUSB.println( __VA_ARGS__ )

// Our peripherals are:
// SERCOM1 CB UART (PA30/PA31)

// The default SAMD21 Xplained Pro peripherals are:
// Serial (sercom3) 

Adafruit_FlashTransport_SPI flashTransport(PIN_SPI_FLASHCS, SPI);
Adafruit_SPIFlash flash(&flashTransport);
FatFileSystem pythonfs;

COBSPacketSerial cbIf;
COBSPacketSerial fpIf;
COBSPacketSerial usbIf;

// Simple readline into a buffer of limited size.
// This will *always* read a line, but it only stores up to len chars.
int pyReadline(File *f, char *buf, uint32_t len) {
  char c;
  if (!f->available()) return 0;
  do {
    c = f->read();
    if (len) {
      *buf = c;
      buf++;
      len--;
    }
    if (c == 0xA) break;
  } while (f->available());
  return 1;
}

// Simple unhexlify. If the character's not 0-f, it'll get mangle-interpreted. Oh well.
uint8_t unhexlify(char msn, char lsn) {
  // Unhexlifying something isn't that hard:
  // ASCII 0-9 are 0x30-0x39
  // ASCII A-F are 0x61-0x66
  msn &= 0x4F;
  lsn &= 0x4F;
  // Now mapped to either 0-0x9 or 0x41-0x46
  if (msn & 0x40) msn = msn - 0x41 + 10;
  // Now mapped to 0-0xF
  msn = msn & 0xF;
  if (lsn & 0x40) lsn = lsn - 0x41 + 10;
  lsn = lsn & 0xF;
  // Merge.
  return (msn << 4) | lsn;  
}

uint8_t clockR(uint8_t addr) {
  Wire.beginTransmission(I2C_CLOCK);
  Wire.write(addr);
  if (Wire.endTransmission() != 0) return 0;
  Wire.requestFrom(I2C_CLOCK, 1);
  if (!Wire.available()) return 0;
  return Wire.read();
}

int clockRMW(uint8_t addr, uint8_t reg,  uint8_t mask) {
  uint8_t curval;
  if (mask != 0xFF) {
    Wire.beginTransmission(I2C_CLOCK);
    Wire.write(addr);
    if (Wire.endTransmission() != 0) return -1;
    Wire.requestFrom(I2C_CLOCK, 1);
    if (!Wire.available()) return -1;
    curval = Wire.read();
    curval &= ~mask;
    curval = curval | (reg & mask);
  } else {
    curval = reg;
  }
  Wire.beginTransmission(I2C_CLOCK);
  Wire.write(addr);
  Wire.write(curval);
  if (Wire.endTransmission() != 0) return -1;
  return 0;
}

int clockConfigure(char *fn) {
  char buf[6]; 
  if (!pythonfs.exists(fn)) return -1;
  File clockFile = pythonfs.open(fn, FILE_READ);
  // start in page 0
  clockRMW(0xFF, 0x0, 0xFF);
  // disable outputs
  clockRMW(230, 0x10, 0x10);
  // pause LOL
  clockRMW(241, 0x80, 0xFF);
  // Run the configuration procedure as outlined in the file.
  while (pyReadline(&clockFile, buf, 6)) {
    uint8_t addr = unhexlify(buf[0], buf[1]);
    uint8_t reg = unhexlify(buf[2], buf[3]);
    uint8_t mask = unhexlify(buf[4], buf[5]);
    clockRMW(addr, reg, mask);
  }        
  // validate input clock status
  while (clockR(218) & 0x4);
  clockRMW(49, 0x00, 0x80);
  clockRMW(246, 0x2, 0x2);
  delay(25);
  clockRMW(241, 0x65, 0xFF);
  while (clockR(218) & 0x11);
  uint8_t val = clockR(237);
  val &= 0x3;
  clockRMW(47, val, 0x3);
  val = clockR(236);
  clockRMW(46, val, 0xFF);
  val = clockR(235);
  clockRMW(45, val, 0xFF);
  clockRMW(47, 0x14, 0xFC);
  clockRMW(49, 0x80, 0x80);
  clockRMW(230, 0x00, 0x10);  
  return 0;
}

//COBSPacketSerial bbSerial;
uint16_t myLoop;

#define BM_ERR_STARTUP_I2C    1
#define BM_ERR_STARTUP_PGV10  2
#define BM_ERR_STARTUP_PGV18  3
#define BM_ERR_STARTUP_PGV25  4
#define BM_ERR_STARTUP_PGV26  5
#define BM_ERR_STARTUP_PGV31  6
#define BM_ERR_STARTUP_CLOCK  7
#define BM_ERR_STARTUP_SPI    8
#define BM_ERR_STARTUP_FAT    9

void diedie(uint32_t errcode) {
  // need to let the CB know, probably a repeating packet.
  // talk to Cosmin about this
  while (1) {
    delay(50);
    DPRINT("DIEDIE: ");
    DPRINTLN(errcode);
    digitalWrite(PIN_LED, LOW);
    digitalWrite(PIN_LED2, HIGH);
    delay(50);
    digitalWrite(PIN_LED, HIGH);
    digitalWrite(PIN_LED2, LOW);
  }
}

// these guys are ALWAYS I/Os. The bootloader forces them.
#define GET_ENV10() (PORT->Group[PORTB].IN.reg & PORT_PB02)
#define GET_ENV18() (PORT->Group[PORTA].IN.reg & PORT_PA04)
#define GET_ENV25() (PORT->Group[PORTA].IN.reg & PORT_PA06)
#define GET_ENV26() (PORT->Group[PORTA].IN.reg & PORT_PA02)
#define GET_ENV31() (PORT->Group[PORTB].IN.reg & PORT_PB08)

#define SET_ENV10() PORT->Group[PORTB].OUTSET.reg = PORT_PB02
#define SET_ENV18() PORT->Group[PORTA].OUTSET.reg = PORT_PA04
#define SET_ENV25() PORT->Group[PORTA].OUTSET.reg = PORT_PA06
#define SET_ENV26() PORT->Group[PORTA].OUTSET.reg = PORT_PA02
#define SET_ENV31() PORT->Group[PORTB].OUTSET.reg = PORT_PB08

#define CLR_ENV10() PORT->Group[PORTB].OUTCLR.reg = PORT_PB02
#define CLR_ENV18() PORT->Group[PORTA].OUTCLR.reg = PORT_PA04
#define CLR_ENV25() PORT->Group[PORTA].OUTCLR.reg = PORT_PA06
#define CLR_ENV26() PORT->Group[PORTA].OUTCLR.reg = PORT_PA02
#define CLR_ENV31() PORT->Group[PORTB].OUTCLR.reg = PORT_PB08

// These are the powergood pins
#define PGV10 8
#define PGV18 14
#define PGV25 15
#define PGV26 12
#define PGV31 13

#define MGTDET 16
#define FPGA_DONE 25
#define SD_DETECT 23
#define BM_EN_10MHZ 22

#define BMGPIO2 33

void setup() {  
  // The powergoods all need pullups.
  pinMode(PGV10, INPUT_PULLUP);
  pinMode(PGV18, INPUT_PULLUP);
  pinMode(PGV25, INPUT_PULLUP);
  pinMode(PGV26, INPUT_PULLUP);
  pinMode(PGV31, INPUT_PULLUP);
  // other pullups
  pinMode(MGTDET, INPUT_PULLUP);
  pinMode(SD_DETECT, INPUT_PULLUP);
  // enable clock by default
  digitalWrite(BM_EN_10MHZ, HIGH);
  pinMode(BM_EN_10MHZ, OUTPUT);
  // LEDs
  digitalWrite(PIN_LED, LOW);
  digitalWrite(PIN_LED2, LOW);
  pinMode(PIN_LED, OUTPUT);
  pinMode(PIN_LED2, OUTPUT);
  

  // analog read resolution
  analogReadResolution(16);    
  SerialUSB.begin(9600);
  // hold up 2 seconds to allow USB initialization
  delay(2000);  
  DPRINTLN("RADIANT: Startup.");
  ////////////////////////////////////////////////////
  //
  // Start the Wire periph.
  //
  ////////////////////////////////////////////////////
  Wire.begin();  
  ////////////////////////////////////////////////////
  if (!flash.begin()) diedie(BM_ERR_STARTUP_SPI);
  if (!pythonfs.begin(&flash)) diedie(BM_ERR_STARTUP_FAT);
  ////////////////////////////////////////////////////  
  SPI1.begin();
  // Re-convert the MISO pin back to an output (low)
  // because VINMON never got connected (... whoops).
  digitalWrite(PIN_SPI1_MISO, LOW);
  pinMode(PIN_SPI1_MISO, OUTPUT);
  
  ////////////////////////////////////////////////////  
  // So how can we figure out if we've never been powered on before?
  // Easy: just read the I2C GPIO registers for GP6, which is the *last stage* in the configuration.
  //
  // However, for debugging, we want an ALTERNATE way to force it.
  // So what we do here is look to see if SerialUSB is *already open*. If it is, we go ahead and do
  // everything *anyway*.
  //
  // So if you do *not* want initial board setup to occur (but still want to use SerialUSB, for example)
  // go ahead and make sure to *close* the serial terminal for ~5 seconds after power on.
  // 
  Wire.beginTransmission(i2c_gp[6]);
  // Check the CONFIGURATION register.
  Wire.write(0x3);
  if (Wire.endTransmission() != 0) diedie(BM_ERR_STARTUP_I2C);
  Wire.requestFrom(i2c_gp[6], 1);
  if (Wire.available()) {
    uint8_t c = Wire.read();
    if (c == 0xFF || SerialUSB) {      
      // INITIAL POWERON STARTUP
      //
      // NOTE: In this version we start up everything automatically,
      // rather than starting "quiet." Later we'll use one of the GPIOs
      // to indicate "dude start low power plz"
      //
      DPRINTLN("RADIANT: Doing initial power-on sequence.");
      // 1: Turn on the FPGA supplies, in sequence, waiting for powergood on each.
      SET_ENV10();
      // Wait 5 milliseconds: the soft-start ramp should only be 0.7 ms.
      delay(5);
      if (!digitalRead(PGV10)) diedie(BM_ERR_STARTUP_PGV10);
      SET_ENV18();
      // Wait 5 milliseconds: soft-start ramp is only 1 ms
      delay(5);
      if (!digitalRead(PGV18)) diedie(BM_ERR_STARTUP_PGV18);
      SET_ENV25();
      // Wait 5 milliseconds: soft start ramp is only 1 ms
      delay(5);
      if (!digitalRead(PGV25)) diedie(BM_ERR_STARTUP_PGV25);
      // 2: Turn on the LAB4/trigger supplies...
      SET_ENV26();
      // Wait 5 milliseconds, soft start ramp is <1 ms
      delay(5);
      if (!digitalRead(PGV26)) diedie(BM_ERR_STARTUP_PGV26);
      SET_ENV31();
      delay(5);
      if (!digitalRead(PGV31)) diedie(BM_ERR_STARTUP_PGV31);
      // 3: Set up the clock.
      if (clockConfigure("intclock25.dat")) diedie(BM_ERR_STARTUP_CLOCK);      
      // 4: Set up the LAB4 GPIOs. If we're a RADIANTv1 we need to enable pulls
      //    first.
      for (unsigned int i=0;i<6;i++) {
#ifndef RADIANTV2
        Wire.beginTransmission(i2c_gp[i]);
        Wire.write(0x4);
        Wire.write(0xC0);
        if (Wire.endTransmission() != 0) diedie(BM_ERR_STARTUP_I2C);         
#endif
        Wire.beginTransmission(i2c_gp[i]);
        Wire.write(0x0);
        if (Wire.endTransmission() != 0) diedie(BM_ERR_STARTUP_I2C);
        Wire.requestFrom(i2c_gp[i], 1);
        if (!Wire.available()) diedie(BM_ERR_STARTUP_I2C);
        c = Wire.read();
        // Copy bits 7 and 6 to bits 5 and 4.
        c &= 0xC0;
        c >>= 2;
        // Light the green LED.
        c |= 0x8;
        Wire.beginTransmission(i2c_gp[i]);
        Wire.write(0x1);
        Wire.write(c);
        if (Wire.endTransmission() != 0) diedie(BM_ERR_STARTUP_I2C);
        Wire.beginTransmission(i2c_gp[i]);
        Wire.write(0x3);
        // Drive everything except the top bits.
        Wire.write(0xC0);
        if (Wire.endTransmission() != 0) diedie(BM_ERR_STARTUP_I2C);
      }
      // Step 5: Set up the signal generator GPIO.
      // This one indicates that we're done.            
      // It also needs to be set up a little weird, since it's got complementary pairs.
      // All outputs except bit 7.
      // x010_1000 = 0x28
      Wire.beginTransmission(i2c_gp[6]);
      Wire.write(1);
      Wire.write(0x28);
      if (Wire.endTransmission() != 0) diedie(BM_ERR_STARTUP_I2C);
      Wire.beginTransmission(i2c_gp[6]);
      Wire.write(3);
      Wire.write(0x80);
      if (Wire.endTransmission() != 0) diedie(BM_ERR_STARTUP_I2C);      
    } else {
      // NO INITIAL POWERON STARTUP
      DPRINTLN("RADIANT: Skipping initial poweron.");
    }
  } else diedie(BM_ERR_STARTUP_I2C);  
  Serial.begin(1000000);
  Serial1.begin(1000000);
  cbIf.setStream(&Serial);
  cbIf.setPacketHandler(&onCbPacketReceived);  
  fpIf.setStream(&Serial1);
  fpIf.setPacketHandler(&onFpPacketReceived);
  usbIf.setStream(&SerialUSB);
  usbIf.setPacketHandler(&onCbPacketReceived);
  
  DPRINTLN("RADIANT: startup complete.");
}


unsigned long time_now = 0;
int period = 1000;
bool gpioHigh = false;

void loop() {
  cbIf.update();
  fpIf.update();
  if (SerialUSB) usbIf.update();
  if (millis() - time_now > period) {
    time_now = millis();
    if (gpioHigh) {
        digitalWrite(PIN_LED, LOW);
        gpioHigh = false;
    } else {
        digitalWrite(PIN_LED, HIGH);
        gpioHigh = true;
    }
  }
}

uint32_t getStatus() {
  uint32_t resp;
  resp = 0;
  if (digitalRead(FPGA_DONE)) resp |= 0x1;
  if (digitalRead(MGTDET)) resp |= 0x2;
  if (digitalRead(SD_DETECT)) resp |= 0x4;
  if (digitalRead(PGV10)) resp |= 0x8;
  if (digitalRead(PGV18)) resp |= 0x10;
  if (digitalRead(PGV25)) resp |= 0x20;
  if (digitalRead(PGV26)) resp |= 0x40;
  if (digitalRead(PGV31)) resp |= 0x80;
  // done for now
  return resp;
}

uint8_t tempBuffer[256];

void onCbPacketReceived(const uint8_t *buffer, size_t size) {
  // Packets have to be at least 4 bytes: addr addr addr data (on write).
  if (size < 4) return;
  if (!(buffer[0] & 0x40)) {
    memcpy(tempBuffer, buffer, size);
    // not for us
    if (control_reg & 0x8) tempBuffer[0] |= 0x40;
    fpIf.send(tempBuffer, size);
  } else {
    // interpret it, build response, send back
    uint32_t addr;
    bool wr;
    if (buffer[0] & 0x80) wr = true; else wr  = false;
    // Build address. We ONLY work with 32-bit addresses here.
    // CB board just... shouldn't send anything else.
    addr =  ((buffer[0] & 0x3F) << 16) |
            (buffer[1] << 8) |
            (buffer[2] & 0xFC);
    addr >>= 2;            
    if (!wr) {
      uint32_t rsp;
      switch(addr) {
        // ID
        case 0: rsp = ident; break;
        // VER
        case 1: rsp = ver; break;
        // STATUS
        case 2: rsp = getStatus(); break;
        case 3: rsp = control_reg; break;
        // Analogs
        case 4: rsp = analogRead(A0); break;
        case 5: rsp = analogRead(A1); break;
        case 6: rsp = analogRead(A2); break;
        case 7: rsp = analogRead(A3); break;
        case 8: rsp = analogRead(A4); break;
        // SPI output (no readback)
        case 9: rsp = 0; break;
        case 16:
        case 17:
        case 18:
        case 19:
        case 20:
        case 21:
        case 22:
          Wire.beginTransmission(i2c_gp[(addr-16)]);
          Wire.write(0);
          if (Wire.endTransmission() == 0) {
            Wire.requestFrom(i2c_gp[(addr-16)], 1);
            if (Wire.available()) rsp = Wire.read();
            else rsp = 0xFFFFFFFF;
          } else rsp = 0xFFFFFFFF;
          break;
          // DACs don't have readback
        default:
          rsp = 0;      
      } 
      tempBuffer[0] = buffer[0];
      tempBuffer[1] = buffer[1];
      tempBuffer[2] = buffer[2];
      tempBuffer[3] = rsp & 0xFF;
      tempBuffer[4] = (rsp >> 8) & 0xFF;
      tempBuffer[5] = (rsp >> 16) & 0xFF;
      tempBuffer[6] = (rsp >> 24) & 0xFF;
      cbIf.send(tempBuffer, 7);
      if (SerialUSB) usbIf.send(tempBuffer, 7);
    } else {
      uint32_t val;
      val = buffer[3];
      if (size > 4) val |= buffer[4] << 8;
      if (size > 5) val |= buffer[5] << 16;
      if (size > 6) val |= buffer[6] << 24;
      switch(addr) {
        // 0, 1, and 2 are read-only
        // Control is harder. For now I'm only capturing the burst bit.
        // I'm not convinced I'm going to keep the "blow things up" bits here anyway.
        case 3: control_reg = val & 0x8; break;
        // 4-8 are read-only
        // SPI write. Only the low byte is used.
        // It's software's job to handle the latch enable, which is an I2C GPIO pin.
        case 9: SPI1.transfer(val & 0xFF); break;
        case 16:
        case 17:
        case 18:
        case 19:
        case 20:
        case 21:
        case 22:
          Wire.beginTransmission(i2c_gp[(addr-16)]);
          Wire.write(val & 0xFF);
          Wire.endTransmission();
          break;
        case 32:
        case 33:
        case 34:
        case 35:
        case 36:
        case 37:
        case 38:
        case 39:
        case 40:
        case 41:
        case 42:
        case 43:
        case 44:
        case 45:
        case 46:
        case 47:
        case 48:
        case 49:
        case 50:
        case 51:
        case 52:
        case 53:
        case 54:
        case 55:
          uint8_t ch = (addr-32) % 4;
          uint8_t quad = (addr-32)/4;
          ch = ch << 1;
          // multi-write command
          // byte 1: 0100 0 (ch) 0
          // vref = 1, pd = 00, gx = 0
          // byte 2: 1000 (val >> 8 & 0xF)
          // byte 3: val & 0xFF
          Wire.beginTransmission(I2C_DAC_BASE+quad+2);
          Wire.write(0x40 | ch);
          Wire.write(0x80 | ((val >> 8) & 0xF));
          Wire.write(val & 0xFF);
          Wire.endTransmission();
          break;
        case 56:
        case 57:
          val = val << 4;
          Wire.beginTransmission(I2C_DAC_BASE + (addr - 56) );
          // 2nd byte has command + powerdown stuff
          // So now 010 00 00 0 = 0x40
          // then (val >> 4) & 0xFF
          // then (val << 4) & 0xF0
          Wire.write(0x40);
          Wire.write((val >> 8) & 0xFF);
          Wire.write(val & 0xF0);
          Wire.endTransmission();
          break;
        default: break;          
      }      
      // don't do anything with writes yet
      tempBuffer[0] = buffer[0];
      tempBuffer[1] = buffer[1];
      tempBuffer[2] = buffer[2];
      if (size > 7) tempBuffer[3] = 4;
      else tempBuffer[3] = size - 3;
      cbIf.send(tempBuffer, 4);
      if (SerialUSB) usbIf.send(tempBuffer, 4);
    }
  }
}

void onFpPacketReceived(const uint8_t *buffer, size_t size) {
  // not for us
  memcpy(tempBuffer, buffer, size);
  cbIf.send(tempBuffer, size);
  if (SerialUSB) usbIf.send(tempBuffer, size);
}
