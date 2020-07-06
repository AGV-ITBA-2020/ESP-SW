#include <SoftwareSerial.h>

#if defined(ESP8266) && !defined(D5)
#define D5 (14)
#define D6 (12)
#define D7 (13)
#define D8 (15)
#define TX (1)
#endif

constexpr int IUTBITRATE = 153600;

constexpr SoftwareSerialConfig swSerialConfig = SWSERIAL_8E1;
constexpr SerialConfig hwSerialConfig = SERIAL_8E1;
constexpr bool invert = false;

constexpr int BLOCKSIZE = 16; // use fractions of 256

unsigned long start;
String effTxTxt("eff. tx: ");
String effRxTxt("eff. rx: ");
int txCount;
int rxCount;
int expected;
int rxErrors;
int rxParityErrors;
constexpr int ReportInterval = IUTBITRATE / 8;

HardwareSerial& hwSerial(Serial);
SoftwareSerial serialIUT;
SoftwareSerial logger;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(IUTBITRATE, hwSerialConfig, SERIAL_FULL, 1, invert);
  Serial.swap();
  Serial.setRxBufferSize(2 * BLOCKSIZE);
  logger.begin(9600, SWSERIAL_8N1, -1, TX);
  serialIUT.begin(IUTBITRATE, swSerialConfig, D5, D6, invert, 4 * BLOCKSIZE);
  serialIUT.enableIntTx(false);
  
  start = micros();
  txCount = 0;
  rxCount = 0;
  rxErrors = 0;
  rxParityErrors = 0;

  logger.println("Loopback example for EspSoftwareSerial");
}

unsigned char c = 0;

void loop() {
  // put your main code here, to run repeatedly:
  unsigned char block[2 * BLOCKSIZE];
  
  
  unsigned char inBuf[2 * BLOCKSIZE];
  for (int i = 0; i < BLOCKSIZE; ++i) {
        block[i] = c;
        c = (c + 1) % 256;
        ++txCount;
  }
  serialIUT.write(block, BLOCKSIZE);
  if (serialIUT.overflow()) { logger.println("SoftwareSerial::overflow"); }

  int inCnt;
  uint32_t deadlineStart;  

   deadlineStart = ESP.getCycleCount();
    inCnt = 0;
    while ((ESP.getCycleCount() - deadlineStart) < (1000000 * 10 * BLOCKSIZE) / IUTBITRATE * 8 * ESP.getCpuFreqMHz()) {
        int avail = hwSerial.available();
        inCnt += hwSerial.readBytes(&inBuf[inCnt], min(avail, min(BLOCKSIZE - inCnt, hwSerial.availableForWrite())));
        if (inCnt >= BLOCKSIZE) { break; }
        // wait for more outstanding bytes to trickle in
        if (avail) deadlineStart = ESP.getCycleCount();
    }
    hwSerial.write(inBuf, inCnt);

    // starting deadline for the first bytes to come in
    deadlineStart = ESP.getCycleCount();
    inCnt = 0;
    while ((ESP.getCycleCount() - deadlineStart) < (1000000 * 10 * BLOCKSIZE) / IUTBITRATE * 2 * ESP.getCpuFreqMHz()) {
        int avail = serialIUT.available();
        for (int i = 0; i < avail; ++i)
        {
            unsigned char r = serialIUT.read();
            if (expected == -1) { expected = r; }
            else {
                expected = (expected + 1) % 256;
            }
            if (r != (expected & ((1 << (5 + swSerialConfig % 4)) - 1))) {
                ++rxErrors;
                expected = -1;
            }
            ++rxCount;
            ++inCnt;
        }

        if (inCnt >= BLOCKSIZE) { break; }
        // wait for more outstanding bytes to trickle in
        if (avail) deadlineStart = ESP.getCycleCount();
    }

    const uint32_t interval = micros() - start;
    if (txCount >= ReportInterval && interval) {
        uint8_t wordBits = (5 + swSerialConfig % 4) + static_cast<bool>(swSerialConfig & 070) + 1 + ((swSerialConfig & 0300) ? 1 : 0);
        logger.println(String("tx/rx: ") + txCount + "/" + rxCount);
        const long txCps = txCount * (1000000.0 / interval);
        const long rxCps = rxCount * (1000000.0 / interval);
        logger.print(effTxTxt + wordBits * txCps + "bps, "
            + effRxTxt + wordBits * rxCps + "bps, "
            + rxErrors + " errors (" + 100.0 * rxErrors / (!rxErrors ? 1 : rxCount) + "%)");
        if (0 != (swSerialConfig & 070))
        {
            logger.println(String(" (") + rxParityErrors + " parity errors)");
        }
        else
        {
            logger.println();
        }
        txCount = 0;
        rxCount = 0;
        rxErrors = 0;
        rxParityErrors = 0;
        expected = -1;
        // resync
        delay(static_cast<uint32_t>(1000 * 10 * BLOCKSIZE / IUTBITRATE * 16));
        start = micros();
    }
  
}
