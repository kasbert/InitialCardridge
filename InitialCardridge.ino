// Initial Cardridge  - C64 Cardridge with a Arduino Mega 2560
//
// Some code from EEPROM Programmer by Written K Adcock.
//
//
// This software presents a 115200-8N1 serial port.
//
// V                                      - prints the version string
// R[hex address]                         - reads 16 bytes of data from the DPRAM
// W[hex address]:[data in two-char hex]  - writes up to 16 bytes of data to the DPRAM
// :<size><offset><type><data><checksum>  - writes intel hex line
// T<four byte hex address><size in hex>  - Transfer from DPRAM $0100 to C64 RAM
// F<four byte hex address><size in hex>  - Transfer from C64 RAM to DPRAM $0300
// ...
// load - load file from SD
// run - run basic
// NMI
// RESET
// GAME=<01>
// EXROM=<01>

/*
   TODO
   - Cleanup
   - Make transfer.asm wait for commands
   - Wait for transfer.asm to perform commands
   - D64 disk emulation with kernal hooks
   - Add monitor (mem dump, disasm etc.) to serial
   - Use double buffering in transfers
   - Store machine state (registers, cia etc.) to $df00
   - Load Vice snapshot to memory
   - Store RAM & state to SD or serial (as Vice snapshot)
   - Load state from a SD or serial (as Vice snapshot)
   - Add pin and command for pressing joystic button
   - Copy startup/run code to ram
   - Use Ultimax mode for more fun
   - Handle filenames like OK and ERR
   - Write and read setting from EEPROM
*/

//#include <avr/pgmspace.h>
#include <avr/io.h>
#include <SPI.h>
#include <SdFat.h>
#include "XModem.h"

#include "commands.h"
#include "transfer.h"

#include "diskimage.h"

PROGMEM const
#include "src-c64/control_bin.h"
PROGMEM const
#include "src-c64/browser_prg.h"
//Another try
//#define CONTROL_PROGMEM_ADDR 0x30000
//#define BROWSER_PROGMEM_ADDR 0x30400

const char hexa[] =
{
  '0', '1', '2', '3', '4', '5', '6', '7',
  '8', '9', 'a', 'b', 'c', 'd', 'e', 'f'
};

const char version_string[] = {"Initial Cardridge 0.9"};

// See SetAddress()
static const int kPin_Addr0   = 8; // PH5
static const int kPin_Addr1   = 9; // PH6
static const int kPin_Addr2   = 10; // PB4
static const int kPin_Addr3   = 11; // PB5
static const int kPin_Addr4   = 15; // PJ0
static const int kPin_Addr5   = 14; // PJ1
static const int kPin_Addr6   = 17; // PH0
static const int kPin_Addr7   = 16; // PH1
static const int kPin_Addr8   = 19; // PD2
static const int kPin_Addr9   = 18; // PD3
static const int kPin_Addr10  = 4; // NOT USED
static const int kPin_Addr11  = 5; // NOT USED
//static const int kPin_Addr12  = ;
//static const int kPin_Addr13  = ;
//static const int kPin_Addr14  = ;

static const int kPin_Data0   = 22;
static const int kPin_Data1   = 23;
static const int kPin_Data2   = 24;
static const int kPin_Data3   = 25;
static const int kPin_Data4   = 26;
static const int kPin_Data5   = 27;
static const int kPin_Data6   = 28;
static const int kPin_Data7   = 29;

//static const int kPin_nCE     = 5; // TODO not there
static const int kPin_nOE     = 2; // TODO not there
static const int kPin_nWE     = 3;

static const int kPin_nNMI    = 7;
static const int kPin_nNMI2    = 62; // 7 was a poor choice. It has no PCINT. (62 = A8)
static const int kPin_nRESET  = 6;
static const int kPin_nEXROM  = 13;
static const int kPin_nGAME   = 12;

static const int kPin_nSW1    = 65;
static const int kPin_nSW2    = 67;

static const int kPin_nSD_SEL    = 53;

//static const int kPin_WaitingForInput  = 48;
static const int kPin_LED_Red = 50;  // TODO not there
static const int kPin_LED_Grn = 52; // TODO not there

#define DEBUG(x) Serial.print(x)
#define DEBUGLN(x) Serial.println(x)

#define NOP __asm__ __volatile__ ("nop\n\t") // delay 62.5ns on a 16MHz AtMega

#define PATH_MAX 128

static const int kMaxBufferSize = 16;
uint8_t buffer[TRANSFER_MAX]; // Max is 256

char cmd_buffer[PATH_MAX]; // >kMaxBufferSize * 2 + 20 . strings received from the controller will go in here

bool exrom = HIGH;
bool game = HIGH;

// Xmodem
File recvFile2;
long maxSize, currentSize;
File sendFile2;
uint16_t loadAddr;


void ReadString();
bool ReadDPRAM();
bool WriteDPRAM();
bool NMI();
bool RESET();
uint8_t CalcBufferChecksum(int size);


SdFat SD;

static File root;
static File currentDir;
static char currentPath[PATH_MAX];
static volatile bool sendDotDot;
static volatile int NMICount;
static volatile bool detectNMI = 1;


void setup()
{
  digitalWrite(kPin_nNMI, HIGH); pinMode(kPin_nNMI, INPUT); // Internal pull-up
  digitalWrite(kPin_nRESET, HIGH); pinMode(kPin_nRESET, INPUT); // Internal pull-up
  pinMode(kPin_nEXROM, OUTPUT); digitalWrite(kPin_nEXROM, exrom);
  pinMode(kPin_nGAME, OUTPUT); digitalWrite(kPin_nGAME, game);

  pinMode(kPin_nSW1, INPUT_PULLUP);
  pinMode(kPin_nSW2, INPUT_PULLUP);

  //pinMode(kPin_WaitingForInput, OUTPUT); digitalWrite(kPin_WaitingForInput, HIGH);
  pinMode(kPin_LED_Red, OUTPUT); digitalWrite(kPin_LED_Red, LOW);
  pinMode(kPin_LED_Grn, OUTPUT); digitalWrite(kPin_LED_Grn, LOW);

  // address lines are ALWAYS outputs
  pinMode(kPin_Addr0,  OUTPUT);
  pinMode(kPin_Addr1,  OUTPUT);
  pinMode(kPin_Addr2,  OUTPUT);
  pinMode(kPin_Addr3,  OUTPUT);
  pinMode(kPin_Addr4,  OUTPUT);
  pinMode(kPin_Addr5,  OUTPUT);
  pinMode(kPin_Addr6,  OUTPUT);
  pinMode(kPin_Addr7,  OUTPUT);
  pinMode(kPin_Addr8,  OUTPUT);
  pinMode(kPin_Addr9,  OUTPUT);
  pinMode(kPin_Addr10, OUTPUT);
  pinMode(kPin_Addr11, OUTPUT);
  //pinMode(kPin_Addr12, OUTPUT);
  //pinMode(kPin_Addr13, OUTPUT);
  //pinMode(kPin_Addr14, OUTPUT);

  // control lines are ALWAYS outputs
  //pinMode(kPin_nCE, OUTPUT); digitalWrite(kPin_nCE, LOW); // might as well keep the chip enabled ALL the time
  pinMode(kPin_nOE, OUTPUT); digitalWrite(kPin_nOE, HIGH);
  pinMode(kPin_nWE, OUTPUT); digitalWrite(kPin_nWE, HIGH); // not writing

  SetDataLinesAsInputs();
  SetAddress(0x200);

  Serial.begin(115200);
  Serial.setTimeout(50);
  Serial.println();
  Serial.println(version_string);

  init_c64();

  //Connect A8 to NMI
  pinMode(kPin_nNMI2, INPUT_PULLUP);
  // does not work attachInterrupt(digitalPinToInterrupt(kPin_nNMI2), blink, CHANGE);
  enableDetectNMI();
  PCMSK2 |= (1 << PCINT17);
  PCMSK2 |= (1 << PCINT19);
  PCMSK2 |= (1 << PCINT21);
}

void loop()
{
  while (true)
  {
    //digitalWrite(kPin_WaitingForInput, HIGH);
    ReadString();
    //digitalWrite(kPin_WaitingForInput, LOW);

    digitalWrite(kPin_LED_Red, HIGH);
    detectNMI = 0; //disableDetectNMI();
    handleSerialCommand();
    detectNMI = 1; //enableDetectNMI();
    digitalWrite(kPin_LED_Red, LOW);
  }
}

void handleSerialCommand() {
  bool ok = false ;
  if (!strcmp((char*)cmd_buffer, "ls")) { // SD
    listFiles();
    ok = true;
  } else if (!strcmp((char*)cmd_buffer, "tree")) { // SD
    root = SD.open("/");
    buffer[0] = '/';
    tree(root, (char*)buffer + 1, (char*)buffer);
    root.close();
    ok = true;
  } else if (!strncmp((char*)cmd_buffer, "cd ", 3)) { // SD
    ok = changeDir((const char*)cmd_buffer + 3);
  } else if (!strcmp((char*)cmd_buffer, "pwd")) { // SD
    Serial.println(currentPath);
  } else if (!strncmp((char*)cmd_buffer, "rx ", 3)) { // Store to SD
    ok = recvFileX((char*)cmd_buffer + 3);
  } else if (!strncmp((char*)cmd_buffer, "sx ", 3)) { // Read from SD
    ok = sendFileX((char*)cmd_buffer + 3);
  } else if (!strncmp((char*)cmd_buffer, "rm ", 3)) { // SD
    ok = removeFileSD((char*)cmd_buffer + 3);
  } else if (!strncmp((char*)cmd_buffer, "load ", 5)) { // load from SD to C64
    ok = loadFileSD((char*)cmd_buffer + 5);
  } else if (!strcmp((char*)cmd_buffer, "run")) { // Run in C64
    ok = runBasic();
  } else if (!strncmp((char*)cmd_buffer, "p ", 2)) { // Print text
    ok = printText((char*)cmd_buffer + 2);
  } else if (!strncmp((char*)cmd_buffer, "autosleep ", 10)) { // Sleep in playlist
    ok = playlistAutoSleep(atoi((char*)cmd_buffer + 10));
  } else if (!strncmp((char*)cmd_buffer, "sleep ", 6)) { // Sleep in playlist
    ok = playlistSleep(atoi((char*)cmd_buffer + 6));
  } else if (!strcmp((char*)cmd_buffer, "lc")) {
    ok = loadControlSD() || loadControl();
  } else if (!strcmp((char*)cmd_buffer, "lb")) {
    ok = loadBrowser();
  } else if (!strcmp((char*)cmd_buffer, "l")) {
    ok = loadFileSD("BOULDE~2.PRG"); // for quick testing
    //ok = listD64("still_strong.d64"); // for quick testing
  } else if (!strcmp((char*)cmd_buffer, "reset")) { // Reset C64 fast
    ok = fastReset();
  } else if (!strcmp((char*)cmd_buffer, "RESET")) {
    ok = RESET();
  } else if (!strcmp((char*)cmd_buffer, "NMI")) {
    ok = RESET();
  } else if (!strncmp((char*)cmd_buffer, "GAME=", 5)) {
    ok = GAME((cmd_buffer[5] & 1) ? HIGH : LOW);
  } else if (!strncmp((char*)cmd_buffer, "EXROM=", 6)) {
    ok = EXROM((cmd_buffer[6] & 1) ? HIGH : LOW);
  } else if (!strncmp((char*)cmd_buffer, "loadx ", 6)) { // Load from serial to C64
    ok = loadFileX((char*)cmd_buffer + 6);
  } else {
    switch (cmd_buffer[0])
    {
      case 'V': Serial.println(version_string); break;
      case 'R': ok = ReadDPRAM(); break;
      case 'W': ok = WriteDPRAM(); break;
      case ':': ok = WriteDPRAMHex(); break;
      case 'T': ok = TransferBytesToC64(); break;
      case 'F': ok = TransferBytesFromC64(); break;
      case '/':
        {
          int len = strlen((char*)cmd_buffer);
          if (len > 3 && !strcmp((char*)cmd_buffer + len - 3, ".ic")) {
            startPlaylist((char*)cmd_buffer );
          } else {
            fastReset();
            ok = (loadFileSD((char*)cmd_buffer) && runBasic()); break;
          }
        }

      case 0: break; // empty string. Don't mind ignoring this.
      default:
        Serial.println("ERR Unrecognised command"); break;
    }
  }
  if (ok) {
    Serial.println("OK");
  }
}

void enableDetectNMI() {
  PCMSK2 |= (1 << PCINT16);
  PCICR |= (1 << PCIE2);
  NMICount = 0;
}

void disableDetectNMI() {
  PCMSK2 &= ~(1 << PCINT16);
  PCICR &= ~(1 << PCIE2);
  NMICount = 0;
}

ISR(PCINT2_vect) {
  if (detectNMI) {
    checkNMI();
  }
}

// ----------------------------------------------------------------------------------------

// Serial commands

void ReadString()
{
  int i = 0;
  uint8_t c;

  cmd_buffer[0] = 0;
  do
  {
    if (Serial.available())
    {
      c = Serial.read();
      if (c > 31)
      {
        cmd_buffer[i++] = c;
        cmd_buffer[i] = 0;
      }
    }

    if (i == 0) {
      Serial.setTimeout(1);
      check_c64_input();
      if (cmd_buffer[0]) {
        break; // Command from playlist
      }

    } else {
      Serial.setTimeout(1000);
    }
  }
  while (c != 10 && c != 13);
}

bool TransferBytesToC64() // T<four byte hex address><size in hex>
{
  if (strlen(cmd_buffer) < 7) {
    Serial.println("ERR");
    return false;
  }
  char *p = cmd_buffer + 1;
  uint16_t addr = HexToVal16(p); p += 4;
  uint8_t size = HexToVal(p); p += 2;
  return TransferBytesInC64(TO_C64_BUFFER1_ADDR, addr, size, TRANSFER_CMD_TRANSFER);
}

bool TransferBytesFromC64() // F<four byte hex address><size in hex>
{
  if (strlen(cmd_buffer) < 7) {
    Serial.println("ERR");
    return false;
  }
  char *p = cmd_buffer + 1;
  uint16_t addr = HexToVal16(p); p += 4;
  uint8_t size = HexToVal(p); p += 2;
  return TransferBytesInC64(addr, 0xdf00, size, TRANSFER_CMD_TRANSFER_2);
}

bool ReadDPRAM() // R<address><size in hex>  - read <size> bytes from DPRAM, beginning at <address> (in hex)
{
  if (strlen(cmd_buffer) < 7) {
    Serial.println("ERR");
    return false;
  }
  char *p = cmd_buffer + 1;
  uint16_t addr = HexToVal16(p); p += 4;
  uint16_t size = HexToVal16(p); p += 4;
  while (size > 0) {
    uint8_t l = size > kMaxBufferSize ? kMaxBufferSize : size;
    ReadDPRAMToBuffer(addr, l);
    PrintBufferHex(addr, l);
    size -= l;
    addr += l;
  }
  Serial.println(":00000001FF");

  return true;
}

bool WriteDPRAMHex() {
  /*
    1.  Start code, one character, an ASCII colon ':'.
    2.  uint8_t count, two hex digits (one hex digit pair), indicating the number of bytes (hex digit pairs) in the data field. The maximum uint8_t count is 255 (0xFF). 16 (0x10) and 32 (0x20) are commonly used uint8_t counts.
    3.  Address, four hex digits, representing the 16-bit beginning memory address offset of the data. The physical address of the data is computed by adding this offset to a previously established base address, thus allowing memory addressing beyond the 64 kilouint8_t limit of 16-bit addresses. The base address, which defaults to zero, can be changed by various types of records. Base addresses and address offsets are always expressed as big endian values.
    4.  Record type (see record types below), two hex digits, 00 to 05, defining the meaning of the data field.
    5.  Data, a sequence of n bytes of data, represented by 2n hex digits. Some records omit this field (n equals zero). The meaning and interpretation of data bytes depends on the application.
    6.  Checksum, two hex digits, a computed value that can be used to verify the record has no errors.
  */

  uint8_t chksum = 0, b;
  char *p = cmd_buffer + 1;

  int slen = strlen((const char *)cmd_buffer);
  if (slen < 10) {
    Serial.println("ERR way too short line");
    return false;
  }

  uint8_t size = HexToVal(p); p += 2;
  if (slen < 1 + 10 + 2 * size) {
    Serial.println("ERR too short line");
    return false;
  }
  chksum += size;
  b = HexToVal(p); p += 2;
  chksum += b;
  uint16_t addr = b << 8;
  b = HexToVal(p); p += 2;
  chksum += b;
  addr |= b;
  uint8_t record_type = HexToVal(p); p += 2;
  chksum += record_type;
  for (int i = 0; i < size; i++) {
    b = HexToVal(p); p += 2;
    buffer[i] = b;
    chksum += b;
  }
  b = HexToVal(p); p += 2;
  chksum += b;
  if (chksum) {
    Serial.print("ERR ");
    Serial.print(b, HEX);
    Serial.print(" ");
    Serial.print(chksum, HEX);
    Serial.println("");
    return false;
  }
  // TODO handle other record types
  if (record_type == 0 && size) {
    WriteBufferToDPRAM(addr, size);
  }
  return record_type == 1; // Return true for END
}

bool WriteDPRAM() // W<four byte address>:<data in hex, two characters per byte, max of 16 bytes per line>
{
  if (strlen(cmd_buffer) < 8) {
    Serial.println("ERR");
    return false;
  }
  char *p = cmd_buffer + 1;
  uint16_t addr = HexToVal16(p); p += 4;
  // cmd_buffer[x] should now be a :
  if (p[0] != ':') {
    Serial.println("ERR");
    return false;
  }
  p++;

  int iBufferUsed = 0;
  while (p[0] && p[1] && iBufferUsed < kMaxBufferSize && p[0] != ',')
  {
    uint8_t c = HexToVal(p); p += 2;
    buffer[iBufferUsed++] = c;
  }

  // if we're pointing to a comma, then the optional checksum has been provided!
  if (p[0] == ',' && p[1] && p[2]) {
    p++;
    uint8_t checksum = HexToVal(p); p += 2;
    uint8_t our_checksum = CalcBufferChecksum(iBufferUsed);
    if (our_checksum != checksum) {
      // checksum fail!
      iBufferUsed = -1;
      Serial.print("ERR ");
      Serial.print(checksum, HEX);
      Serial.print(" ");
      Serial.print(our_checksum, HEX);
      Serial.println("");
      return false;
    }
  }

  // buffer should now contains some data
  if (iBufferUsed > 0) {
    WriteBufferToDPRAM(addr, iBufferUsed);
  }

  return (iBufferUsed > -1);
}

// ----------------------------------------------------------------------------------------

static inline void SetDataLinesAsInputs() {
  DDRA = 0x00;
}

static inline void SetDataLinesAsOutputs() {
  DDRA = 0xff;
}

void SetAddress(uint16_t a)
{
#if 1
  PORTH = (PORTH & ~B01100011) | ((a & 0B00000011) << 5) | ((a & 0B11000000) >> 6);
  PORTB = (PORTB & ~B00110000) | ((a & 0B00001100) << 2);
  PORTJ = (PORTJ & ~B00000011) | ((a & 0B00110000) >> 4);
  PORTD = (PORTD & ~B00001100) | ((a & 0x300) >> 6);
  // TODO kPin_Addr10 and kPin_Addr11
#else
  // This is surprisingly slow
  digitalWrite(kPin_Addr0,  (a & 1) ? HIGH : LOW    );
  digitalWrite(kPin_Addr1,  (a & 2) ? HIGH : LOW    );
  digitalWrite(kPin_Addr2,  (a & 4) ? HIGH : LOW    );
  digitalWrite(kPin_Addr3,  (a & 8) ? HIGH : LOW    );
  digitalWrite(kPin_Addr4,  (a & 16) ? HIGH : LOW   );
  digitalWrite(kPin_Addr5,  (a & 32) ? HIGH : LOW   );
  digitalWrite(kPin_Addr6,  (a & 64) ? HIGH : LOW   );
  digitalWrite(kPin_Addr7,  (a & 128) ? HIGH : LOW  );
  digitalWrite(kPin_Addr8,  (a & 256) ? HIGH : LOW  );
  digitalWrite(kPin_Addr9,  (a & 512) ? HIGH : LOW  );
  digitalWrite(kPin_Addr10, (a & 1024) ? HIGH : LOW );
  digitalWrite(kPin_Addr11, (a & 2048) ? HIGH : LOW );
#endif
  ////digitalWrite(kPin_Addr12, (a&4096)?HIGH:LOW );
  ////digitalWrite(kPin_Addr13, (a&8192)?HIGH:LOW );
  ////digitalWrite(kPin_Addr14, (a&16384)?HIGH:LOW);
}

// this function assumes that data lines have already been set as OUTPUTS.
static inline void SetData(uint8_t b) {
  PORTA = b;
}

// this function assumes that data lines have already been set as INPUTS.
static inline uint8_t ReadData() {
  return PINA;
}

static void beginDPRAMWrite() {
  digitalWrite(kPin_nOE, HIGH); // stop DPRAM from outputting byte
  digitalWrite(kPin_nWE, HIGH); // disables write
  SetDataLinesAsOutputs();
}

static void endDPRAMWrite() {
  SetDataLinesAsInputs();
}

static void beginDPRAMRead() {
  digitalWrite(kPin_nWE, HIGH); // disables write
  SetDataLinesAsInputs();
  digitalWrite(kPin_nOE, LOW); // makes the DPRAM output the byte
  delayMicroseconds(1);
}

static void endDPRAMRead() {
  digitalWrite(kPin_nOE, HIGH); // stops the DPRAM outputting the byte
}

// ----------------------------------------------------------------------------------------
void ReadDPRAMToBuffer(uint16_t addr, int size)
{
  digitalWrite(kPin_LED_Grn, HIGH);
  beginDPRAMRead();
  for (int x = 0; x < size; ++x)
  {
    buffer[x] = ReadByteFrom(addr + x);
  }
  endDPRAMRead();
  digitalWrite(kPin_LED_Grn, LOW);
}

void WriteBufferToDPRAM(uint16_t addr, int size)
{
  beginDPRAMWrite();
  for (int x = 0; x < size; ++x)
  {
    WriteByteTo(addr + x, buffer[x]);
    //delayMicroseconds(k_uTime_WriteDelay_uS);
  }
  endDPRAMWrite();
}


// this function assumes that data lines have already been set as INPUTS, and that
// nOE is set LOW.
uint8_t ReadByteFrom(uint16_t addr)
{
  SetAddress(addr);
  //digitalWrite(kPin_nCE, LOW);
  //delayMicroseconds(k_uTime_ReadPulse_uS);
  NOP;
  uint8_t b = ReadData();
  //digitalWrite(kPin_nCE, HIGH);

  return b;
}

// this function assumes that data lines have already been set as OUTPUTS, and that
// nOE is set HIGH.
void WriteByteTo(uint16_t addr, uint8_t b)
{
  SetAddress(addr);
  SetData(b);

  //digitalWrite(kPin_nCE, LOW);
  digitalWrite(kPin_nWE, LOW); // enable write
  NOP;
  //delayMicroseconds(k_uTime_WritePulse_uS);

  digitalWrite(kPin_nWE, HIGH); // disable write
  //digitalWrite(kPin_nCE, HIGH);
}

// ----------------------------------------------------------------------------------------

void PrintBufferHex(int addr, int size)
{
  uint8_t chk = size + addr + (addr >> 8);
  Serial.print(':');

  Serial.print(hexa[ (size & 0xF0) >> 4 ]);
  Serial.print(hexa[ (size & 0x0F)      ]);
  Serial.print(hexa[ (addr & 0xF000) >> 12 ]);
  Serial.print(hexa[ (addr & 0x0F00) >> 8 ]);
  Serial.print(hexa[ (addr & 0x00F0) >> 4 ]);
  Serial.print(hexa[ (addr & 0x000F)      ]);
  Serial.print("00"); // Record type DATA

  for (int x = 0; x < size; ++x) {
    Serial.print(hexa[ (buffer[x] & 0xF0) >> 4 ]);
    Serial.print(hexa[ (buffer[x] & 0x0F)      ]);
    chk = chk + buffer[x];
  }

  chk = -chk;
  Serial.print(hexa[ (chk & 0xF0) >> 4 ]);
  Serial.print(hexa[ (chk & 0x0F)      ]);
  Serial.println("");
}

void PrintBuffer(int size)
{
  uint8_t chk = 0;

  for (int x = 0; x < size; ++x)
  {
    Serial.print(hexa[ (buffer[x] & 0xF0) >> 4 ]);
    Serial.print(hexa[ (buffer[x] & 0x0F)      ]);
    chk = chk ^ buffer[x];
  }

  Serial.print(",");
  Serial.print(hexa[ (chk & 0xF0) >> 4 ]);
  Serial.print(hexa[ (chk & 0x0F)      ]);
  Serial.println("");
}

uint8_t CalcBufferChecksum(int size)
{
  uint8_t chk = 0;
  for (int x = 0; x < size; ++x) {
    chk = chk ^  buffer[x];
  }
  return (chk);
}

// converts one character of a HEX value into its absolute value (nibble)
static inline uint8_t nibble(char b)
{
  if (b >= '0' && b <= '9') return (b - '0');
  if (b >= 'A' && b <= 'F') return ((b - 'A') + 10);
  if (b >= 'a' && b <= 'f') return ((b - 'a') + 10);
  return (0);
}

uint8_t HexToVal(const char *p)
{
  return (nibble(p[0]) << 4) | nibble(p[1]);
}

uint16_t HexToVal16(const char *p)
{
  return (nibble(p[0]) << 12) | (nibble(p[1]) << 8) | (nibble(p[2]) << 4) | nibble(p[3]);
}


// ----------------------------------------------------------------------------------------

bool NMI() {
  pinMode(kPin_nNMI, OUTPUT);
  digitalWrite(kPin_nNMI, LOW);
  delayMicroseconds(100);
  digitalWrite(kPin_nNMI, HIGH);
  pinMode(kPin_nNMI, INPUT);
  return true;
}

bool RESET() {
  pinMode(kPin_nRESET, OUTPUT);
  digitalWrite(kPin_nRESET, LOW);
  NOP;
  digitalWrite(kPin_nRESET, HIGH);
  pinMode(kPin_nRESET, INPUT);
  return true;
}


bool EXROM(uint8_t value) {
  if (exrom != value) {
    exrom = value;
    digitalWrite(kPin_nEXROM, value);
    delayMicroseconds(1); // How much is needed for PLA ?
  }
  return true;
}

bool GAME(uint8_t value) {
  if (game != value) {
    game = value;
    digitalWrite(kPin_nGAME, value);
    delayMicroseconds(1); // How much is needed for PLA ?
  }
  return true;
}


bool TransferBytesInC64(uint16_t from_addr, uint16_t to_addr, uint8_t size, uint8_t cmd)  {

  beginDPRAMWrite();
  // addresses in transfer.asm
  WriteByteTo(TRANSFER_COUNT_LOCATION, size);
  WriteByteTo(TRANSFER_SOURCE_LOCATION, from_addr);
  WriteByteTo(TRANSFER_SOURCE_LOCATION + 1, from_addr >> 8);
  WriteByteTo(TRANSFER_TARGET_LOCATION, to_addr);
  WriteByteTo(TRANSFER_TARGET_LOCATION + 1, to_addr >> 8);
  WriteByteTo(TRANSFER_CMD_ADDR, cmd);
  endDPRAMWrite();

  // TODO wait response from C64

  bool ex = exrom;
  if (ex) {
    EXROM(LOW);
  }
  NMI();

  if (ex) { // restore previous state
    delay(5); // Wait for C64 loop, 5 for 256, 3 for 128 bytes
    EXROM(HIGH);
  }
  return true;
}


bool runBasic() {
  beginDPRAMWrite();
  WriteByteTo(TRANSFER_CMD_ADDR, TRANSFER_CMD_RUN);
  endDPRAMWrite();

  EXROM(LOW);
  NMI();
  delayMicroseconds(500);
  EXROM(HIGH);

  return true;
}

bool printText(const char *text) {
  // FIXME works for <= 40 chars
  beginDPRAMWrite();
  int i;
  for (i = 0; text[i] && i < 40; i++) {
    WriteByteTo(TO_C64_BUFFER1_ADDR + i, text[i]);
  }
  WriteByteTo(TO_C64_BUFFER1_ADDR + i, 0);
  WriteByteTo(TRANSFER_CMD_ADDR, TRANSFER_CMD_PRINT);
  endDPRAMWrite();

  EXROM(LOW);
  NMI();
  delay(1 + i / 3 );
  EXROM(HIGH);

  return true;
}


bool fastReset() {
  EXROM(LOW);
  RESET();
  delay(200);
  EXROM(HIGH);
  return true;
}

// ----------------------------------------------------------------------------------------

// SD commands

void initSD() {
  if (!SD.begin(kPin_nSD_SEL, SPI_FULL_SPEED)) {
    Serial.println("SD initialization failed!");
    // while (1); // FIXME should not be mandatory
  } else {
    Serial.println("SD initialization done.");
  }
}

bool loadControl() {

  uint32_t far_address = (uint32_t)control_bin; // CONTROL_PROGMEM_ADDR;

  int i = 0;
  int len = sizeof(control_bin);
  // pgm_read_byte_far( far_address + i++) + (pgm_read_byte_far( far_address + i++) << 8);
  uint16_t addr = 0x000;
  DEBUG("LOADING CONTROL ");
  DEBUG(len);
  long before = millis();

  beginDPRAMWrite();
  while (len > 0) {
    uint8_t data = pgm_read_byte_far( far_address + i++);
    WriteByteTo(addr, data);
    addr ++;
    len--;
  }
  endDPRAMWrite();

  DEBUG(" bytes in ");
  DEBUG(millis() - before);
  DEBUGLN("ms");
  return true;
}

bool loadControlSD() {
  File control = SD.open("control.bin", FILE_READ);
  if (!control) {
    return false;
  }
  uint16_t addr = 0x000;
  DEBUG("LOADING CONTROL ");
  long before = millis();

  while (control.available()) {
    int len = control.read(buffer, sizeof(buffer));
    if (len <= 0) {
      break;
    }
    WriteBufferToDPRAM(addr, len);
    addr += len;
  }
  control.close();
  DEBUG(addr);
  DEBUG(" bytes in ");
  DEBUG(millis() - before);
  DEBUGLN("ms");
  return true;
}

bool loadBrowser() {
  uint32_t far_address = (uint32_t)browser_prg; //BROWSER_PROGMEM_ADDR;

  DEBUG("LOADING BROWSER ");
  long before = millis();

  int i = 0;
  uint16_t len = sizeof(browser_prg); // pgm_read_byte_far( far_address + i++) + (pgm_read_byte_far( far_address + i++) << 8);
  uint16_t addr = pgm_read_byte_far( far_address + i++) + (pgm_read_byte_far( far_address + i++) << 8);
  //uint16_t addr = 0x400;
  DEBUG(len);
  DEBUG(" ADDRESS ");
  DEBUG(addr);

  EXROM(LOW);
  while (len > 0) {
    int size = len > TRANSFER_MAX ? TRANSFER_MAX : len;
    for (int j = 0; j < size; j++) {
      uint8_t data = pgm_read_byte_far( far_address + i++);
      buffer[j] = data;
      len--;
    }
    WriteBufferToDPRAM(TO_C64_BUFFER1_ADDR, size);
    TransferBytesInC64(TO_C64_BUFFER1_ADDR, addr, size, TRANSFER_CMD_TRANSFER);
    delay(6); // TODO use double buffering

    addr += size;
  }

  DEBUG(" - ");
  DEBUG(addr);
  setBasicEnd(addr);

  delay(2);
  EXROM(HIGH);

  DEBUG(" TIME ");
  DEBUG(millis() - before);
  DEBUGLN("ms");
  return true;
}

bool loadFileSD(const char *filename) {
  initSD();
  if (!filename || !*filename)
    filename = "browser.prg";

  File dataFile = SD.open(filename);
  if (!dataFile) {
    Serial.println("ERR File not found error");
    return false;
  }
  uint16_t addr = dataFile.read() + (dataFile.read() << 8);
  //uint16_t addr = 0x400;
  DEBUG("LOADING FILE ");
  DEBUG(filename);
  DEBUG(" ");
  DEBUG(addr);
  long before = millis();

  EXROM(LOW);
  while (dataFile.available()) {
    int len = dataFile.read(buffer, sizeof(buffer));
    if (len <= 0) {
      break;
    }

    WriteBufferToDPRAM(TO_C64_BUFFER1_ADDR, len);
    TransferBytesInC64(TO_C64_BUFFER1_ADDR, addr, len, TRANSFER_CMD_TRANSFER);
    // FIXME check status of C64 command
    delay(5); // TODO use double buffering

    /* FIXME optimize
        Leave C64 in waiting state instead of NMI on every command.
        Use double buffering
    */


    addr += len;
  }

  dataFile.close();
  DEBUG(" - ");
  DEBUG(addr);
  setBasicEnd(addr);

  delay(2);
  EXROM(HIGH);

  DEBUG(" TIME ");
  DEBUG(millis() - before);
  DEBUGLN("ms");

  /*
    //Verify
    dataFile = SD.open(filename);
    if (!dataFile) {
      Serial.println("ERR File not found error");
      return false;
    }
    addr = dataFile.read() + (dataFile.read() << 8);
    //uint16_t addr = 0x400;
    EXROM(LOW);
    while (dataFile.available()) {
      int len = dataFile.read(buffer, 64);
      if (len <= 0) {
        break;
      }

      TransferBytesInC64(addr, 0xdf00, len, TRANSFER_CMD_TRANSFER_2);
      delay(3);
      beginDPRAMRead();
      for (int x = 0; x < len; ++x)
      {
        uint8_t b = ReadByteFrom(0xdf00 + x);
        if (b != buffer[x]) {
        Serial.print("VERIFY ERROR ");
        Serial.print(addr);
        Serial.print(" : ");
        DEBUGLN(x);
        }
      }
      endDPRAMRead();

      addr += len;
    }
    dataFile.close();
    EXROM(HIGH);
  */


  return true;
}

void setBasicEnd(uint16_t addr) {
  buffer[0] = buffer[2] = buffer[4] = addr;
  buffer[1] = buffer[3] = buffer[5] = addr >> 8;
  WriteBufferToDPRAM(TO_C64_BUFFER1_ADDR, 6);
  TransferBytesInC64(TO_C64_BUFFER1_ADDR, 0x2d, 6, TRANSFER_CMD_TRANSFER);
}

void listFiles() {
  changeDir(".");
  while (true) {

    File entry =  currentDir.openNextFile();
    if (! entry) { // no more files
      break;
    }
    entry.getName((char*)buffer, 22);
    Serial.print((char*)buffer);
    if (entry.isDirectory()) {
      Serial.println("\t\tDIR");
    } else {
      // files have sizes, directories do not
      Serial.print("\t\t");
      //Serial.println(entry.getSize(), DEC);
      Serial.println(entry.size(), DEC);
    }
    entry.close();
  }
}

void tree(File dir, char *p, char *buffer) {
  *p = 0;
  while (true) {

    File entry =  dir.openNextFile();
    if (! entry) {
      // no more files
      break;
    }
    entry.getName(p, 22);
    Serial.print(buffer);
    if (entry.isDirectory()) {
      Serial.println("/");
      strcat (p, "/");
      tree(entry, p + strlen(p), buffer);
    } else {
      // files have sizes, directories do not
      //Serial.print(entry.size(), DEC);
      Serial.println();
    }
    entry.close();
  }
}

bool removeFileSD(const char *filename) {
  char newPath[PATH_MAX];
  makeFilePath(newPath, filename);
  if (!SD.remove(newPath)) {
    Serial.println("ERROR");
    return false;
  }
  return true;
}


// ----------------------------------------------------------------------------------------


void init_c64() {
  beginDPRAMWrite();
  WriteByteTo(CMD_IC_STATUS_ADDR, CMD_READY);
  WriteByteTo(CMD_TO_IC_ADDR, CMD_IDLE);

  WriteByteTo(TRANSFER_CMD_ADDR, CMD_IDLE);
  WriteByteTo(TRANSFER_STATUS_ADDR, CMD_IDLE);

  // SetAddress(0x0200);// Park address lines
  endDPRAMWrite();
  currentPath[0] = 0;

  initSD();
  changeDir(".");
  loadControlSD() || loadControl();
}

static bool sw1, sw2;
static bool lastSW1 = HIGH, lastSW2 = HIGH;
static unsigned long lastNMITime = -1;

void checkNMI() {
  static int last = -1;

  if (lastSW1 != digitalRead(kPin_nSW1)) {
    DEBUGLN("SW1");
    lastSW1 = digitalRead(kPin_nSW1);
    sw1 = !lastSW1;
  }
  if (lastSW2 != digitalRead(kPin_nSW2)) {
    DEBUGLN("SW2");
    lastSW2 = digitalRead(kPin_nSW2);
    sw2 = !lastSW2;
  }

  int val = digitalRead(kPin_nNMI2);
  if (val == last) {
    return;
  }
  last = val;
  if (val) { // TODO configure to detect only falling edge
    return;
  }
  unsigned long currentTime = millis();
  if (currentTime - lastNMITime > 20) { // filter noise
    if (currentTime - lastNMITime > 1000) {
      NMICount = 0;
    } else {
      NMICount++;
    }
  }
  lastNMITime = currentTime;

  /*
    DEBUG("NMI:");
    DEBUG(currentTime);
    DEBUG("NMI:");
    DEBUGLN(NMICount);
  */
}


void check_NMI() {
  if (NMICount < 2 && !sw2 && !sw1) {
    return;
  }
  DEBUG("RESTORE * 3 - LOADING BROWSER ");
  DEBUGLN(NMICount);
  sw1 = sw2 = false;
  NMICount = lastNMITime = 0;
  endPlaylist();
  loadControlSD() || loadControl();
  fastReset();
  loadFileSD(0) || loadBrowser();
  runBasic();
}

void check_c64_input() {
  static uint8_t last = -1;
  int i = 1;
  //disableDetectNMI();
  check_NMI();
  handlePlaylist();

  while (i > 0) {
    beginDPRAMRead();
    uint8_t cmd = ReadByteFrom(CMD_TO_IC_ADDR);
    SetAddress(0x0200);// Don't leave the address lines to point 0x1fff
    endDPRAMRead();

    if (cmd != last) {
      last = cmd;
      // FIXME allow commands only if CMD_READY is issued first to prevent random commands
      uint8_t status = handle_c64_input(cmd);
      beginDPRAMWrite();
      WriteByteTo(CMD_IC_STATUS_ADDR, status);
      SetAddress(0x0200);// Park address lines
      endDPRAMWrite();
      i = 100;
      if (CMD_READY == cmd) {
        i = 1000;
      }
      if (CMD_IDLE == cmd) {
        break;
      }
    }

    i--;
  }
  //enableDetectNMI();
}


// ----------------------------------------------------------------------------------------

// XModem

int recvChar(int) {
  for (int i = 0; i < 100000; i++) {
    if (Serial.available()) {
      int c =  Serial.read();
      return c;
    }
  }
  return -1;
}

void sendChar(char c) {
  Serial.write(c);
}

bool recvDataHandler(unsigned long offset, char* data, int size) {
  if (maxSize && currentSize + size > maxSize) {
    size = maxSize - currentSize;
  }
  int len = recvFile2.write(data, size);
  //  DEBUGLN(len);
  currentSize += size;
  return len > 0;
}

bool recvFileX(const char *filename) {
  char newPath[PATH_MAX];
  makeFilePath(newPath, filename);

  char *p = strrchr(newPath, ' ');
  if (p) {
    *p = 0;
    maxSize = atol(p + 1);
  } else {
    maxSize = 0;
  }
  currentSize = 0;

  if (SD.exists(newPath)) {
    SD.remove(newPath);
  }

  recvFile2 = SD.open(newPath, FILE_WRITE);
  if (!recvFile2) {
    Serial.println("ERR Directory or something not found error");
    return false;
  }

  Serial.setTimeout(1000);
  XModem xm(recvChar, sendChar, recvDataHandler);
  bool result = xm.receive();
  recvFile2.close();
  if (!result) {
    SD.remove(newPath);
    Serial.println("ERR in transfer");
  }
  return result;
}

bool sendDataHandler(unsigned long offset, char* data, int size) {
  int len = sendFile2.read(data, size);
  if (len > 0 && len < size) {
    memset(data + len, 0, size - len);
  }
  //  DEBUG("SEND DATA");
  //  DEBUGLN(len);
  return len > 0;
}

bool sendFileX(const char *filename) {
  char newPath[PATH_MAX];
  makeFilePath(newPath, filename);
  sendFile2 = SD.open(newPath, FILE_READ);
  if (!sendFile2) {
    Serial.println("ERR File not found error");
    return false;
  }
  Serial.print("rx ");
  Serial.print(newPath);
  Serial.print(" ");
  Serial.println(sendFile2.fileSize());

  Serial.setTimeout(1000);
  XModem xm(recvChar, sendChar, sendDataHandler);
  bool result = xm.transmit();
  sendFile2.close();
  if (!result) {
    Serial.println("ERR in transfer");
  }
  return result;
}

bool loadDataHandler(unsigned long offset, char* data, int size) {
  if (maxSize && currentSize + size > maxSize) {
    size = maxSize - currentSize;
  }

  if (size <= 0) {
    return false;
  }

  // FIXME size <= 128, not 1024
  memcpy (buffer, data, size);
  WriteBufferToDPRAM(TO_C64_BUFFER1_ADDR, size); // 15 % of the load time
  TransferBytesInC64(TO_C64_BUFFER1_ADDR, loadAddr, size, TRANSFER_CMD_TRANSFER);
  delay(3); // TODO use double buffering. However, this is only 10% of the load time

  /* FIXME optimize
      Leave C64 in waiting state instead of NMI on every command.
      Use double buffering
  */
  loadAddr += size;
  currentSize += size;
  return true;
}


bool loadFileX(const char *params) {
  char *p = strchr(params, ' ');
  if (p) {
    *p = 0;
    maxSize = atol(p + 1);
  } else {
    maxSize = 0;
  }
  currentSize = 0;

  loadAddr = atoi(params);

  DEBUG("LOADING SIZE ");
  DEBUG(maxSize);
  DEBUG(" ADDRESS ");
  DEBUGLN(loadAddr);
  long before = millis();

  EXROM(LOW);
  Serial.setTimeout(1000);
  XModem xm(recvChar, sendChar, loadDataHandler);
  bool result = xm.receive();
  if (!result) {
    Serial.println("ERR in transfer");
  } else {
    Serial.print("OK ");
    Serial.print(loadAddr);
    setBasicEnd(loadAddr);
  }
  delay(2);
  EXROM(HIGH);

  DEBUG(" TIME ");
  DEBUG(millis() - before);
  DEBUGLN("ms");

  return result;
}

// ----------------------------------------------------------------------------------------


void makeFilePath(char *newPath, const char *filename) {
  // FIXME check PATH_MAX
  strcpy(newPath, currentPath);
  if (newPath[strlen(newPath) - 1] != '/') {
    strcat(newPath, "/");
  }
  strcat(newPath, filename);
}

static bool compare_extension(const char *ext1, const char *ext2) {
  for (int i = 0; i < 3; i++) {
    if (ext1[i] >= 'a' && ext1[i] <= 'z') {
      if ((ext1[i] - 0x20) != ext2[i]) {
        return false;
      }
    } else if (ext1[i] != ext2[i]) {
      return false;
    }
  }

  return true;
}

uint8_t separate_file_type(char *filename) {
  char *p = strrchr(filename, '.');
  if (p) {
    if (compare_extension(p + 1, "PRG")) {
      *p = 0;
      return FILE_PRG;
    }
    if (compare_extension(p + 1, "TCR")) {
      *p = 0;
      return FILE_TCRT;
    }
    if (compare_extension(p + 1, "TAP")) {
      *p = 0;
      return FILE_TAP;
    }
    if (compare_extension(p + 1, "D64")) {
      *p = 0;
      return FILE_D64;
    }
  }
  return FILE_UNKNOWN;
}

void fix_case(char *path) {
  for (int i = 0; i <  FILENAME_LENGTH; i++) {
    if (path[i] >= 'A' && path[i] <= 'Z') {
      path[i] += 0x20;
    } else if (path[i] >= 'a' && path[i] <= 'z') {
      path[i] -= 0x20;
    }
  }
}


bool d64Open = false;
DiskImage di;
ImageFile dh;
static int doffset = 254;


void dirTitleD64(char *buffer) {
  char name[17];
  char id[6];

  di_name_from_rawname(name, di_title(&di));
  //ptoa(name);

  /* Convert ID to ascii */
  memcpy(id, di_title(&di) + 18, 5);
  id[5] = 0;
  //ptoa(id);

  /* Print title and disk ID */
  DEBUGLN(name);
  DEBUGLN(id);
  //printf("0 \"%-16s\" %s\n", name, id);
  strcpy(buffer, name);
  strcat(buffer, " ");
  strcat(buffer, id);
  return;
}

void dirTitle(char *buffer) {
  int len = strlen(currentPath);
  if (len > FILENAME_LENGTH) {
    strcpy(buffer + 2, currentPath + len - 14);
    buffer[0] = buffer[1] = '.';
  } else {
    strcpy(buffer, currentPath);
  }
}

File playlistFile;
unsigned long nextPlay = 0;
unsigned int autoSleep = 0; // secs

void endPlaylist() {
  if (playlistFile) {
    playlistFile.close();
    nextPlay = 0;
    DEBUGLN("End playlist");
  }
}

bool playlistAutoSleep(int s) {
  autoSleep = s;
  return true;
}

bool playlistSleep(int s) {
  if (!nextPlay) {
    return false;
  }
  nextPlay = millis() + s * 1000L;
  DEBUG("Sleeping for ");
  DEBUGLN(s);
  return true;
}

void startPlaylist(const char *filename) {
  endPlaylist();

  playlistFile = SD.open(filename);
  if (!playlistFile) {
    DEBUGLN("Cannot open playlist");
    return;
  }
  nextPlay = millis();
  DEBUG("Start playlist ");
  DEBUGLN(filename);
}

void handlePlaylist() {
  if (!nextPlay || nextPlay > millis()) {
    return;
  }
  nextPlay = millis() + autoSleep * 1000L;

  int n = playlistFile.fgets(cmd_buffer, sizeof(cmd_buffer));
  if (n <= 0) {
    endPlaylist();
    return;
  }

  char *p = strrchr(cmd_buffer, '\n');
  if (p) {
    *p = 0;
  }
  p = strrchr(cmd_buffer, '\r');
  if (p) {
    *p = 0;
  }

  DEBUG("PLAY ");
  DEBUGLN(cmd_buffer);
  // Command is handled in ReadString & loop()
}


bool changeDir(const char *path) {
  char newPath[PATH_MAX];
  strcpy(newPath, currentPath);
  if (!strcmp(path, "..")) {
    if (!d64Open) {
      char *p = strrchr(newPath, '/');
      if (p) {
        *p = 0;
      }
    }
  } else if (!strcmp(path, ".")) {
    //
  } else if (path[0] == '/') {
    strcpy(newPath, path);
  } else {
    if (newPath[strlen(newPath) - 1] != '/') {
      strcat(newPath, "/");
    }
    strcat(newPath, path);
  }
  if (*newPath == 0) {
    strcpy(newPath, "/");
  }

  File newDir = SD.open(newPath);
  if (!newDir) {
    // cd into D64 image
    strcat(newPath, ".d64");
    //newDir = SD.open(newPath);
    if (!SD.exists(newPath)) {
      Serial.println("ERR DIR NOT FOUND");
      return false;
    }
    // Target is a D64 image
    if (!di_load_image(&di, newPath)) {
      Serial.println("ERR di_load_image failed");
      return false;
    }

    /* Open directory for reading */
    if (!di_open(&di, &dh, (unsigned char *) "$", T_PRG, "rb")) {
      Serial.println("ERR Couldn't open directory");
      goto CloseImage;
    }

    /* Read first block into buffer */
    if (di_read(&dh, buffer, 254) != 254) {
      Serial.println("ERR BAM read failed");
      goto CloseDir;
    }

    /* Print number of blocks free */
    DEBUG(di.blocksfree);
    DEBUGLN("blocks free");

    doffset = 254;
    d64Open = true;
    return true;

CloseDir:
    di_close(&dh);

CloseImage:
    di_free_image(&di);
  }
  if (d64Open) {
    d64Open = false;
    di_close(&dh);
    di_free_image(&di);
  }

  if (currentDir) {
    currentDir.close();
  }
  currentDir = newDir;
  strcpy(currentPath, newPath);
  DEBUGLN(currentPath);
  return true;
}

// ----------------------------------------------------------------------------------------
// C64 commands

void ptoa(char *s) {
  unsigned char c;

  while ((c = (unsigned char) * s)) {
    c &= 0x7f;
    if (c >= 'A' && c <= 'Z') {
      c += 32;
    } else if (c >= 'a' && c <= 'z') {
      c -= 32;
    } else if (c == 0x7f) {
      c = 0x3f;
    }
    *s++ = c;
  }
}

void atop(char *s) {
  unsigned char c;

  while ((c = (unsigned char) * s)) {
    c &= 0x7f;
    if (c >= 'A' && c <= 'Z') {
      c += 32;
    } else if (c >= 'a' && c <= 'z') {
      c -= 32;
    }
    *s++ = c;
  }
}


static const char *ftype[] = {
  "del",
  "seq",
  "prg",
  "usr",
  "rel",
  "cbm",
  "dir",
  "???"
};

static bool listD64(const char *filename) {
  ImageFile dh;
  char name[17];
  char id[6];

  int offset;
  char quotename[19];
  int type;
  int closed;
  int locked;
  int size;
  int track, sector;

  if (!SD.exists(filename)) {
    Serial.print("ERR FILE NOT FOUND ERROR ");
    Serial.println((char*)filename);
    return false;
  }

  /* Load image into ram */
  if (!di_load_image(&di, filename)) {
    Serial.println("ERR di_load_image failed");
    return (1);
  }

  /* Open directory for reading */
  if (!di_open(&di, &dh, (unsigned char *) "$", T_PRG, "rb")) {
    Serial.println("ERR Couldn't open directory");
    goto CloseImage;
  }
  /* Convert title to ascii */
  di_name_from_rawname(name, di_title(&di));
  ptoa(name);

  /* Convert ID to ascii */
  memcpy(id, di_title(&di) + 18, 5);
  id[5] = 0;
  ptoa(id);

  /* Print title and disk ID */
  Serial.println(name);
  Serial.println(id);
  //printf("0 \"%-16s\" %s\n", name, id);

  /* Read first block into buffer */
  if (di_read(&dh, buffer, 254) != 254) {
    Serial.println("ERR BAM read failed");
    goto CloseDir;
  }

  /* Read directory blocks */
  while (di_read(&dh, buffer, 254) == 254) {
    for (offset = -2; offset < 254; offset += 32) {

      /* If file type != 0 */
      if (buffer[offset + 2]) {

        di_name_from_rawname(name, buffer + offset + 5);
        type = buffer[offset + 2] & 7;
        closed = buffer[offset + 2] & 0x80;
        locked = buffer[offset + 2] & 0x40;
        track = buffer[offset + 3];
        sector = buffer[offset + 4];

        size = buffer[offset + 31] << 8 | buffer[offset + 30];

        /* Convert to ascii and add quotes */
        ptoa(name);
        sprintf(quotename, "\"%s\"", name);

        /* Print directory entry */
        Serial.print(size);
        Serial.print(",");
        Serial.print(quotename);
        Serial.print(",");
        Serial.print(closed);
        Serial.print(",");
        Serial.print(ftype[type]);
        Serial.print(",");
        Serial.print(locked);
        Serial.print(",");
        Serial.print(track);
        Serial.print(",");
        Serial.println(sector);


        //printf("%-4d  %-18s%c%s%c  <%2d,%2d>\n", size, quotename, closed ? ' ' : '*', ftype[type], locked ? '<' : ' ', track, sector);
      }
    }
  }
  /* Print number of blocks free */
  Serial.print(di.blocksfree);
  Serial.println("blocks free");


CloseDir:
  /* Close file */
  di_close(&dh);

CloseImage:
  /* Release image */
  di_free_image(&di);


  return true;

}



bool loadD64File(const char *filename) {
  char name[17];
  unsigned char rawname[16];
  int len;

  long before = millis();

  /* Convert filename */
  strncpy(name, filename, 16);
  name[16] = 0;
  //atop(name);
  di_rawname_from_name(rawname, name);


  /* Open file for reading */
  if (!di_open(&di, &dh, rawname, T_PRG, "rb")) {
    di_status(&di, (char*)buffer);
    Serial.print("ERR ");
    Serial.println((const char*)buffer);
    return false;
  }


  len = di_read(&dh, buffer, 2);
  uint16_t addr = buffer[0] + (buffer[1] << 8);
  //uint16_t addr = 0x400;
  DEBUG("ADDRESS ");
  DEBUG(addr);

  EXROM(LOW);

  while ((len = di_read(&dh, buffer, sizeof(buffer))) > 0) {
    WriteBufferToDPRAM(TO_C64_BUFFER1_ADDR, len);
    TransferBytesInC64(TO_C64_BUFFER1_ADDR, addr, len, TRANSFER_CMD_TRANSFER);
    // FIXME check status of C64 command
    delay(5); // TODO use double buffering

    /* FIXME optimize
        Leave C64 in waiting state instead of NMI on every command.
        Use double buffering
    */
    addr += len;
  }

  DEBUG(" - ");
  DEBUG(addr);
  setBasicEnd(addr);

  delay(2);
  EXROM(HIGH);

  di_close(&dh);

  DEBUG(" TIME ");
  DEBUG(millis() - before);
  DEBUGLN("ms");

  return true;
}

//

static void cmdSelectFileD64(DirElement *element) {
  if (element->type == FILE_PRG) {
    // FIXME check if file is present
    fastReset();
    printText("LOADING ");
    printText(element->name);
    if (!loadD64File(element->name)) {
      return;
    }
    runBasic();
  }
}

static void cmdSelectFile(DirElement *element) {
  char newPath[PATH_MAX];
  int len;
  makeFilePath(newPath, element->name);
  switch (element->type) {
    case FILE_PRG:
      // FIXME check PATH_MAX
      strcat (newPath, ".PRG");
      break;
    case FILE_D64:
      strcat (newPath, ".D64");
      listD64(newPath);
      return;
      break;
    case FILE_UNKNOWN:
      len = strlen(newPath);
      if (len > 3 && (!strcmp(newPath + len - 3, ".ic") || !strcmp(newPath + len - 3, ".IC"))) {
        startPlaylist(newPath);
      }
      return;

    case FILE_TCRT:
    case FILE_TAP:
    default:
      DEBUG("Not supported");
      return;
  }
  if (SD.exists(newPath)) {
    fastReset();
    printText("LOADING ");
    printText(element->name);
    loadFileSD(newPath);
    runBasic();
  } else {
    Serial.print("FILE NOT FOUND ERROR ");
    Serial.println((char*)newPath);
  }
}


bool cmdReadDirD64(DirElement *current) { // Current points to buffer
  static uint8_t dbuffer[256];
  char name[17];
  int type;
  int closed;
  int locked;
  int size;
  int track, sector;

  memset(current, 0, sizeof(DirElement));
  if (sendDotDot) {
    sendDotDot = false;
    current->type = FILE_DIR;
    strcpy(current->name, "..");
    return true;
  }

  if (doffset >= 254) {
    /* Read directory blocks */
    if (di_read(&dh, dbuffer, 254) != 254) {
      DEBUGLN("END OF DIR");
      current->type = FILE_NONE;
      return false;
    }
    doffset = -2;
  }

  /* If file type != 0 */
  if (dbuffer[doffset + 2]) {

    di_name_from_rawname(name, dbuffer + doffset + 5);
    type = dbuffer[doffset + 2] & 7;
    closed = dbuffer[doffset + 2] & 0x80;
    locked = dbuffer[doffset + 2] & 0x40;
    track = dbuffer[doffset + 3];
    sector = dbuffer[doffset + 4];

    size = dbuffer[doffset + 31] << 8 | dbuffer[doffset + 30];
    /* Convert to ascii and add quotes */
    //ptoa(name);

    char newName[FILENAME_LENGTH + 5];
    strcpy(newName, name);
    current->type = (type == 2 ? FILE_PRG : FILE_UNKNOWN);
    newName[FILENAME_LENGTH] = 0;
    strcpy(current->name, newName);
    current->size = size * 256;

    /* Print directory entry */
    DEBUG(size);
    DEBUG(",");
    DEBUG(name);
    DEBUG(",");
    DEBUG(closed);
    DEBUG(",");
    DEBUG(ftype[type]);
    DEBUG(",");
    DEBUG(locked);
    DEBUG(",");
    DEBUG(track);
    DEBUG(",");
    DEBUGLN(sector);


    //printf("%-4d  %-18s%c%s%c  <%2d,%2d>\n", size, quotename, closed ? ' ' : '*', ftype[type], locked ? '<' : ' ', track, sector);
  }

  doffset += 32;

  return true;
}


bool cmdReadDir(DirElement *current) { // Current points to buffer
  memset(current, 0, sizeof(DirElement));
  if (sendDotDot) {
    sendDotDot = false;
    current->type = FILE_DIR;
    strcpy(current->name, "..");
    return true;
  }

  File entry =  currentDir.openNextFile();
  if (! entry) {  // no more files
    current->type = FILE_NONE;
  } else {
    if (entry.isDirectory()) {
      current->type = FILE_DIR;
      entry.getName(current->name, FILENAME_LENGTH);
      DEBUG("DIR\t");
    } else {
      char newName[FILENAME_LENGTH + 5];
      entry.getName(newName, FILENAME_LENGTH + 4);
      current->type = separate_file_type(newName);
      newName[FILENAME_LENGTH] = 0;
      strcpy(current->name, newName);
      DEBUG(entry.size());
      DEBUG("\t");
      // files have sizes, directories do not
      current->size = entry.size(); // Both are little endian
    }
    entry.close();
    ptoa(current->name);
    DEBUGLN(current->name);
  }
  return current->type != FILE_NONE;
}

uint8_t handle_c64_input(uint8_t cmd) {
  uint8_t status = CMD_READY;
  DEBUG(millis());
  DEBUG(':');

  switch (cmd) {
    case CMD_READY:
      DEBUGLN("CMD_READY");
      status = CMD_IDLE;
      break;

    case CMD_IDLE: // NOT SENT
      DEBUGLN("CMD_IDLE");
      status = CMD_IDLE;
      break;

    case CMD_LED_ON:
      DEBUGLN("CMD_LED_ON");
      digitalWrite(kPin_LED_Grn, HIGH);
      break;

    case CMD_LED_OFF:
      DEBUGLN("CMD_LED_OFF");
      digitalWrite(kPin_LED_Grn, LOW);
      break;

    case CMD_SD_SELECT_FILE:
      {
        DEBUGLN("CMD_SD_SELECT_FILE");
        ReadDPRAMToBuffer(IO_BUFFER_ADDR, sizeof(DirElement));
        //PrintBuffer(FILENAME_LENGTH);
        DirElement *element = (DirElement *)buffer;
        element->name[FILENAME_LENGTH] = 0;
        if (d64Open) {
          cmdSelectFileD64(element);
        } else {
          cmdSelectFile(element);
        }
      }
      break;

    case CMD_SD_OPEN_DIR:
      DEBUG("CMD_SD_OPEN_DIR: ");
      ReadDPRAMToBuffer(IO_BUFFER_ADDR, FILENAME_LENGTH);
      buffer[FILENAME_LENGTH] = 0;
      //PrintBuffer(FILENAME_LENGTH);
      DEBUGLN((char*)buffer);
      changeDir((char*)buffer);

      if (d64Open || strcmp(currentPath, "/")) {
        sendDotDot = true;
      }
      if (d64Open) {
        dirTitleD64((char*)buffer);
      } else {
        dirTitle((char*)buffer);
      }
      WriteBufferToDPRAM(IO_BUFFER_ADDR, FILENAME_LENGTH);
      break;

    case CMD_SD_READ_DIR_FAST:
      {
        DEBUG("CMD_SD_READ_DIR_FAST: ");
        DirElement *element = (DirElement *)buffer;
        int i = 0;
        while (i < ELEMS_PER_CMD) {
          i++;
          if (d64Open) {
            if (!cmdReadDirD64(element++)) {
              break;
            }
          } else {
            if (!cmdReadDir(element++)) {
              break;
            }
          }
        }
        WriteBufferToDPRAM(IO_BUFFER_ADDR, sizeof(DirElement) * i);
      }
      break;

    default:
      DEBUG("UNKNOWN CMD:");
      DEBUGLN(cmd);
      break;
  }
  return status;
}
