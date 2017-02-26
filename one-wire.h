#ifndef BV
#  define BV(n) (1 << (n))
#endif

/** The current state of the wire */
enum wire1state_t {
  // Must issue the reset command to continue from here
  IDLE,
  // Can issue any of the ROM commands from here:
  // search [F0h], read [33h], match [55h], skip [CCh], alarm search [ECh]
  // (both search variants will return the state to idle when finished)
  ROM_COMMAND,
  // Can issue the function commands from here:
  // convert T [44h], write scratchpad [4Eh], read scratchpad [BEh],
  // copy scratchpad [48h], recall EEPROM [B8h], read power supply [B4h]
  FUNCTION_COMMAND,
  // Must poll the line until it is free to get out of here
  WAIT_POLL
};

/** The device type */
enum wire1device_t {
  DS18B20 = 0x28
};

typedef struct {
  /** Address of the device */
  uint8_t address[8];
  /**
   * A bit field specifying the cached status of the device
   * Bits represent boolean values:
   *   [7:1] device specific (reserved)
   *   [1] Parasite power
   *   [0] reserved
   */
  uint8_t status, status_init;
  uint8_t scratchPad[8];
} wire1_t;

// Bit positions in the status byte for each device
#define W1_STATUS_PARASITE_POWER_BIT 1
#define W1_STATUS_ADDRESS_BIT        0

// The polynomial used for the one wire CRC
#define W1_CRC_POLYNOMIAL            0x8C

// Significant byte positions in one wire address
#define W1_ADDR_BYTE_CRC        7
#define W1_ADDR_BYTE_DEV_TYPE   0

// ROM commands
#define W1_ROMCMD_READ             0x33
#define W1_ROMCMD_MATCH            0x55
#define W1_ROMCMD_SEARCH           0xF0
#define W1_ROMCMD_ALARM            0xEC
#define W1_ROMCMD_SKIP             0xCC

// Function commands
#define W1_FUNC_PARASITE_POWER       0xB4

// "Macro" functions
void    wire1Hold(void);
void    wire1Release(void);
uint8_t wire1Poll4Hold(uint8_t us);
uint8_t wire1Poll4Release(uint8_t us);

// Initialization of devices
int8_t  wire1Reset(void);
void    wire1SetupPoll4Idle(uint16_t nloops);

// Reading/writing bits/bytes
uint8_t wire1ReadBit(void);
void    wire1WriteBit(uint8_t bit);
uint8_t wire1ReadByte(void);
void    wire1WriteByte(uint8_t writeByte);

// Searching devices
int8_t  wire1SearchLargerROM(
  uint8_t *const addrOut,
  uint8_t *const addrStart,
  const uint8_t lastConfPos
);
int8_t  wire1AlarmSearchLargerROM(
  uint8_t *const addrOut,
  uint8_t *const addrStart,
  const uint8_t lastConfPos
);

// Addressing devices
int8_t wire1ReadSingleROM(uint8_t *const addr);
int8_t wire1MatchROM(uint8_t *const addr);
int8_t wire1SkipROM();

int8_t wire1ReadPowerSupply(void);

// General functions
uint8_t crc8(
  uint8_t crcIn,
  uint8_t polynomial,
  uint8_t *const array,
  uint8_t const size
);
enum wire1state_t wire1GetState(void);
