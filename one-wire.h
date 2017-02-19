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
  FUNCTION_COMMAND
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
  uint8_t scratchPad[9];
} wire1_t;

#define W1_STATUS_PARASITE_POWER_BIT 1

#define W1_CRC_POLYNOMIAL            0x8C

void    wire1Hold(void);
void    wire1Release(void);
uint8_t wire1Poll4Hold(uint8_t us);
uint8_t wire1Poll4Release(uint8_t us);
int8_t  wire1Reset(void);
int8_t  wire1ReadBit(void);
void    wire1WriteBit(uint8_t bit);
uint8_t wire1ReadByte(void);
void    wire1WriteByte(uint8_t writeByte);
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
void wire1ReadSingleROM(uint8_t *const addr);
void wire1MatchROM(uint8_t *const addr);
void wire1SkipROM();
uint8_t wire1ReadPowerSuppy();
uint8_t wire1ReadScratchpad(uint8_t *const scratchpad);
uint8_t crc8(
  uint8_t crcIn,
  uint8_t polynomial,
  uint8_t *const array,
  uint8_t const size
);