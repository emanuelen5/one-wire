#ifndef BV
#  define BV(n) (1 << (n))
#endif

/** The current state of the wire */
enum wire1state_t {
  // Must issue the reset command to continue from here
  idle,
  // Can issue any of the ROM commands from here: 
  // search [F0h], read [33h], match [55h], skip [CCh], alarm search [ECh]
  // (both search variants will return the state to idle when finished)
  rom_command, 
  // Can issue the function commands from here:
  // convert T [44h], write scratchpad [4Eh], read scratchpad [BEh],
  // copy scratchpad [48h], recall EEPROM [B8h], read power supply [B4h]
  function_command
};

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
  const int8_t lastConfPos
);
void wire1ReadSingleROM(uint8_t *const addr);
void wire1MatchROM(uint8_t *const addr);