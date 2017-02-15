#ifndef BV
#  define BV(n) (1 << (n))
#endif

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