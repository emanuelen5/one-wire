#ifndef BV
#  define BV(n) (1 << (n))
#endif

void wire1Hold(void);
void wire1Release(void);
uint8_t wire1Poll4Hold(uint8_t us);
uint8_t wire1Poll4Release(uint8_t us);
int8_t wire1Reset(void);
int8_t wire1Read(void);
void   wire1Write(uint8_t bit);
int8_t wire1SearchROM(uint8_t *const addrOut, uint8_t *const addrStart, const int8_t lastConfPos);