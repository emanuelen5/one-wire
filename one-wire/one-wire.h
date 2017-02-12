#ifndef uint8_t
typedef unsigned char uint8_t;
#endif
#ifndef int8_t
typedef signed char int8_t;
#endif
#ifndef BV
#  define BV(n) (1 << (n))
#endif

void wire1Hold(void);
void wire1Release(void);
uint8_t wire1Poll4Hold(uint8_t us);
uint8_t wire1Poll4Release(uint8_t us);
int8_t  wire1Reset(void);
int8_t  wire1Read(void);
uint8_t wire1Write(uint8_t bit);