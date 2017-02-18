#include <avr/io.h>
#include "one-wire.h"

// MACROS for being able to use convenient W1_-"variables
#ifndef W1_PORT_LETTER
  #warning W1_PORT_LETTER needs to be defined to be able to resolve \
          the 1-wire interface position. It was set B as default.
  #define W1_PORT_LETTER  B
#endif
#ifndef W1_PIN_POS
  #warning W1_PIN_POS needs to be defined to be able to resolve \
           the 1-wire interface position. It was set 0 as default.
  #define W1_PIN_POS      0
#endif

#define CONCAT(a, b)         a ## b // Concatenates a and b
#define CONCAT_EXPAND(a, b)  CONCAT(a, b) // Resolves a and b, then concatenates them

#define STRINGIFY(a)         #a // Puts quotes around a
#define STRINGIFY_EXPAND(a)  STRINGIFY(a) // Resolves a, then turns into string

static enum wire1state_t wire1state = idle; 

/**
 * Function for returning the state of the one-wire interface
 * @return  The state value
 */
enum wire1state_t wire1GetState() {
  return wire1state;
}

/**
 * Hold the wire down (drives it low).
 */
inline void wire1Hold(void) {
  CONCAT_EXPAND(PORT, W1_PORT_LETTER) &= ~BV(W1_PIN_POS); // Drive low/remove pullup
  CONCAT_EXPAND(DDR,  W1_PORT_LETTER) |=  BV(W1_PIN_POS); // Pin as output
}

/**
 * Releases the wire and lets it be pulled up.
 * Also adds the internal pullup to strengthen the pull up "force".
 */
inline void wire1Release(void) {
  CONCAT_EXPAND(DDR,  W1_PORT_LETTER) &= ~BV(W1_PIN_POS); // Pin as input
  CONCAT_EXPAND(PORT, W1_PORT_LETTER) |=  BV(W1_PIN_POS); // Add pullup
}

/**
 * Polls the wire a number of times, or until someone drives it low
 * Cycles taken for a complete function call:
 *   - If timeout:    14 + 4*nloops
 *   - If driven low: 14 + 4*(return value)
 * Time when line was driven:
 *   At most three cycles before 6 + 4*(return value) if
 *   return value is greater than 1, otherwise between cycle 1-10
 *
 * @param  nloops  Maximum number of times to poll
 * @return         0 if timeout, otherwise the number of samples taken before low
 */
uint8_t wire1Poll4Hold(uint8_t nloops) {
  uint8_t i = nloops;
  asm volatile(
      "tst  %[nloops] \n\t"
      "breq done%= \n\t"
    "loop%=:" 
      "subi %[count], 1  \n\t"
      "sbic %[port], " STRINGIFY_EXPAND(W1_PIN_POS) " \n\t" // Exit loop if wire is held low
      "brne loop%= \n\t" // Keep looping if larger than 0

      "brne done%= \n\t"
      "clr  %[nloops] \n\t" // Return 0 if timeout
    "done%=:"
      "sub  %[nloops], %[count] \n\t" // Return number of loops that were run
    : [count]  "+r" (i),       // Output operands
      [nloops] "+r" (nloops)
    : [port]   "I"  (_SFR_IO_ADDR(CONCAT_EXPAND(PIN, W1_PORT_LETTER)))
  );
  return nloops;
}

/**
 * Polls the wire a number of times, or until it is released
 * Cycles taken for a complete function call:
 *   - If timeout:    14 + 4*nloops
 *   - If driven low: 14 + 4*(return value)
 * Time when line was driven:
 *   At most three cycles before 6 + 4*(return value) if
 *   return value is greater than 1, otherwise between cycle 1-10
 *
 * @param  nloops  Maximum number of times to poll
 * @return         0 if timeout, otherwise the number of samples taken before high
 */
uint8_t wire1Poll4Release(uint8_t nloops) {
  uint8_t i = nloops;
  asm volatile(
      "tst  %[nloops] \n\t"
      "breq done%= \n\t"
    "loop%=:" 
      "subi %[count], 1  \n\t"        // Add 1 to counter (loop time)
      "sbis %[port], " STRINGIFY_EXPAND(W1_PIN_POS) " \n\t" // Exit loop if wire is held low
      "brne loop%= \n\t" // Keep looping if larger than 0

      "brne done%= \n\t"
      "clr  %[nloops] \n\t" // Return 0 if timeout
    "done%=:"
      "sub  %[nloops], %[count] \n\t" // Return number of loops that were run
    : [count]  "+r" (i),       // Output operands
      [nloops] "+r" (nloops)
    : [port]   "I"  (_SFR_IO_ADDR(CONCAT_EXPAND(PIN, W1_PORT_LETTER)))
  );
  return nloops;
}

/**
 * Resets all 1-wire devices and checks if there are any slaves that responds.
 * The time of the last sample is written within parentheses as comments after
 * the poll calls. The total time for the function call is written after that.
 * 
 * @return  negative if error, 0 if no slave responds, 1 if a slave responds
 */
int8_t wire1Reset(void) {
  // Hold for 450+ us to reset
  wire1Hold();
  // Use our own precision delay (will not exit early, since we hold the wire)
  wire1Poll4Release(122); // (494) 502 us = 4*122 + 14
  wire1Release();

  // Check if there is a response within 60 us
  if (!wire1Poll4Hold(15)) { // (66) 74 us = 4*15 + 14
    return 0;
  }

  wire1state = idle;
  // Wire shall be held by slave for 60-240 us
  if (!wire1Poll4Release(60)) { // (246) 254 us = 4*60 + 14
    return -1; // The wire was never released
  } else if (wire1Poll4Hold(58)) { // Wait out the rest of the slot
    return -2;
  } else {
    wire1state = rom_command;
    return 1; // Success!
  }
}

/**
 * Forces slaves into next state and then reads the returned value
 * @return  0 if wire was held low, otherwise nonzero
 */
int8_t wire1ReadBit(void) {
  uint8_t bittest = 0;
  uint8_t i = 5;

  // Hold for >1 us to update state of slaves
  wire1Hold();
  asm volatile("nop\n\t" : : ); // Wait one cycle before releasing
  wire1Release();

  // Supersample wire 24 times for 60 us to determine if it is driven low
  asm volatile(
      "nop \n\t"
    "loop:"
      "sbis %[port], " STRINGIFY_EXPAND(W1_PIN_POS) " \n\t" // 1
      "inc  %[bittest]                                \n\t" // 2
      "sbis %[port], " STRINGIFY_EXPAND(W1_PIN_POS) " \n\t" // 3
      "inc  %[bittest]                                \n\t" // 4
      "sbis %[port], " STRINGIFY_EXPAND(W1_PIN_POS) " \n\t" // 5
      "inc  %[bittest]                                \n\t" // 6
      "sbis %[port], " STRINGIFY_EXPAND(W1_PIN_POS) " \n\t" // 7
      "inc  %[bittest]                                \n\t" // 8
      "sbis %[port], " STRINGIFY_EXPAND(W1_PIN_POS) " \n\t" // 9
      "inc  %[bittest]                                \n\t" // 10
      "sbis %[port], " STRINGIFY_EXPAND(W1_PIN_POS) " \n\t" // 11
      "inc  %[bittest]                                \n\t" // 12
      // Go back
      "subi %[i], 1                                   \n\t" // 13
      "brne loop                                      \n\t" // 14 + 1
    : [bittest]  "+r" (bittest),  // Output operands
      [i]        "+r" (i)
    : [port]     "I"  (_SFR_IO_ADDR(CONCAT_EXPAND(PIN, W1_PORT_LETTER)))
  );

  // Total samples = 7 + 9 = 16
  return bittest>0?0:0xFF; // If sampled low at least one time, set to 0
}

/**
 * Forces slaves into next state and then send a 1 or 0
 * @param bit [boolean] Send a 0 if zero, otherwise send 1
 */
void wire1WriteBit(uint8_t bit) {
  // Hold for >1 us to update state of slaves
  wire1Hold();

  // Release before delaying if sending 1
  if (bit) {
    wire1Release();
    wire1Poll4Hold(15);
  } else {
    wire1Poll4Release(15); // (66) 74 us = 4*15 + 14
    wire1Release();
  }
  asm volatile("nop");
}

/**
 * Reads a byte over one-wire, LSB first
 * @return  The value that was read
 */
uint8_t wire1ReadByte(void) {
  uint8_t readByte = 0;
  for (int i = 0; i < 8; i++) {
    if (wire1ReadBit()) {
      readByte |= BV(i);
    }
  }
  return readByte;
}

/**
 * Writes a byte over one-wire, LSB first
 * @param  writeByte    The byte to write over the wire
 */
void wire1WriteByte(uint8_t writeByte) {
  for (int i = 0; i < 8; i++) {
    wire1WriteBit(writeByte & BV(i));
  }
}

/**
 * Mask out a specific bit in a bit array
 * @param  arr  Array of bytes (at most 32 byte)
 * @param  bit  Bit position in array
 * @return      The bit masked out in the byte that it corresponds to
 */
inline static uint8_t maskBitInArray(uint8_t *const arr, uint8_t bit) {
  return arr[bit/8] & BV(bit%8);
}

/**
 * Searches the address space for the next larger device address
 * compared to addrStart. Reads ACK and NACK of the ROM bit and decides
 * what branch to choose depending on the conflict position in the last 
 * search and the start address.
 * 
 * @param  addrOut      The output address for the returned ROM (8 byte)
 * @param  addrStart    Starting point for searching ROM address from 
 *                      (shall be a zero-vector if starting a new search,
 *                      or the last found ROM address if continuing search).
 *                      Can be a pointer to the same location as the output
 *                      address.
 * @param  lastConfPos  The bit position of the conflict bit in the last 
 *                      search (shall be above 63 - signed or unsigned if 
 *                      starting a new search, otherwise the returned value 
 *                      from the last search)
 * @return              A negative value if error, 0-63 if a device was found
 *                      and there exists a device with ROM address with a 
 *                      higher address which conflicts at that bit, 64 if a
 *                      device was found but no devices had a higher address.
 */
int8_t wire1SearchLargerROM(
  uint8_t * addrOut,
  uint8_t * addrStart,
  const int8_t lastConfPos
) {
  // Detect if there are any devices connected and init ROM command
  if (wire1Reset() != 1) {
    return -1; // Nothing connected!
  }

  PORTB ^= BV(1); // Trigger condition for oscilloscope
  // Issue the search ROM command to one-wire devices
  wire1WriteByte(0xF0);
  PORTB ^= BV(1); // Trigger condition for oscilloscope

  int8_t currConfPos = 64;
  uint8_t addrAck, addrNAck;
  uint8_t iByte = -1;
  uint8_t writeBit;
  for (int iBit = 0; iBit < 64; iBit++) {
    // Go to next byte when bit has "overflowed"
    if (iBit % 8 == 0) {
      iByte++;
      addrOut[iByte] = 0;
    }

    // Read the ACK and NACK bit (ROM bit)
    addrAck  = wire1ReadBit();
    addrNAck = wire1ReadBit();

    if (!addrAck && !addrNAck) { // Conflict - driven low both times

      // If at the conflict position, take the other branch, 
      // otherwise keep following the search start direction
      if (iBit == lastConfPos) {
        // Previously visited ROM's that were in conflict will have been zero, 
        // since we only search upward
        writeBit = 1;
      } else {
        writeBit = maskBitInArray(addrStart, iBit);
        // There is something to search that has not been searched before in this branch,
        // so store this location for next search
        if (!maskBitInArray(addrStart, iBit)) {
          currConfPos = iBit;
        }
      }
      // Update the output address
      if (writeBit) {
        addrOut[iByte] |= BV(iBit%8);
      }
      wire1WriteBit(writeBit); // Choose 0 if not supposed to branch yet
    } else if (addrAck && addrNAck) { // No device responds: strange error!
      return -128;
    } else { // ACK and NACK were different => no discrepancy, just follow along
      wire1WriteBit(addrAck);
      if (addrAck) {
        addrOut[iByte] |= BV(iBit%8);
      }
    }
  }

  wire1state = function_command;
  // Correctly found a device!
  return currConfPos;

}

/**
 * Read the ROM address of the one-wire device
 * (as long as there is only one slave connected)
 * @param addr  Pointer to an 8-byte array where the read ROM address shall be stored
 */
void wire1ReadSingleROM(uint8_t *const addr) {
  wire1Reset();
  wire1WriteByte(0x33);
  for (int i = 0; i < 8; i++) {
    addr[i] = wire1ReadByte();
  }
  wire1state = function_command;
}

/**
 * Sends the ROM address of a device that we want to access.
 * @param addr  Pointer to an 8-byte array where the ROM address is stored
 */
void wire1MatchROM(uint8_t *const addr) {
  wire1Reset();
  wire1WriteByte(0x55);
  for (int i = 0; i < 8; i++) {
    wire1WriteByte(addr[i]);
  }
  wire1state = function_command;
}

