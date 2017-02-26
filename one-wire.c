// For pin definitions
#include <avr/io.h>
#include "one-wire.h"

// MACROS for being able to use convenient W1_-"variables
#ifndef W1_PORT_LETTER
  #warning W1_PORT_LETTER needs to be defined to be able to resolve \
           the 1-wire interface port. It was set to B as default.
  #define W1_PORT_LETTER  B
#endif
#ifndef W1_PIN_POS
  #warning W1_PIN_POS needs to be defined to be able to resolve \
           the 1-wire interface pin position. It was set to 0 as default.
  #define W1_PIN_POS      0
#endif

#define CONCAT(a, b)         a ## b // Concatenates a and b
#define CONCAT_EXPAND(a, b)  CONCAT(a, b) // Resolves a and b, then concatenates them

#define STRINGIFY(a)         #a // Puts quotes around a
#define STRINGIFY_EXPAND(a)  STRINGIFY(a) // Resolves a, then turns into string

// The factor needed to multiply each delay with, since the code is 
// optimized for an AVR with frequency of 1 MHz. 
// Adding 1/2 MHz to make rounding correct.
#define F_CPU_TIME_FACTOR ((F_CPU + 500000UL) / 1000000UL)

static enum wire1state_t wire1state = IDLE;
static uint16_t wire1_idleloops = 0;
uint16_t wire1Poll4Idle(void);
static int8_t wire1Search(
  uint8_t * addrOut,
  uint8_t * addrStart,
  const uint8_t lastConfPos,
  const uint8_t rom_command
);

/**
 * Function for returning the state of the one-wire interface
 * @return  The state value
 */
enum wire1state_t wire1GetState(void) {
  return wire1state;
}

/**
 * Hold the wire down (drives it low).
 */
inline void wire1Hold(void) {
  CONCAT_EXPAND(PORT, W1_PORT_LETTER) &= ~BV(W1_PIN_POS); // Remove pullup/drive low
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
       // Exit loop if wire is held low
      "sbic %[port], " STRINGIFY_EXPAND(W1_PIN_POS) " \n\t"
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
      "subi %A[count], 1  \n\t"
      // Exit loop if wire is held low
      "sbis %[port], " STRINGIFY_EXPAND(W1_PIN_POS) " \n\t"
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
 * Polls the wire slaves a number of times, or until no slaves respond with 0.
 * wire1SetupPoll4Idle must be run before this function with the time for the
 * polling to run. Takes about 95 cycles per loop.
 *
 * @return         0 if only '0' responses, otherwise the number of loops before
 *                 the '1' response
 */
uint16_t wire1Poll4Idle(void) {
  uint16_t i;
  for (i = 0; !wire1ReadBit() && i < wire1_idleloops; i++);
  if (i == wire1_idleloops) {
    // Timeout! The wire never went to IDLE state
    return 0;
  } else {
    // The wire went to IDLE state, and the slaves are ready to be
    // accessed again
    wire1state = IDLE;
    return i;
  }
}

/**
 * Set up for polling the wire at reset. If this function is run before a reset,
 * the wire will run wire1Poll4Idle and check up to nloop times for response on
 * the wire. The wait time increases with 95 cycles per nloops.
 *
 * @param  nloops  The number of loops to poll the wire for when running the
 *                 reset
 */
void wire1SetupPoll4Idle(uint16_t nloops) {
  wire1state = WAIT_POLL;
  wire1_idleloops = nloops;
}

/**
 * Resets all 1-wire devices and checks if there are any slaves that responds.
 * The time of the last sample is written within parentheses as comments after
 * the poll calls. The total time for the function call is written after that.
 *
 * @return  negative if error, 0 if no slave responds, 1 if a slave responds
 */
int8_t wire1Reset(void) {
  // Check first that we are not waiting for a slave to release the wire
  if (wire1state == WAIT_POLL && !wire1Poll4Idle()) {
    return -1;
  }
  // Hold for 450+ us to reset
  wire1Hold();
  // Use our own precision delay (will not exit early, since we hold the wire)
  wire1Poll4Release(122); // (494) 502 us = 4*122 + 14
  wire1Release();

  // Check if there is a response within 60 us
  if (!wire1Poll4Hold(15)) { // (66) 74 us = 4*15 + 14
    wire1state = IDLE;
    return 0;
  }

  // Wire shall be held by slave for 60-240 us
  if (!wire1Poll4Release(60)) { // (246) 254 us = 4*60 + 14
    wire1state = IDLE;
    return -1; // The wire was never released
  } else if (wire1Poll4Hold(58)) { // Wait out the rest of the slot
    wire1state = IDLE;
    return -2;
  } else {
    wire1state = ROM_COMMAND;
    return 1; // Success!
  }
}

/**
 * Forces slaves into next state and then reads the returned value
 * @return  0 if sampled low any amount of times; otherwise 0xFF
 */
uint8_t wire1ReadBit(void) {
  uint8_t bittest = 0;
  uint8_t i = 4;

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
 * Internal function that implements the one-wire search algorithm. Used for
 * both the search ROM and alarm search.
 *
 * @param  addrOut      The output address for the returned ROM (8 bytes)
 * @param  addrStart    Starting point for searching ROM address from
 *                      (shall be the last found ROM address if continuing
 *                      a search). Can be a pointer to the same location as the
 *                      output address.
 * @param  lastConfPos  The bit position of the conflict bit in the last
 *                      search (shall be above 63 unsigned if starting a new
 *                      search, otherwise the returned value from the last
 *                      search)
 * @param  rom_command  The command to issue before commencing the search
 *                      algorithm.
 * @return              A negative value if error, 0-63 if a device was found
 *                      and there exists a device with ROM address with a
 *                      higher address which conflicts at that bit, 64 if a
 *                      device was found but no devices had a higher address.
 */
static int8_t wire1Search(
  uint8_t * addrOut,
  uint8_t * addrStart,
  const uint8_t lastConfPos,
  const uint8_t rom_command
) {
  // Detect if there are any devices connected and init ROM command
  if (wire1Reset() != 1) {
    return -1; // Nothing connected!
  }

  // Issue the search ROM command to one-wire devices
  wire1WriteByte(rom_command);

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
      // -1 means that it is uninitialized => find lowest ROM
      } else if (lastConfPos == -1) {
        writeBit = 0;
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

  // Make sure that the ROM was read correctly, otherwise the device will not
  // have been selected
  if (addrOut[W1_ADDR_BYTE_CRC] == crc8(0, W1_CRC_POLYNOMIAL, addrOut, 7)) {
    wire1state = FUNCTION_COMMAND;
    return currConfPos;
  // CRC did not match. Most probably, no device has been selected
  } else {
    wire1state = IDLE;
    return -1;
  }
}

/**
 * Searches the address space for the next larger device address
 * compared to addrStart. Reads ACK and NACK of the ROM bit and decides
 * what branch to choose depending on the conflict position in the last
 * search and the start address.
 *
 * @param  addrOut      The output address for the returned ROM (8 bytes)
 * @param  addrStart    Starting point for searching ROM address from
 *                      (shall be the last found ROM address if continuing
 *                      a search). Can be a pointer to the same location as the
 *                      output address.
 * @param  lastConfPos  The bit position of the conflict bit in the last
 *                      search (shall be above 63 unsigned if starting a new
 *                      search, otherwise the returned value from the last
 *                      search)
 * @return              A negative value if error, 0-63 if a device was found
 *                      and there exists a device with ROM address with a
 *                      higher address which conflicts at that bit, 64 if a
 *                      device was found but no devices had a higher address.
 */
int8_t wire1SearchLargerROM(
  uint8_t * addrOut,
  uint8_t * addrStart,
  const uint8_t lastConfPos
) {
  return wire1Search(addrOut, addrStart, lastConfPos, W1_ROMCMD_SEARCH);
}

/**
 * Searches the address space for the next larger device address
 * compared to addrStart. Reads ACK and NACK of the ROM bit and decides
 * what branch to choose depending on the conflict position in the last
 * search and the start address.
 *
 * @param  addrOut      The output address for the returned ROM (8 bytes)
 * @param  addrStart    Starting point for searching ROM address from
 *                      (shall be the last found ROM address if continuing
 *                      a search). Can be a pointer to the same location as the
 *                      output address.
 * @param  lastConfPos  The bit position of the conflict bit in the last
 *                      search (shall be above 63 unsigned if starting a new
 *                      search, otherwise the returned value from the last
 *                      search)
 * @return              A negative value if no alarms were triggered;
 *                      0-63 if an alarm was triggered and there is another
 *                      triggered alarm for a device with a higher ROM; 64 if an
 *                      alarm was triggered for a device, but for no devices
 *                      that had a higher ROM.
 */
int8_t wire1AlarmSearchLargerROM(
  uint8_t * addrOut,
  uint8_t * addrStart,
  const uint8_t lastConfPos
) {
  return wire1Search(addrOut, addrStart, lastConfPos, W1_ROMCMD_ALARM);
}

/**
 * Read the ROM address of the one-wire device
 * (will ONLY work if there is only one slave connected!)
 * @param addr  Pointer to an 8-byte array where the read ROM address shall be store
 * @return      Whether the function call succeeded or not: 0 - OK; -1 - no
 *              device present; 1 - calculated CRC mismatch
 */
int8_t wire1ReadSingleROM(uint8_t *const addr) {
  wire1Reset();
  if (wire1state != ROM_COMMAND)
    return -1;
  wire1WriteByte(W1_ROMCMD_READ);
  for (int i = 0; i < 8; i++) {
    addr[i] = wire1ReadByte();
  }
  // Make sure that the ROM was read correctly, otherwise the device will not
  // have been selected
  if (addr[W1_ADDR_BYTE_CRC] == crc8(0, W1_CRC_POLYNOMIAL, addr, 7)) {
    wire1state = FUNCTION_COMMAND;
    return 0;
  } else {
    wire1state = IDLE;
    return 1;
  }
}

/**
 * Sends the ROM address of a device that we want to access.
 * @param addr  Pointer to an 8-byte array where the ROM address is stored
 * @return      Whether the function call succeeded or not: 0 - OK; -1 - no
 *              device present
 */
int8_t wire1MatchROM(uint8_t *const addr) {
  wire1Reset();
  if (wire1state != ROM_COMMAND)
    return -1;
  wire1WriteByte(W1_ROMCMD_MATCH);
  for (int i = 0; i < 8; i++) {
    wire1WriteByte(addr[i]);
  }
  wire1state = FUNCTION_COMMAND;
  return 0;
}

/**
 * Skips ROM addressing so that all devices can be written to simultaneously
 * @return      Whether the function call succeeded or not: 0 - OK; -1 - no
 *              device present
 */
int8_t wire1SkipROM(void) {
  wire1Reset();
  if (wire1state != ROM_COMMAND)
    return -1;
  wire1WriteByte(W1_ROMCMD_SKIP);
  wire1state = FUNCTION_COMMAND;
  return 0;
}

/**
 * Read the power supply status of a one-wire device
 * @return 1 if any of the addressed slaves use parasite power; 0 if not; -2 if
 *         not starting in the correct state.
 */
int8_t wire1ReadPowerSupply(void) {
  if (wire1state != FUNCTION_COMMAND)
    return -2;
  wire1WriteByte(W1_FUNC_PARASITE_POWER);
  // Check if at least one of the responding slaves use parasite power
  uint8_t parasite_power = !wire1ReadBit();
  wire1state = IDLE;
  return parasite_power;
}

/**
 * Calculate an 8-bit CRC for size number of byte of data. Shifts the data
 * from MSB to LSB and XOR:s the polynomial each time the LSB of the remainder
 * XOR the LSB of the shifted data is 1.
 *
 * @param  crcIn       The initialization for the CRC (input residual)
 * @param  polynomial  The polynomial (XOR byte)
 * @param  data        Array of data to calculate CRC for
 * @param  size        Number of byte in the data
 * @return             The calculated CRC
 */
uint8_t crc8(
  uint8_t crcIn,
  uint8_t polynomial,
  uint8_t *const data,
  uint8_t const size
) {
  // Input CRC can be used for chaining several CRC calculations
  uint8_t remainder = crcIn;
  for (int i = 0; i < size; i++) {
    for (int j = 0; j < 8; j++) {
      uint8_t data_bit_j = (data[i] >> j)  & BV(0);
      uint8_t rem_bit_0  = remainder & BV(0);

      // XOR between last remainder bit and bit j of the data
      remainder >>= 1;
      if (data_bit_j ^ rem_bit_0) {
        remainder ^= polynomial;
      }
    }
  }
  return remainder;
}