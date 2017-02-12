#include <util/delay.h>
#include <avr/io.h>
#include "wire1.h"

// MACROS for being able to use convenient DB-"variables"
#define DBPORT_LETTER  B
#define DBPP           0
#define CONCAT(a, b)      a ## b
#define CONCAT_EXP(a, b)  CONCAT(a, b)
#define DBDDR             CONCAT_EXP(DDR,  DBPORT_LETTER)
#define DBPIN             CONCAT_EXP(PIN,  DBPORT_LETTER)
#define DBPORT            CONCAT_EXP(PORT, DBPORT_LETTER)

#define STRINGIFY(a) #a
#define STRINGIFY_EXPAND(a) STRINGIFY(a)

/**
 * Drives the wire low.
 */
inline void wire1Hold(void) {
  DBPORT &= ~BV(DBPP); // Drive low/remove pullup
  DBDDR  |=  BV(DBPP); // Pin as output
}

/**
 * Releases the wire and lets it be pulled up.
 *  Also adds the internal pullup to strengthen the pull up "force".
 */
inline void wire1Release(void) {
  DBDDR  &= ~BV(DBPP); // Pin as input
  DBPORT |=  BV(DBPP); // Add pullup
}

/**
 * Polls the wire a maximum of times, or until someone drives it low
 * @param  nloops  Number of times to poll
 * @return         0 if it goes low within the time limit, otherwise DBPP
 */
inline uint8_t wire1Poll4Hold(uint8_t nloops) {
  uint8_t readVal = DBPIN & BV(DBPP);
  uint8_t i = 0;
  while (i < nloops && readVal) {
    readVal &= DBPIN & BV(DBPP);
    i++;
  }
  return readVal;
}

/**
 * Polls the wire a maximum of times, or until it is released
 * @param  nloops  Number of times to poll
 * @return         DBPP if it goes high within the time limit, otherwise 0
 */
inline uint8_t wire1Poll4Release(uint8_t nloops) {
  uint8_t readVal = DBPIN & BV(DBPP);
  uint8_t i = 0;
  while (i < nloops && !readVal) {
    readVal |= DBPIN & BV(DBPP);
    i++;
  }
  return readVal;
}

/**
 * Resets all 1-wire devices and checks if there are any slaves that responds.
 * @return  negative if error, 0 if no slave responds, 1 if a slave responds
 */
uint8_t  wire1Reset(void) {
  // Hold for 450+ us to reset
  wire1Hold();
  if (wire1Poll4Release(550)) {
    return -1; // Wire went high even though we're pulling it down!
  }
  wire1Release();

  // Keep released for 15-60 us before response
  if (!wire1Poll4Hold(10)) {
    return -2; // The wire was held too early, someone is driving it simultaneously
  }
  if (wire1Poll4Hold(60)) {
    return 0; // The wire never went low
  }
  // Held 60-240 us by slave
  if (!wire1Poll4Release(300)) {
    return -3; // The wire was never released
  } else {
    return 1; // Success!
  }
}

/**
 * Forces slaves into next state and then reads the returned value
 * @return  0 if wire was held low, otherwise nonzero
 */
uint8_t  wire1Read(void) {
  // Hold for >1 us to update state of slaves
  wire1Hold();
  if (wire1Poll4Release(1)) {
    return -1; // Wire went high even though we're pulling it down!
  }
  wire1Release();

  uint8_t readVal = wire1Poll4Release(10);
  _delay_us(45);
  return readVal;
}

/**
 * Forces slaves into next state and then send a 1 or 0
 * @param bit [boolean] Send a 0 if zero, otherwise send 1
 */
void wire1Write(uint8_t bit) {
  // Hold for >1 us to update state of slaves
  wire1Hold();
  if (wire1Poll4Release(1)) {
    return; // Wire went high even though we're pulling it down!
  }

  // Release before delaying if sending 1
  if (bit) {
    wire1Release();
  }

  _delay_us(60);
  wire1Release();
}