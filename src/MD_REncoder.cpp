/*
MD_REncoder - Library for Rotary Encoders
  
See header file for comments
  
This version copyright (C) 2014 Marco Colli. All rights reserved.

Original library copyright 2011 Ben Buxton. Licenced under the GNU GPL Version 3.
Contact: bb@cactii.net

This library is free software; you can redistribute it and/or
modify it under the terms of the GNU Lesser General Public
License as published by the Free Software Foundation; either
version 3 of the License, or (at your option) any later version.

This library is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
Lesser General Public License for more details.

You should have received a copy of the GNU Lesser General Public
License along with this library; if not, write to the Free Software
Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA 02110-1301 USA
*/

/**
 * \file
 * \brief Implements core MD_REncoder class methods
 */
#include <MD_REncoder.h>

/*
 * The below state table has, for each state (row), the new state
 * to set based on the next encoder output. From left to right in,
 * the table, the encoder outputs are 00, 01, 10, 11, and the value
 * in that position is the new state to set.
 */
#define R_START 0x0

#if ENABLE_HALF_STEP
// Use the half-step state table (emits a code at 00 and 11)
#define R_CCW_BEGIN   0x1
#define R_CW_BEGIN    0x2
#define R_START_M     0x3
#define R_CW_BEGIN_M  0x4
#define R_CCW_BEGIN_M 0x5

const unsigned char ttable[][4] = 
{
  // 00                  01              10            11
  {R_START_M,           R_CW_BEGIN,     R_CCW_BEGIN,  R_START},           // R_START (00)
  {R_START_M | DIR_CCW, R_START,        R_CCW_BEGIN,  R_START},           // R_CCW_BEGIN
  {R_START_M | DIR_CW,  R_CW_BEGIN,     R_START,      R_START},           // R_CW_BEGIN
  {R_START_M,           R_CCW_BEGIN_M,  R_CW_BEGIN_M, R_START},           // R_START_M (11)
  {R_START_M,           R_START_M,      R_CW_BEGIN_M, R_START | DIR_CW},  // R_CW_BEGIN_M 
  {R_START_M,           R_CCW_BEGIN_M,  R_START_M,    R_START | DIR_CCW}  // R_CCW_BEGIN_M
};
#else
// Use the full-step state table (emits a code at 00 only)
#define R_CW_FINAL   0x1
#define R_CW_BEGIN   0x2
#define R_CW_NEXT    0x3
#define R_CCW_BEGIN  0x4
#define R_CCW_FINAL  0x5
#define R_CCW_NEXT   0x6

const unsigned char ttable[][4] = 
{
  // 00         01           10           11
  {R_START,    R_CW_BEGIN,  R_CCW_BEGIN, R_START},           // R_START
  {R_CW_NEXT,  R_START,     R_CW_FINAL,  R_START | DIR_CW},  // R_CW_FINAL
  {R_CW_NEXT,  R_CW_BEGIN,  R_START,     R_START},           // R_CW_BEGIN
  {R_CW_NEXT,  R_CW_BEGIN,  R_CW_FINAL,  R_START},           // R_CW_NEXT
  {R_CCW_NEXT, R_START,     R_CCW_BEGIN, R_START},           // R_CCW_BEGIN
  {R_CCW_NEXT, R_CCW_FINAL, R_START,     R_START | DIR_CCW}, // R_CCW_FINAL
  {R_CCW_NEXT, R_CCW_FINAL, R_CCW_BEGIN, R_START}            // R_CCW_NEXT
};
#endif

MD_REncoder::MD_REncoder(uint8_t pinA, uint8_t pinB):
_pinA (pinA), _pinB (pinB), 
_state(R_START)
#if ENABLE_SPEED
, _spd(0), _count(0), _period(DEFAULT_PERIOD), _timeLast(0)
#endif
{
}

void MD_REncoder::begin(void)
{
  pinMode(_pinA, (ENABLE_PULLUPS ? INPUT_PULLUP : INPUT));
  pinMode(_pinB, (ENABLE_PULLUPS ? INPUT_PULLUP : INPUT));
}

uint8_t MD_REncoder::read(void) 
// Grab state of input pins, determine new state from the pins 
// and state table, and return the emit bits (ie the generated event).
{
  uint8_t pinstate = (digitalRead(_pinB) << 1) | digitalRead(_pinA);
  
  _state = ttable[_state & 0xf][pinstate]; 
  
#if ENABLE_SPEED
  // handle the encoder velocity calc
  if (_state & 0x30) _count++;
  if (millis() - _timeLast >= _period)
  {
    _spd = _count * (1000/_period);
    _timeLast = millis();
    _count = 0;
  }
#endif

  return (_state & 0x30);
}
