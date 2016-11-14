/**
\mainpage Main Page
Rotary Encoder Library for Arduino
----------------------------------

This is an adaptation of Ben Buxton's excellent 
[rotary library](http://www.buxtronix.net/2011/10/rotary-encoders-done-properly.html)
and implements additional features for encoder rotation speed.

Original library copyright 2011 Ben Buxton. Licensed under the GNU GPL Version 3.
Contact: bb@cactii.net

Features
--------
- Debounce handling with support for high rotation speeds
- Correctly handles direction changes mid-step
- Checks for valid state changes for more robust counting and noise immunity
- Interrupt based or polling in loop()
- Counts full-steps (default) or half-steps
- Calculates speed of rotation

Topics
------
- \subpage pageBackground

- \subpage pageLibrary

Revision History
----------------
April 2014 - version 1.0
- Initial implementation from Ben's code
- Cleaned up some compile issues and added begin() method
- Updated and documented
- Added speed functionality

Copyright
---------
This adaptation copyright (C) 2014 Marco Colli. All rights reserved.

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

\page pageBackground Detecting an Encoder's Rotation
_This great explanation is reproduced from Ben Buxton's example code._

A typical mechanical rotary encoder emits a two bit gray code on 3 output pins.
Every step in the output (often accompanied by a physical 'click') generates a 
specific sequence of output codes on the pins.

There are 3 pins used for the rotary encoding - one common and two 'bit' pins
(called A and B).

The following is the typical sequence of codes output when moving from one 
step to the next:

Position | Bit1| Bit2|
--------:|:---:|:---:|
Step1    |  0  |  0  |
  1/4    |  1  |  0  |
  1/2    |  1  |  1  |
  3/4    |  0  |  1  |
Step2    |  0  |  0  |

From this table, we can see that when moving from one 'click' to the next, 
there are 4 changes in the output code. 

- From an initial 0 - 0, Bit1 goes high, Bit0 stays low.
- Then both bits are high, halfway through the step.
- Then Bit1 goes low, but Bit2 stays high.
- Finally at the end of the step, both bits return to 0.

Detecting the direction is easy - the table simply goes in the other direction
(read up instead of down).

To decode this, we use a simple state machine. Every time the output code changes,
it follows state, until finally a full steps worth of code is received (in the 
correct order). At the final 0-0, it returns a value indicating a step in one 
direction or the other.

It's also possible to use 'half-step' mode. This just emits an event at both the 
0-0 and 1-1 positions. This might be useful for some encoders where you want to 
detect all positions. In MD_REncoder.h set ENABLE_HALF_STEP to 1 to enable 
half-step mode.

If an invalid state happens (for example we go from '0-1' straight to '1-0'), the 
state machine resets to the start until 0-0 and the next valid codes occur.

The biggest advantage of using a state machine over other algorithms is that this 
has inherent debounce built in. Other algorithms emit spurious output with switch 
bounce, but this one will simply flip between sub-states until the bounce settles, 
then continue along the state machine. A side effect of debounce is that fast 
rotations can cause steps to be skipped. By not requiring debounce, fast rotations 
can be accurately measured. Another advantage is the ability to properly handle bad 
state, such as due to EMI, etc. It is also a lot simpler than others - a static 
state table and less than 10 lines of logic.

\page pageLibrary The REncoder Library

Compile Time Switches
---------------------

ENABLE_HALF_STEP is 0 by default. Set this to 1 to emit codes when the rotary encoder
is at 11 as well as 00. The default is to emit codes only at 00.

ENABLE_PULLUPS is set to 1 by default. Set this 0 if internal pullup resistors on the
input pins are not required.

ENABLE_SPEED is set to 1 by default. Set this to 0 to disable the code and storage used to 
calculate the speed of the encoder rotation.

Speed Calculation
-----------------
The number of clicks is accumulated during the period defined by setPeriod(). Once the time 
has expired the velocity is calculated in clicks per second by multiplying the number of 
clicks by the number of periods in a second.

speed = ClickCount * (1000 / period)

*/
#ifndef _MD_RENCODER_H
#define _MD_RENCODER_H

#include <Arduino.h>
/**
 * \file
 * \brief Main header file for the MD_Parola library
 */

// Library options
/**
 \def ENABLE_HALF_STEP
 Set this to 1 to emit codes when the rotary encoder is at 11 as well as 00. 
 The default is to emit codes only at 00.
 */
#define ENABLE_HALF_STEP  0

/**
 \def ENABLE_PULLUPS
 Set this 0 if internal pullup resistors on the input pins are not required.
 */
#define ENABLE_PULLUPS    1

/**
 \def ENABLE_SPEED
 Set this to 0 to disable the code and storage used to calculate the encoder rotation speed.
 */
#define ENABLE_SPEED      1

/**
 Set the default sampling period for measuring the speed, in milliseconds. This works best as 
 a whole fraction of 1000 (ie 100, 200, 500, 1000). Longer periods provide some hysteresis 
 which is useful when the encoder is being turned by hand.
 */
#define DEFAULT_PERIOD    500


//  Direction values returned by read() method 
/**
 \def DIR_NONE
  read() return value - No complete step/movement
 */
#define DIR_NONE  0x00
/**
 \def DIR_CW
 read() return value - Clockwise step/movement
 */
#define DIR_CW    0x10
/**
 \def DIR_CCW
 read() return value - Counter-clockwise step/movement
 */
#define DIR_CCW   0x20  

/**
 * Core object for the MD_REncoder library
 */
class MD_REncoder
{
  public:
  /** 
   * Class Constructor.
   *
   * Instantiate a new instance of the class. 
   *
   * \param pinA  the pin number for the encoder A output
   * \param pinB  the pin number for the encoder B output
   */
    MD_REncoder(uint8_t pinA, uint8_t pinB);
    
  /** 
   * Initialize the object.
   *
   * Initialize the object data. This will be called to initialize 
   * new data for the class that cannot be done during the object creation.
   */
    void begin(void);
    
  /** 
   * Read the direction of rotation.
   *
   * Read the direction of rotation inferred from the previous state of the 
   * encoder the current state of the inputs. This method should be called 
   * on a frequent regular basis to ensure smooth encoder inputs.
   *
   * \return One of the DIR_NONE, DIR_CW or DIR_CCW.
   */
    uint8_t read(void);
    
#if ENABLE_SPEED
  /** 
   * Set the sampling period for the speed detection.
   *
   * Set the speed sampling interval in milliseconds. The period 
   * must be greater than 0 and less than 1000. This works best as
   * a whole fraction of 1000 (ie 100, 200, 500, 1000). Longer 
   * periods provide some hysteresis which is useful when the 
   * encoder is being turned by hand.
   *
   * \param t time in millisecond between 0 and 1000 inclusive.
   */
    inline void setPeriod(uint16_t t) { if ((t != 0) && (t <= 1000)) _period = t; };

  /** 
   * Return the speed of the encoder.
   *
   * Calculate the speed (steps per second) for the encoder.
   * The sampling period is set using the setPeriod() method.
   * If the encoder is used to enter numbers or scan through menus, the speed 
   * can be used to accelerate the display (eg, skip larger values for each click).
   *
   * \return The speed in clicks per second.
   */
    inline uint16_t speed(void) { return(_spd); };
#endif

  private:
    // Hardware data
    uint8_t _pinA;      // pin A number
    uint8_t _pinB;      // pin B number
    
    // Encoder value
    uint8_t _state;     // latest state for the encoder

#if ENABLE_SPEED    
    // Velocity data
    uint16_t  _period;  // velocity calculation period
    uint16_t  _count;   // running count of encoder clicks
    uint16_t  _spd;     // last calculated speed (no sign) in clicks/second
    uint32_t  _timeLast;  // last time read
#endif
};

#endif
 
