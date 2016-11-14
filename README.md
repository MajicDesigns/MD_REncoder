This is an adaptation of Ben Buxton's excellent [rotary library](http://www.buxtronix.net/2011/10/rotary-encoders-done-properly.html) and implements additional features for encoder rotation speed.

## Features
* Debounce handling with support for high rotation speeds
* Correctly handles direction changes mid-step
* Checks for valid state changes for more robust counting and noise immunity
* Interrupt based or polling in loop()
* Counts full-steps (default) or half-steps
* Calculates speed of rotation
