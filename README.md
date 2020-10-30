# sx1280_driver
Portable SX1280 driver written in C++.

## Requirements
- C++14

## Features
- Should work on nearly all platforms
- Multiple instances
- No macros, and no collisions

## Usage
Write your own hardware abstraction layer (HAL) class and implement the `Hal*` functions below:

- `HalGpioRead`
- `HalGpioWrite`
- `HalSpiTransfer` - This should take care of chip select signal

And these if you have RF switches / external RF PA (optional): 
- `HalPreTx` - Called before a TX operation
- `HalPreRx` - Called before a RX operation
- `HalPostTx` - Called when (1) `TX_DONE` (2) `TX_TIMEOUT` (3) before a RX operation
- `HalPostRx` - Called when (1) `RX_TIMEOUT` (2) before a TX operation


Process GPIO interrupts by yourself. When DIO fires, simply call `SX1280::ProcessIrqs()`. Everything is thread-safe.
 
That's all. The code is well documented. Most usages are exactly same as the SX1280 mbed driver.

## License
LGPLv3

## Acknowledgements
This library is based on the SX1280 mbed driver by Semtech S.A. See `LICENSE-SEMTECH.txt` for details.