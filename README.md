# sx128x_driver
Portable SX128x driver written in C++.

Supported chips: `SX1280`, `SX1281`, `SX1282`. Only `SX1280` is tested.

## Requirements
- C++14

## Features
- Easy to use
- Should work on nearly all platforms
- Multiple instances
- No macros, no global scope shit, and thus no collisions with other libs

## Usage

Write your own hardware abstraction layer (HAL) class and implement the `Hal*` functions below:

- `HalGpioRead`
- `HalGpioWrite`
- `HalSpiTransfer` - This should take care of chip select signal

**It's very simple. Right?**

And these if you have RF switches / external RF PA (optional): 

- `HalPreTx` - Called before a TX operation
- `HalPreRx` - Called before a RX operation
- `HalPostTx` - Called when (1) `TX_DONE` (2) `TX_TIMEOUT` (3) before a RX operation
- `HalPostRx` - Called when (1) `RX_TIMEOUT` (2) before a TX operation


**Enable DIOs & process GPIO interrupts by yourself.** When DIO fires, simply call `SX1280::ProcessIrqs()`. Everything is thread-safe.
 
That's all. The code is well documented. Most usages are exactly same as the SX1280 mbed driver.

## Examples
[sx128x_linux_driver](https://github.com/YukiWorkshop/sx128x_linux_driver) is a good demonstration of how to use this driver in Linux.

## License
LGPLv3

## Acknowledgements
This library is based on the SX1280 mbed driver by Semtech S.A. See `LICENSE-SEMTECH.txt` for details.