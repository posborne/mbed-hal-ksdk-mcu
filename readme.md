# mbed HAL ksdk mcu

This is the HAL implementation for mcu which are supported by KSDK. Part of this module is [Freescale KSDK](http://www.freescale.com/tools/software-and-tools/run-time-software/kinetis-software-and-tools/development-platforms-with-mbed/software-development-kit-for-kinetis-mcus:KINETIS-SDK) version 1.0.0.

## Freescale KSDK code

KSDK code is located in under TARGET_KSDK_CODE. We had to do couple of modifications, to be able to use it in mbed.

- this module does not contain entire KSDK. Only few drivers and HAL are included.
- a header file like fsl_peripheral_features (peripheral here is any freescale mcu peripheral, lpuart, dspi, ..) defines features for MCU. In original KSDK features files, it would display an error (No valid CPU defined), thus we defined MBED_NO_PERIPHERAL and removed the error.
