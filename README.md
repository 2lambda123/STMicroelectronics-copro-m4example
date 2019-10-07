# copro_m4example #

This application is a basic example of a proprietary firmware running on Cortex-M4 and using several resources : TIM2, TIM7, DAC1, ADC1, DMA2, CRC2, HASH2 CRY2.
A sinusoid is generated on the DAC which is looped through a wire to the ADC input.
The ADC results are sent to the application and displayed, and CRC32, SHA-256, AES-ECB results are sent to the application which checks the results.


## License ##

This module is distributed under STMicroelectronics License, which can be found in the [LICENSE](./LICENSE) file.
