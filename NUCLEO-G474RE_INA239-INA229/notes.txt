1. Clock Polarity (CPOL) = High
2. When using INA239, some are not present compared to NA229, some data will be zero.
3. When reading, if number of bit exceeded the specified number, the remaining reading will be all zero.
4. When reading, the data sent wont matter
5. Hal_delay is not allowed inside a callback void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)