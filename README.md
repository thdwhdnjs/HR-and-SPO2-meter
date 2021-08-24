# HR-and-SPO2-meter
1. 타킷보드: NUCLEO-F401RE
2. MAX30102와 ssd1306은 I2C로 동작하며 병렬연결하여 I2C 1번채널에 연결
3. STM32CubeMX사용시 I2C는 1채널, 고속모드, DMA, DMA tx인터럽트설정
