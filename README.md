# STM32 WS2812B Display demo
STM32 with display on WS2812B (demo project) 

 - Assume that we have STM32L152RB as MCU
 - Assume that we have CY8CMBR3108 as touch sensor (with 4 buttons)
 - Assume that we have something like HC06 as Bluetooth UART pipe or any other as transparent pipe.
 - Use WS2812B as 2 segments by 20px every.
 - Use 20*5 WS2812B for display speed

CY8CMBR3108 I2C pins are connected to I2C1 of STM32L152RB (SDA -> PB9, SCL -> PB8), also pin ^HI is connected to PB0.


Phone <-> Controller message format
-----------------------------------------------------
|------| Start | Length | OpCode |  Payload  | CRC8 |
| Byte | 0x55  |   0    |   0    | len bytes | 0xXX |
-----------------------------------------------------

Possible op codes:
	0x00 -- Status, payload: 0 or !=0 if error
	0x01 -- Write segment, payload: segment(1b), enable(1b), temp_sec(1b), color_r(1b), color_g(1b), color_b(1b)
	0x02 -- Read segment, payload is same as for write

Write command sould be confirmed with status cmd.
CRC is CRC8-CCITT algo, and should be calculated from start field
