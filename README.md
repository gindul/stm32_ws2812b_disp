# STM32 WS2812B Display demo
STM32 with display on WS2812B (demo project) 

 - Assume that we have STM32L152RB as MCU
 - Assume that we have CY8CMBR3108 as touch sensor (with 4 buttons)
 - Assume that we have something like HC06 as Bluetooth UART pipe or any other as transparent pipe.
 - Use WS2812B as 2 segments by max 20px every.
 - Use 20*5 WS2812B for display speed

CY8CMBR3108 I2C pins are connected to I2C1 of STM32L152RB (SDA -> PB9, SCL -> PB8), also pin ^HI is connected to PB0.
CY8CMBR3108 has 4 touch buttons: #0: <<<, #1: >>>, #2: Start/Stop, #3: Next Segment


### Phone <-> Controller message format
|      | FLAG1 | FLAG2 | OpCode | Length |  Payload  | CRC8 |
|------|-------|-------|--------|--------|-----------|------|
| Byte |   0   |   1   |   2    |   3    |     n     |  n+4 |
|      | 0x55  | 0xAA  |   0    |   0    | len bytes | 0xXX |

Possible op codes:
	0x00 -- Status, payload: 0 or !=0 if error

	0x01 -- Configure segment lenght, payload: segment(1b), leds_count(1b)
	0x02 -- Start/Stop segment, payload: segment(1b), start_stop(1b)
	0x03 -- Set segment color, payload: segment(1b), color_r(1b), color_g(1b), color_b(1b)
	0x04 -- Set segment temp, payload: segment(1b), temp_sec(1b)

	0x13 -- Read segment color, payload: segment(1b), response payload is same as for set segment color
	0x14 -- Read segment temp, payload: segment(1b), response payload is same as for set segment temp
	

Any write command sould be confirmed with status cmd.
CRC is CRC8-CCITT algo, and should be calculated from start field


### Tasks Description
 - Once user press any touch button the CY8CMBR3108 fall ^HI(PB0) pin. Once MCU was interruptet by touch controller it send a signal that wakeUp buttonsTask. After wakeUp, task will read buttons state from touch controller and change settings of LED segments.
 
 - On receiving byte over UART (in IRQ) put it into queue. BleTask read bytes from osMessageQ one by one (while UART but bytes async into), parse frames, validate it and execute necesary actions.
 
 - For driving leds (WS2812B) I use Timers/PWM. WS2812B expect bits (Manchester encoding) with freq=800kHz (1.25uS per bit), 24 bits for RGB.
	In short words: Send TReset(at least 50uS of 0), make buffer for 2*24 (2 leds x 24bits RGB), fill buffer with PWM width for 1 and 0 bits for first 2 leds, start DMA tor run circular from allocated buffer, once DMA send first half of buffer (birst 24 bytes, as PWM of RGB of first led) it trigger a half send callback and continue to send second half of buffer and in this time we populate first half with 3rd led RGB. Once DMA send a second half of PWMs it trigger a complete callback and continue work from head of buffer and we populate a second half with next led.
	
	A good portion of work with leds will be done in LedsTask().

### TODO:
 [-] Nothing...
