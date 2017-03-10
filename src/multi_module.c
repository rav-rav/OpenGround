#include <stdlib.h>
#include <stdint.h>
#include <string.h>

#include "multi_module.h"

void randomSeed(unsigned int seed)
{
  if (seed != 0) {
    srandom(seed);
  }
}

#define random(x) random()
/*
long random(long howbig)
{
  if (howbig == 0) {
    return 0;
  }
  return 0 % howbig;
}
*/

long map(long x, long in_min, long in_max, long out_min, long out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

#define PROGMEM
#define STM32_BOARD	// Let's automatically select this board if arm is selected since this is the only one for now...
#define STM32_EVO

#define _BV(x) ((1 << x))
#define highByte(x) ( (x) >> (8) ) // keep upper 8 bits
#define lowByte(x) ( (x) & (0xff) ) // keep lower 8 bits

#define pgm_read_byte_near(address_short) *address_short //myfunc((uint16_t)(address_short)) Read a byte from the program space with a 16-bit (near) address.
#define pgm_read_word(address_short) *address_short //myfunc((uint16_t)(address_short)) Read a word from the program space with a 16-bit (near) address.

//AFHDS2A_a7105.ino
//ReadAFHDS2A   eeprom_write_byte((EE_ADDR)(temp+i),rx_id[i]); //TODO: rav - implement me
//initAFHDS2A   rx_id[i]=eeprom_read_byte((EE_ADDR)(temp+i));  //TODO: rav - implement me

#define delay(x)
#define	cli() 			noInterrupts()
#define	sei() 			interrupts()
#define	delayMilliseconds(x) delay(x)
#define	delayMicroseconds(x) delay(x)

#include "_Config.h"
#undef ENABLE_PPM
#undef MULTI_TELEMETRY
#undef MULTI_STATUS

#include "Pins.h"
#include "TX_Def.h"

#include <SPI.h>	

// Protocol variables
uint8_t  cyrfmfg_id[6];//for dsm2 and devo
uint8_t  rx_tx_addr[5];
uint8_t  rx_id[4];
uint8_t  phase;
uint16_t bind_counter;
uint8_t  bind_phase;
uint8_t  binding_idx;
uint16_t packet_period;
uint8_t  packet_count;
uint8_t  packet_sent;
uint8_t  packet_length;
uint8_t  hopping_frequency[50];
uint8_t  *hopping_frequency_ptr;
uint8_t  hopping_frequency_no=0;
uint8_t  rf_ch_num;
uint8_t  throttle, rudder, elevator, aileron;
uint8_t  flags;
uint16_t crc;
uint8_t  crc8;
uint16_t seed;
//
uint16_t state;
uint8_t  len;
uint8_t  RX_num;

uint8_t SPI_Write(uint8_t command) {
  return spi_tx(command);
}

uint8_t SPI_Read(void){
  return spi_rx();
}

uint8_t SPI_SDI_Read(void);

// Convert 32b id to rx_tx_addr
static void set_rx_tx_addr(uint32_t id) { // Used by almost all protocols
	rx_tx_addr[0] = (id >> 24) & 0xFF;
	rx_tx_addr[1] = (id >> 16) & 0xFF;
	rx_tx_addr[2] = (id >>  8) & 0xFF;
	rx_tx_addr[3] = (id >>  0) & 0xFF;
	rx_tx_addr[4] = (rx_tx_addr[2]&0xF0)|(rx_tx_addr[3]&0x0F);
}


// Callback
typedef uint16_t (*void_function_t) (void);//pointer to a function with no parameters which return an uint16_t integer
void_function_t remote_callback = 0;

//Global constants/variables
uint32_t MProtocol_id;//tx id,
uint32_t MProtocol_id_master;
//
uint16_t counter;
uint8_t  channel;
uint8_t  packet[40];

#define NUM_CHN 16
// Servo data
uint16_t Servo_data[NUM_CHN];
uint8_t  Servo_AUX;
uint16_t servo_max_100,servo_min_100,servo_max_125,servo_min_125;
uint16_t servo_mid;


#if defined(FRSKYX_CC2500_INO) || defined(SFHSS_CC2500_INO)
	uint8_t calData[48];
#endif

//Channel mapping for protocols
const uint8_t CH_AETR[]={AILERON, ELEVATOR, THROTTLE, RUDDER, AUX1, AUX2, AUX3, AUX4, AUX5, AUX6, AUX7, AUX8, AUX9, AUX10};
const uint8_t CH_TAER[]={THROTTLE, AILERON, ELEVATOR, RUDDER, AUX1, AUX2, AUX3, AUX4, AUX5, AUX6, AUX7, AUX8};
const uint8_t CH_RETA[]={RUDDER, ELEVATOR, THROTTLE, AILERON, AUX1, AUX2, AUX3, AUX4, AUX5, AUX6, AUX7, AUX8};
const uint8_t CH_EATR[]={ELEVATOR, AILERON, THROTTLE, RUDDER, AUX1, AUX2, AUX3, AUX4, AUX5, AUX6, AUX7, AUX8};

// Mode_select variables
uint8_t mode_select;
uint8_t protocol_flags=0,protocol_flags2=0;

//Serial protocol
uint8_t sub_protocol;
uint8_t protocol;
uint8_t option;
uint8_t cur_protocol[3];
uint8_t prev_option;
uint8_t prev_power=0xFD; // unused power value

// Telemetry
#define MAX_PKT 29
uint8_t pkt[MAX_PKT];//telemetry receiving packets

	uint8_t v_lipo1;
	uint8_t v_lipo2;
	uint8_t RX_RSSI;
	uint8_t TX_RSSI;
	uint8_t RX_LQI;
	uint8_t TX_LQI;
	uint8_t telemetry_link=0; 
	uint8_t telemetry_counter=0;
	uint8_t telemetry_lost;

//#include "SPI.ino"

#include "A7105_SPI.ino"
#include "CC2500_SPI.ino"
#include "CYRF6936_SPI.ino"
#include "NRF24l01_SPI.ino"

#include "Common.ino"
#include "Arduino.ino"

#include "AFHDS2A_a7105.ino"
#include "ASSAN_nrf24l01.ino"
#include "Bayang_nrf24l01.ino"
#include "CG023_nrf24l01.ino"
#include "CX10_nrf24l01.ino"
#include "Devo_cyrf6936.ino"
#include "DSM_cyrf6936.ino"
#include "ESky_nrf24l01.ino"
#include "FlySky_a7105.ino"
#include "FQ777_nrf24l01.ino"
#include "FrSkyD_cc2500.ino"
#include "FrSkyV_cc2500.ino"
#include "FrSkyX_cc2500.ino"
#include "FY326_nrf24l01.ino"
#include "GW008_nrf24l01.ino"
#include "Hisky_nrf24l01.ino"
#include "Hontai_nrf24l01.ino"
#include "Hubsan_a7105.ino"
#include "J6Pro_cyrf6936.ino"
#include "KN_nrf24l01.ino"
#include "MJXQ_nrf24l01.ino"
#include "MT99xx_nrf24l01.ino"
#include "Q303_nrf24l01.ino"
#include "SFHSS_cc2500.ino"
#include "SHENQI_nrf24l01.ino"
#include "SLT_nrf24l01.ino"
#include "Symax_nrf24l01.ino"
#include "V2X2_nrf24l01.ino"
#include "WK2x01_cyrf6936.ino"
#include "YD717_nrf24l01.ino"

//#include "Telemetry.ino"

void stuff() {
/*  uint16_t next_callback;
  PE1_off;	//antenna RF2
  PE2_on;
  next_callback = initFrSkyX();
  remote_callback = ReadFrSkyX;
  remote_callback();
  */
}
/*
int main()
{
	init();
	setup();
	for(;;)
	{
		loop();
	}
}
*/
