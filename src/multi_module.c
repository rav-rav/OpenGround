#include <stdlib.h>
#include <stdint.h>
#include <string.h>

#include "multi_module.h"

void randomSeed(unsigned int seed) {
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

long map(long x, long in_min, long in_max, long out_min, long out_max) {
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
uint8_t cyrfmfg_id[6]; //for dsm2 and devo
uint8_t rx_tx_addr[5];
uint8_t rx_id[4];
uint8_t phase;
uint16_t bind_counter;
uint8_t bind_phase;
uint8_t binding_idx;
uint16_t packet_period;
uint8_t packet_count;
uint8_t packet_sent;
uint8_t packet_length;
uint8_t hopping_frequency[50];
uint8_t *hopping_frequency_ptr;
uint8_t hopping_frequency_no = 0;
uint8_t rf_ch_num;
uint8_t throttle, rudder, elevator, aileron;
uint8_t flags;
uint16_t crc;
uint8_t crc8;
uint16_t seed;
//
uint16_t state;
uint8_t len;
uint8_t RX_num;

uint8_t SPI_Write(uint8_t command) {
    return spi_tx(command);
}

uint8_t SPI_Read(void) {
    return spi_rx();
}

uint8_t SPI_SDI_Read(void);

// Convert 32b id to rx_tx_addr
static void set_rx_tx_addr(uint32_t id) { // Used by almost all protocols
    rx_tx_addr[0] = (id >> 24) & 0xFF;
    rx_tx_addr[1] = (id >> 16) & 0xFF;
    rx_tx_addr[2] = (id >> 8) & 0xFF;
    rx_tx_addr[3] = (id >> 0) & 0xFF;
    rx_tx_addr[4] = (rx_tx_addr[2] & 0xF0) | (rx_tx_addr[3] & 0x0F);
}

// Callback
typedef uint16_t (*void_function_t)(void); //pointer to a function with no parameters which return an uint16_t integer
void_function_t remote_callback = 0;

//Global constants/variables
uint32_t MProtocol_id; //tx id,
uint32_t MProtocol_id_master;
//
uint16_t counter;
uint8_t channel;
uint8_t packet[40];

#define NUM_CHN 16
// Servo data
uint16_t Servo_data[NUM_CHN];
uint8_t Servo_AUX;
uint16_t servo_max_100, servo_min_100, servo_max_125, servo_min_125;
uint16_t servo_mid;

#if defined(FRSKYX_CC2500_INO) || defined(SFHSS_CC2500_INO)
uint8_t calData[48];
#endif

//Channel mapping for protocols
const uint8_t CH_AETR[] = { AILERON, ELEVATOR, THROTTLE, RUDDER, AUX1, AUX2, AUX3, AUX4, AUX5, AUX6, AUX7, AUX8, AUX9, AUX10 };
const uint8_t CH_TAER[] = { THROTTLE, AILERON, ELEVATOR, RUDDER, AUX1, AUX2, AUX3, AUX4, AUX5, AUX6, AUX7, AUX8 };
const uint8_t CH_RETA[] = { RUDDER, ELEVATOR, THROTTLE, AILERON, AUX1, AUX2, AUX3, AUX4, AUX5, AUX6, AUX7, AUX8 };
const uint8_t CH_EATR[] = { ELEVATOR, AILERON, THROTTLE, RUDDER, AUX1, AUX2, AUX3, AUX4, AUX5, AUX6, AUX7, AUX8 };

// Mode_select variables
uint8_t mode_select;
uint8_t protocol_flags = 0, protocol_flags2 = 0;

//Serial protocol
uint8_t sub_protocol;
uint8_t protocol;
uint8_t option;
uint8_t cur_protocol[3];
uint8_t prev_option;
uint8_t prev_power = 0xFD; // unused power value

// Telemetry
#define MAX_PKT 29
uint8_t pkt[MAX_PKT]; //telemetry receiving packets

uint8_t v_lipo1;
uint8_t v_lipo2;
uint8_t RX_RSSI;
uint8_t TX_RSSI;
uint8_t RX_LQI;
uint8_t TX_LQI;
uint8_t telemetry_link = 0;
uint8_t telemetry_counter = 0;
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

void multi_module_init() {
}

// Protocol start
static void protocol_init() {
    static uint16_t next_callback;
    if (IS_WAIT_BIND_off) {
        remote_callback = 0;            // No protocol
        next_callback = 0;                // Default is immediate call back
        modules_reset();                // Reset all modules

        //Set global ID and rx_tx_addr
        MProtocol_id = RX_num + MProtocol_id_master;
        set_rx_tx_addr(MProtocol_id);

        if (IS_BIND_BUTTON_FLAG_on)
            AUTOBIND_FLAG_on;
        if (IS_AUTOBIND_FLAG_on)
            BIND_IN_PROGRESS;           // Indicates bind in progress for blinking bind led
        else
            BIND_DONE;

        PE1_off;                         //CC2500 antenna RF2 by default
        PE2_on;                        //CC2500 antenna RF2 by default

        switch (protocol)                // Init the requested protocol
        {
#ifdef A7105_INSTALLED
#if defined(FLYSKY_A7105_INO)
        case MODE_FLYSKY:
            PE1_off;    //antenna RF1
            next_callback = initFlySky();
            remote_callback = ReadFlySky;
            break;
#endif
#if defined(AFHDS2A_A7105_INO)
        case MODE_AFHDS2A:
            PE1_off;    //antenna RF1
            next_callback = initAFHDS2A();
            remote_callback = ReadAFHDS2A;
            break;
#endif
#if defined(HUBSAN_A7105_INO)
        case MODE_HUBSAN:
            PE1_off;    //antenna RF1
            if (IS_BIND_BUTTON_FLAG_on)
                random_id(10, true); // Generate new ID if bind button is pressed.
            next_callback = initHubsan();
            remote_callback = ReadHubsan;
            break;
#endif
#endif
#ifdef CC2500_INSTALLED
#if defined(FRSKYD_CC2500_INO)
        case MODE_FRSKYD:
            PE1_off;    //antenna RF2
            PE2_on;
            next_callback = initFrSky_2way();
            remote_callback = ReadFrSky_2way;
            break;
#endif
#if defined(FRSKYV_CC2500_INO)
        case MODE_FRSKYV:
            PE1_off;    //antenna RF2
            PE2_on;
            next_callback = initFRSKYV();
            remote_callback = ReadFRSKYV;
            break;
#endif
#if defined(FRSKYX_CC2500_INO)
        case MODE_FRSKYX:
            PE1_off;    //antenna RF2
            PE2_on;
            next_callback = initFrSkyX();
            remote_callback = ReadFrSkyX;
            break;
#endif
#if defined(SFHSS_CC2500_INO)
        case MODE_SFHSS:
            PE1_off;    //antenna RF2
            PE2_on;
            next_callback = initSFHSS();
            remote_callback = ReadSFHSS;
            break;
#endif
#endif
#ifdef CYRF6936_INSTALLED
#if defined(DSM_CYRF6936_INO)
        case MODE_DSM:
            PE2_on; //antenna RF4
            next_callback = initDsm();
            //Servo_data[2]=1500;//before binding
            remote_callback = ReadDsm;
            break;
#endif
#if defined(DEVO_CYRF6936_INO)
        case MODE_DEVO:
#ifdef ENABLE_PPM
            if(mode_select) //PPM mode
            {
                if(IS_BIND_BUTTON_FLAG_on)
                {
                    eeprom_write_byte((EE_ADDR)(30+mode_select),0x00);  // reset to autobind mode for the current model
                    option=0;
                }
                else
                {
                    option=eeprom_read_byte((EE_ADDR)(30+mode_select)); // load previous mode: autobind or fixed id
                    if(option!=1) option=0;// if not fixed id mode then it should be autobind
                }
            }
#endif //ENABLE_PPM
            PE2_on; //antenna RF4
            next_callback = DevoInit();
            remote_callback = devo_callback;
            break;
#endif
#if defined(WK2x01_CYRF6936_INO)
        case MODE_WK2x01:
#ifdef ENABLE_PPM
            if(mode_select) //PPM mode
            {
                if(IS_BIND_BUTTON_FLAG_on)
                {
                    eeprom_write_byte((EE_ADDR)(30+mode_select),0x00);  // reset to autobind mode for the current model
                    option=0;
                }
                else
                {
                    option=eeprom_read_byte((EE_ADDR)(30+mode_select)); // load previous mode: autobind or fixed id
                    if(option!=1) option=0;// if not fixed id mode then it should be autobind
                }
            }
#endif //ENABLE_PPM
            PE2_on; //antenna RF4
            next_callback = WK_setup();
            remote_callback = WK_cb;
            break;
#endif
#if defined(J6PRO_CYRF6936_INO)
        case MODE_J6PRO:
            PE2_on; //antenna RF4
            next_callback = initJ6Pro();
            remote_callback = ReadJ6Pro;
            break;
#endif
#endif
#ifdef NRF24L01_INSTALLED
#if defined(HISKY_NRF24L01_INO)
        case MODE_HISKY:
            next_callback = initHiSky();
            remote_callback = hisky_cb;
            break;
#endif
#if defined(V2X2_NRF24L01_INO)
        case MODE_V2X2:
            next_callback = initV2x2();
            remote_callback = ReadV2x2;
            break;
#endif
#if defined(YD717_NRF24L01_INO)
        case MODE_YD717:
            next_callback = initYD717();
            remote_callback = yd717_callback;
            break;
#endif
#if defined(KN_NRF24L01_INO)
        case MODE_KN:
            next_callback = initKN();
            remote_callback = kn_callback;
            break;
#endif
#if defined(SYMAX_NRF24L01_INO)
        case MODE_SYMAX:
            next_callback = initSymax();
            remote_callback = symax_callback;
            break;
#endif
#if defined(SLT_NRF24L01_INO)
        case MODE_SLT:
            next_callback = initSLT();
            remote_callback = SLT_callback;
            break;
#endif
#if defined(CX10_NRF24L01_INO)
        case MODE_Q2X2:
            sub_protocol |= 0x08;     // Increase the number of sub_protocols for CX-10
        case MODE_CX10:
            next_callback = initCX10();
            remote_callback = CX10_callback;
            break;
#endif
#if defined(CG023_NRF24L01_INO)
        case MODE_CG023:
            next_callback = initCG023();
            remote_callback = CG023_callback;
            break;
#endif
#if defined(BAYANG_NRF24L01_INO)
        case MODE_BAYANG:
            next_callback = initBAYANG();
            remote_callback = BAYANG_callback;
            break;
#endif
#if defined(ESKY_NRF24L01_INO)
        case MODE_ESKY:
            next_callback = initESKY();
            remote_callback = ESKY_callback;
            break;
#endif
#if defined(MT99XX_NRF24L01_INO)
        case MODE_MT99XX:
            next_callback = initMT99XX();
            remote_callback = MT99XX_callback;
            break;
#endif
#if defined(MJXQ_NRF24L01_INO)
        case MODE_MJXQ:
            next_callback = initMJXQ();
            remote_callback = MJXQ_callback;
            break;
#endif
#if defined(SHENQI_NRF24L01_INO)
        case MODE_SHENQI:
            next_callback = initSHENQI();
            remote_callback = SHENQI_callback;
            break;
#endif
#if defined(FY326_NRF24L01_INO)
        case MODE_FY326:
            next_callback = initFY326();
            remote_callback = FY326_callback;
            break;
#endif
#if defined(FQ777_NRF24L01_INO)
        case MODE_FQ777:
            next_callback = initFQ777();
            remote_callback = FQ777_callback;
            break;
#endif
#if defined(ASSAN_NRF24L01_INO)
        case MODE_ASSAN:
            next_callback = initASSAN();
            remote_callback = ASSAN_callback;
            break;
#endif
#if defined(HONTAI_NRF24L01_INO)
        case MODE_HONTAI:
            next_callback = initHONTAI();
            remote_callback = HONTAI_callback;
            break;
#endif
#if defined(Q303_NRF24L01_INO)
        case MODE_Q303:
            next_callback = initQ303();
            remote_callback = Q303_callback;
            break;
#endif
#if defined(GW008_NRF24L01_INO)
        case MODE_GW008:
            next_callback = initGW008();
            remote_callback = GW008_callback;
            break;
#endif
#endif
        }
    }

#if defined(WAIT_FOR_BIND) && defined(ENABLE_BIND_CH)
    if ( IS_AUTOBIND_FLAG_on && !( IS_BIND_CH_PREV_on || IS_BIND_BUTTON_FLAG_on || (cur_protocol[1] & 0x80) != 0)) {
        WAIT_BIND_on;
        return;
    }
#endif
    WAIT_BIND_off;
    CHANGE_PROTOCOL_FLAG_off;
    BIND_BUTTON_FLAG_off;                       // do not bind/reset id anymore even if protocol change
}
