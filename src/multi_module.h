#include "Multiprotocol.h"

void randomSeed(unsigned int seed);
long map(long x, long in_min, long in_max, long out_min, long out_max);
void stuff(void);

typedef struct {
} PPM_Parameters;
#define EE_ADDR uint32_t

//generated from .ino files
void A7105_WriteData(uint8_t len, uint8_t channel);
void A7105_ReadData(uint8_t len);
void A7105_WriteReg(uint8_t address, uint8_t data);
uint8_t A7105_ReadReg(uint8_t address);
void A7105_SetTxRxMode(uint8_t mode);
uint8_t A7105_Reset(void);
void A7105_WriteID(uint32_t ida);
void A7105_SetPower(void);
void A7105_Strobe(uint8_t address);
void A7105_Init(void);
void CC2500_WriteReg(uint8_t address, uint8_t data);
void CC2500_ReadData(uint8_t *dpbuffer, uint8_t len);
void CC2500_Strobe(uint8_t state);
void CC2500_WriteData(uint8_t *dpbuffer, uint8_t len);
void CC2500_SetTxRxMode(uint8_t mode);
uint8_t CC2500_Reset(void);
void CC2500_SetPower(void);
void CYRF_WriteRegister(uint8_t address, uint8_t data);
uint8_t CYRF_ReadRegister(uint8_t address);
uint8_t CYRF_Reset(void);
void CYRF_GetMfgData(uint8_t data[]);
void CYRF_SetTxRxMode(uint8_t mode);
void CYRF_ConfigRFChannel(uint8_t ch);
void CYRF_SetPower(uint8_t val);
void CYRF_ConfigCRCSeed(uint16_t crc);
void CYRF_ConfigSOPCode(const uint8_t *sopcodes);
void CYRF_ConfigDataCode(const uint8_t *datacodes, uint8_t len);
void CYRF_WritePreamble(uint32_t preamble);
void CYRF_ReadDataPacketLen(uint8_t dpbuffer[], uint8_t length);
void CYRF_WriteDataPacket(const uint8_t dpbuffer[]);
void CYRF_FindBestChannels(uint8_t *channels, uint8_t len, uint8_t minspace, uint8_t min, uint8_t max);
void NRF24L01_Initialize(void);
void NRF24L01_WriteReg(uint8_t reg, uint8_t data);
void NRF24L01_WriteRegisterMulti(uint8_t reg, uint8_t * data, uint8_t length);
void NRF24L01_WritePayload(uint8_t * data, uint8_t length);
uint8_t NRF24L01_ReadReg(uint8_t reg);
void NRF24L01_FlushTx(void);
void NRF24L01_FlushRx(void);
void NRF24L01_Activate(uint8_t code);
void NRF24L01_SetBitrate(uint8_t bitrate);
void NRF24L01_SetPower(void);
void NRF24L01_SetTxRxMode(enum TXRX_State mode);
void NRF24L01_Reset(void);
uint8_t NRF24L01_packet_ack(void);
void XN297_SetTXAddr(const uint8_t* addr, uint8_t len);
void XN297_SetRXAddr(const uint8_t* addr, uint8_t len);
void XN297_Configure(uint8_t flags);
void XN297_SetScrambledMode(const uint8_t mode);
void XN297_WritePayload(uint8_t* msg, uint8_t len);
void XN297_WriteEnhancedPayload(uint8_t* msg, uint8_t len, uint8_t noack, uint16_t crc_xorout);
void XN297_ReadPayload(uint8_t* msg, uint8_t len);
uint8_t XN297_ReadEnhancedPayload(uint8_t* msg, uint8_t len);
void LT8900_Config(uint8_t preamble_len, uint8_t trailer_len, uint8_t flags, uint8_t crc_init);
void LT8900_SetChannel(uint8_t channel);
void LT8900_SetTxRxMode(enum TXRX_State mode);
void LT8900_BuildOverhead(void);
void LT8900_SetAddress(uint8_t *address, uint8_t addr_size);
uint8_t LT8900_ReadPayload(uint8_t* msg, uint8_t len);
void LT8900_WritePayload(uint8_t* msg, uint8_t len);
uint8_t convert_channel_8b(uint8_t num);
uint8_t convert_channel_8b_scale(uint8_t num, uint8_t min, uint8_t max);
uint8_t convert_channel_s8b(uint8_t num);
uint16_t convert_channel_10b(uint8_t num);
uint16_t convert_channel_frsky(uint8_t num);
void convert_channel_HK310(uint8_t num, uint8_t *low, uint8_t *high);
uint16_t convert_channel_16b(uint8_t num, int16_t out_min, int16_t out_max);
uint16_t convert_channel_16b_nolim(uint8_t num, int16_t out_min, int16_t out_max);
uint16_t limit_channel_100(uint8_t ch);
void Frsky_init_hop(void);
void FRSKY_init_cc2500(const uint8_t *ptr);
uint16_t ReadAFHDS2A(void);
uint16_t initAFHDS2A(void);
void ASSAN_init(void);
void ASSAN_send_packet(void);
uint16_t ASSAN_callback(void);
uint16_t initASSAN(void);
uint16_t BAYANG_callback(void);
uint16_t initBAYANG(void);
uint16_t CG023_callback(void);
uint16_t initCG023(void);
uint16_t CX10_callback(void);
uint16_t initCX10(void);
uint16_t devo_callback(void);
uint16_t DevoInit(void);
uint16_t ReadDsm(void);
uint16_t initDsm(void);
uint16_t ESKY_callback(void);
uint16_t initESKY(void);
uint16_t ReadFlySky(void);
uint16_t initFlySky(void);
uint16_t FQ777_callback(void);
uint16_t initFQ777(void);
uint16_t initFrSky_2way(void);
uint16_t ReadFrSky_2way(void);
uint16_t ReadFRSKYV(void);
uint16_t initFRSKYV(void);
uint16_t ReadFrSkyX(void);
uint16_t initFrSkyX(void);
uint16_t FY326_callback(void);
uint16_t initFY326(void);
uint16_t GW008_callback(void);
uint16_t initGW008(void);
uint16_t hisky_cb(void);
uint16_t initHiSky(void);
uint16_t HONTAI_callback(void);
uint16_t initHONTAI(void);
uint16_t ReadHubsan(void);
uint16_t initHubsan(void);
uint16_t ReadJ6Pro(void);
uint16_t initJ6Pro(void);
uint16_t initKN(void);
uint16_t kn_callback(void);
uint16_t MJXQ_callback(void);
uint16_t initMJXQ(void);
uint16_t MT99XX_callback(void);
uint16_t initMT99XX(void);
uint16_t Q303_callback(void);
uint16_t initQ303(void);
uint16_t ReadSFHSS(void);
uint16_t initSFHSS(void);
void SHENQI_init(void);
void SHENQI_send_packet(void);
uint16_t SHENQI_callback(void);
uint16_t initSHENQI(void);
uint16_t SLT_callback(void);
uint16_t initSLT(void);
uint16_t symax_callback(void);
uint16_t initSymax(void);
uint16_t ReadV2x2(void);
uint16_t initV2x2(void);
uint16_t WK_cb(void);
uint16_t WK_setup(void);
uint16_t yd717_callback(void);
uint16_t initYD717(void);
void DSM_frame(void);
void AFHDSA_short_frame(void);
void frskySendStuffed(void);
void frsky_check_telemetry(uint8_t *pkt, uint8_t len);
void init_frskyd_link_telemetry(void);
void frsky_link_frame(void);
void frsky_user_frame(void);
void sportSend(uint8_t *p);
void sportIdle(void);
void sportSendFrame(void);
void proces_sport_data(uint8_t data);
void TelemetryUpdate(void);

//***************
//***  Flags  ***
//***************
#define RX_FLAG_on          protocol_flags |= _BV(0)
#define RX_FLAG_off         protocol_flags &= ~_BV(0)
#define IS_RX_FLAG_on       ( ( protocol_flags & _BV(0) ) !=0 )
//
#define CHANGE_PROTOCOL_FLAG_on     protocol_flags |= _BV(1)
#define CHANGE_PROTOCOL_FLAG_off    protocol_flags &= ~_BV(1)
#define IS_CHANGE_PROTOCOL_FLAG_on  ( ( protocol_flags & _BV(1) ) !=0 )
//
#define POWER_FLAG_on       protocol_flags |= _BV(2)
#define POWER_FLAG_off      protocol_flags &= ~_BV(2)
#define IS_POWER_FLAG_on    ( ( protocol_flags & _BV(2) ) !=0 )
//
#define RANGE_FLAG_on       protocol_flags |= _BV(3)
#define RANGE_FLAG_off      protocol_flags &= ~_BV(3)
#define IS_RANGE_FLAG_on    ( ( protocol_flags & _BV(3) ) !=0 )
//
#define AUTOBIND_FLAG_on    protocol_flags |= _BV(4)
#define AUTOBIND_FLAG_off   protocol_flags &= ~_BV(4)
#define IS_AUTOBIND_FLAG_on ( ( protocol_flags & _BV(4) ) !=0 )
//
#define BIND_BUTTON_FLAG_on     protocol_flags |= _BV(5)
#define BIND_BUTTON_FLAG_off    protocol_flags &= ~_BV(5)
#define IS_BIND_BUTTON_FLAG_on  ( ( protocol_flags & _BV(5) ) !=0 )
//PPM RX OK
#define PPM_FLAG_off        protocol_flags &= ~_BV(6)
#define PPM_FLAG_on         protocol_flags |= _BV(6)
#define IS_PPM_FLAG_on      ( ( protocol_flags & _BV(6) ) !=0 )
//Bind flag
#define BIND_IN_PROGRESS    protocol_flags &= ~_BV(7)
#define BIND_DONE           protocol_flags |= _BV(7)
#define IS_BIND_DONE_on     ( ( protocol_flags & _BV(7) ) !=0 )
//
#define BAD_PROTO_off       protocol_flags2 &= ~_BV(0)
#define BAD_PROTO_on        protocol_flags2 |= _BV(0)
#define IS_BAD_PROTO_on     ( ( protocol_flags2 & _BV(0) ) !=0 )
//
#define RX_DONOTUPDTAE_off  protocol_flags2 &= ~_BV(1)
#define RX_DONOTUPDTAE_on   protocol_flags2 |= _BV(1)
#define IS_RX_DONOTUPDTAE_on    ( ( protocol_flags2 & _BV(1) ) !=0 )
//
#define RX_MISSED_BUFF_off  protocol_flags2 &= ~_BV(2)
#define RX_MISSED_BUFF_on   protocol_flags2 |= _BV(2)
#define IS_RX_MISSED_BUFF_on    ( ( protocol_flags2 & _BV(2) ) !=0 )
//TX Pause
#define TX_MAIN_PAUSE_off   protocol_flags2 &= ~_BV(3)
#define TX_MAIN_PAUSE_on        protocol_flags2 |= _BV(3)
#define IS_TX_MAIN_PAUSE_on ( ( protocol_flags2 & _BV(3) ) !=0 )
#define TX_RX_PAUSE_off     protocol_flags2 &= ~_BV(4)
#define TX_RX_PAUSE_on      protocol_flags2 |= _BV(4)
#define IS_TX_RX_PAUSE_on   ( ( protocol_flags2 & _BV(4) ) !=0 )
#define IS_TX_PAUSE_on      ( ( protocol_flags2 & (_BV(4)|_BV(3)) ) !=0 )
//Signal OK
#define INPUT_SIGNAL_off    protocol_flags2 &= ~_BV(5)
#define INPUT_SIGNAL_on     protocol_flags2 |= _BV(5)
#define IS_INPUT_SIGNAL_on  ( ( protocol_flags2 & _BV(5) ) !=0 )
#define IS_INPUT_SIGNAL_off ( ( protocol_flags2 & _BV(5) ) ==0 )
//Bind from channel
#define BIND_CH_PREV_off    protocol_flags2 &= ~_BV(6)
#define BIND_CH_PREV_on     protocol_flags2 |= _BV(6)
#define IS_BIND_CH_PREV_on  ( ( protocol_flags2 & _BV(6) ) !=0 )
#define IS_BIND_CH_PREV_off ( ( protocol_flags2 & _BV(6) ) ==0 )
//Wait for bind
#define WAIT_BIND_off       protocol_flags2 &= ~_BV(7)
#define WAIT_BIND_on        protocol_flags2 |= _BV(7)
#define IS_WAIT_BIND_on     ( ( protocol_flags2 & _BV(7) ) !=0 )
#define IS_WAIT_BIND_off    ( ( protocol_flags2 & _BV(7) ) ==0 )

