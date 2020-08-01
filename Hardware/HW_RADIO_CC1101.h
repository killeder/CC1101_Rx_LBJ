/*-----------------------------------------------------------------------
*@file     HW_RADIO_CC1101.h
*@brief    德州仪器CC1101无线芯片驱动程序 - 头文件
*@author   谢英男(xieyingnan1994@163.com）
*@version  1.0
*@date     2020/07/14
-----------------------------------------------------------------------*/
#ifndef HW_RADIO_CC1101_H
#define HW_RADIO_CC1101_H

// CC1101 physical layer properties
#define CC1101_FIFO_SIZE                      	  	  64
#define CC1101_CRYSTAL_FREQ                           26.0
// CC1101 SPI commands
#define CC1101_CMD_READ                               0x80
#define CC1101_CMD_WRITE                              0x00
#define CC1101_CMD_BURST                              0x40
#define CC1101_CMD_ACCESS_STATUS_REG                  0x40
#define CC1101_CMD_RESET                              0x30
#define CC1101_CMD_FSTXON                             0x31
#define CC1101_CMD_XOFF                               0x32
#define CC1101_CMD_CAL                                0x33
#define CC1101_CMD_RX                                 0x34
#define CC1101_CMD_TX                                 0x35
#define CC1101_CMD_IDLE                               0x36
#define CC1101_CMD_WOR                                0x38
#define CC1101_CMD_POWER_DOWN                         0x39
#define CC1101_CMD_FLUSH_RX                           0x3A
#define CC1101_CMD_FLUSH_TX                           0x3B
#define CC1101_CMD_WOR_RESET                          0x3C
#define CC1101_CMD_NOP                                0x3D
// CC1101 register map
#define CC1101_REG_IOCFG2                             0x00
#define CC1101_REG_IOCFG1                             0x01
#define CC1101_REG_IOCFG0                             0x02
#define CC1101_REG_FIFOTHR                            0x03
#define CC1101_REG_SYNC1                              0x04
#define CC1101_REG_SYNC0                              0x05
#define CC1101_REG_PKTLEN                             0x06
#define CC1101_REG_PKTCTRL1                           0x07
#define CC1101_REG_PKTCTRL0                           0x08
#define CC1101_REG_ADDR                               0x09
#define CC1101_REG_CHANNR                             0x0A
#define CC1101_REG_FSCTRL1                            0x0B
#define CC1101_REG_FSCTRL0                            0x0C
#define CC1101_REG_FREQ2                              0x0D
#define CC1101_REG_FREQ1                              0x0E
#define CC1101_REG_FREQ0                              0x0F
#define CC1101_REG_MDMCFG4                            0x10
#define CC1101_REG_MDMCFG3                            0x11
#define CC1101_REG_MDMCFG2                            0x12
#define CC1101_REG_MDMCFG1                            0x13
#define CC1101_REG_MDMCFG0                            0x14
#define CC1101_REG_DEVIATN                            0x15
#define CC1101_REG_MCSM2                              0x16
#define CC1101_REG_MCSM1                              0x17
#define CC1101_REG_MCSM0                              0x18
#define CC1101_REG_FOCCFG                             0x19
#define CC1101_REG_BSCFG                              0x1A
#define CC1101_REG_AGCCTRL2                           0x1B
#define CC1101_REG_AGCCTRL1                           0x1C
#define CC1101_REG_AGCCTRL0                           0x1D
#define CC1101_REG_WOREVT1                            0x1E
#define CC1101_REG_WOREVT0                            0x1F
#define CC1101_REG_WORCTRL                            0x20
#define CC1101_REG_FREND1                             0x21
#define CC1101_REG_FREND0                             0x22
#define CC1101_REG_FSCAL3                             0x23
#define CC1101_REG_FSCAL2                             0x24
#define CC1101_REG_FSCAL1                             0x25
#define CC1101_REG_FSCAL0                             0x26
#define CC1101_REG_RCCTRL1                            0x27
#define CC1101_REG_RCCTRL0                            0x28
#define CC1101_REG_FSTEST                             0x29
#define CC1101_REG_PTEST                              0x2A
#define CC1101_REG_AGCTEST                            0x2B
#define CC1101_REG_TEST2                              0x2C
#define CC1101_REG_TEST1                              0x2D
#define CC1101_REG_TEST0                              0x2E
#define CC1101_REG_PARTNUM                            0x30
#define CC1101_REG_VERSION                            0x31
#define CC1101_REG_FREQEST                            0x32
#define CC1101_REG_LQI                                0x33
#define CC1101_REG_RSSI                               0x34
#define CC1101_REG_MARCSTATE                          0x35
#define CC1101_REG_WORTIME1                           0x36
#define CC1101_REG_WORTIME0                           0x37
#define CC1101_REG_PKTSTATUS                          0x38
#define CC1101_REG_VCO_VC_DAC                         0x39
#define CC1101_REG_TXBYTES                            0x3A
#define CC1101_REG_RXBYTES                            0x3B
#define CC1101_REG_RCCTRL1_STATUS                     0x3C
#define CC1101_REG_RCCTRL0_STATUS                     0x3D
#define CC1101_REG_PATABLE                            0x3E
#define CC1101_REG_FIFO                               0x3F

// CC1101_REG_IOCFG2                                                  MSB   LSB   DESCRIPTION
#define CC1101_GDO2_NORM                              0x00  //  6     6     GDO2 output: active high (default)
#define CC1101_GDO2_INV                               0x40  //  6     6                  active low

// CC1101_REG_IOCFG1
#define CC1101_GDO1_DS_LOW                            0x00  //  7     7     GDO1 output drive strength: low (default)
#define CC1101_GDO1_DS_HIGH                           0x80  //  7     7                                 high
#define CC1101_GDO1_NORM                              0x00  //  6     6     GDO1 output: active high (default)
#define CC1101_GDO1_INV                               0x40  //  6     6                  active low

// CC1101_REG_IOCFG0
#define CC1101_GDO0_TEMP_SENSOR_OFF                   0x00  //  7     7     analog temperature sensor output: disabled (default)
#define CC1101_GDO0_TEMP_SENSOR_ON                    0x80  //  7     0                                       enabled
#define CC1101_GDO0_NORM                              0x00  //  6     6     GDO0 output: active high (default)
#define CC1101_GDO0_INV                               0x40  //  6     6                  active low

// CC1101_REG_IOCFG2 + REG_IOCFG1 + REG_IOCFG0
#define CC1101_GDOX_RX_FIFO_FULL                      0x00        //  5     0     Rx FIFO full or above threshold
#define CC1101_GDOX_RX_FIFO_FULL_OR_PKT_END           0x01        //  5     0     Rx FIFO full or above threshold or reached packet end
#define CC1101_GDOX_TX_FIFO_ABOVE_THR                 0x02        //  5     0     Tx FIFO above threshold
#define CC1101_GDOX_TX_FIFO_FULL                      0x03        //  5     0     Tx FIFO full
#define CC1101_GDOX_RX_FIFO_OVERFLOW                  0x04        //  5     0     Rx FIFO overflowed
#define CC1101_GDOX_TX_FIFO_UNDERFLOW                 0x05        //  5     0     Tx FIFO underflowed
#define CC1101_GDOX_SYNC_WORD_SENT_OR_RECEIVED        0x06        //  5     0     sync word was sent or received
#define CC1101_GDOX_PKT_RECEIVED_CRC_OK               0x07        //  5     0     packet received and CRC check passed
#define CC1101_GDOX_PREAMBLE_QUALITY_REACHED          0x08        //  5     0     received preamble quality is above threshold
#define CC1101_GDOX_CHANNEL_CLEAR                     0x09        //  5     0     RSSI level below threshold (channel is clear)
#define CC1101_GDOX_PLL_LOCKED                        0x0A        //  5     0     PLL is locked
#define CC1101_GDOX_SERIAL_CLOCK                      0x0B        //  5     0     serial data clock
#define CC1101_GDOX_SERIAL_DATA_SYNC                  0x0C        //  5     0     serial data output in: synchronous mode
#define CC1101_GDOX_SERIAL_DATA_ASYNC                 0x0D        //  5     0                            asynchronous mode
#define CC1101_GDOX_CARRIER_SENSE                     0x0E        //  5     0     RSSI above threshold
#define CC1101_GDOX_CRC_OK                            0x0F        //  5     0     CRC check passed
#define CC1101_GDOX_RX_HARD_DATA1                     0x16        //  5     0     direct access to demodulated data
#define CC1101_GDOX_RX_HARD_DATA0                     0x17        //  5     0     direct access to demodulated data
#define CC1101_GDOX_PA_PD                             0x1B        //  5     0     power amplifier circuit is powered down
#define CC1101_GDOX_LNA_PD                            0x1C        //  5     0     low-noise amplifier circuit is powered down
#define CC1101_GDOX_RX_SYMBOL_TICK                    0x1D        //  5     0     direct access to symbol tick of received data
#define CC1101_GDOX_WOR_EVNT0                         0x24        //  5     0     wake-on-radio event 0
#define CC1101_GDOX_WOR_EVNT1                         0x25        //  5     0     wake-on-radio event 1
#define CC1101_GDOX_CLK_256                           0x26        //  5     0     256 Hz clock
#define CC1101_GDOX_CLK_32K                           0x27        //  5     0     32 kHz clock
#define CC1101_GDOX_CHIP_RDYN                         0x29        //  5     0      (default for GDO2)
#define CC1101_GDOX_XOSC_STABLE                       0x2B        //  5     0
#define CC1101_GDOX_HIGH_Z                            0x2E        //  5     0     high impedance state (default for GDO1)
#define CC1101_GDOX_HW_TO_0                           0x2F        //  5     0
#define CC1101_GDOX_CLOCK_XOSC_1                      0x30        //  5     0     crystal oscillator clock: f = f(XOSC)/1
#define CC1101_GDOX_CLOCK_XOSC_1_5                    0x31        //  5     0                               f = f(XOSC)/1.5
#define CC1101_GDOX_CLOCK_XOSC_2                      0x32        //  5     0                               f = f(XOSC)/2
#define CC1101_GDOX_CLOCK_XOSC_3                      0x33        //  5     0                               f = f(XOSC)/3
#define CC1101_GDOX_CLOCK_XOSC_4                      0x34        //  5     0                               f = f(XOSC)/4
#define CC1101_GDOX_CLOCK_XOSC_6                      0x35        //  5     0                               f = f(XOSC)/6
#define CC1101_GDOX_CLOCK_XOSC_8                      0x36        //  5     0                               f = f(XOSC)/8
#define CC1101_GDOX_CLOCK_XOSC_12                     0x37        //  5     0                               f = f(XOSC)/12
#define CC1101_GDOX_CLOCK_XOSC_16                     0x38        //  5     0                               f = f(XOSC)/16
#define CC1101_GDOX_CLOCK_XOSC_24                     0x39        //  5     0                               f = f(XOSC)/24
#define CC1101_GDOX_CLOCK_XOSC_32                     0x3A        //  5     0                               f = f(XOSC)/32
#define CC1101_GDOX_CLOCK_XOSC_48                     0x3B        //  5     0                               f = f(XOSC)/48
#define CC1101_GDOX_CLOCK_XOSC_64                     0x3C        //  5     0                               f = f(XOSC)/64
#define CC1101_GDOX_CLOCK_XOSC_96                     0x3D        //  5     0                               f = f(XOSC)/96
#define CC1101_GDOX_CLOCK_XOSC_128                    0x3E        //  5     0                               f = f(XOSC)/128
#define CC1101_GDOX_CLOCK_XOSC_192                    0x3F        //  5     0                               f = f(XOSC)/192 (default for GDO0)

// CC1101_REG_FIFOTHR
#define CC1101_ADC_RETENTION_OFF                      0x00  //  6     6     do not retain ADC settings in sleep mode (default)
#define CC1101_ADC_RETENTION_ON                       0x40  //  6     6     retain ADC settings in sleep mode
#define CC1101_RX_ATTEN_0_DB                          0x00  //  5     4     Rx attenuation: 0 dB (default)
#define CC1101_RX_ATTEN_6_DB                          0x10 	//  5     4                     6 dB
#define CC1101_RX_ATTEN_12_DB                         0x20  //  5     4                     12 dB
#define CC1101_RX_ATTEN_18_DB                         0x30  //  5     4                     18 dB
#define CC1101_FIFO_THR                               0x07  //  5     4     Rx FIFO threshold [bytes] = CC1101_FIFO_THR * 4; Tx FIFO threshold [bytes] = 65 - (CC1101_FIFO_THR * 4)

// CC1101_REG_SYNC1
#define CC1101_SYNC_WORD_MSB                          0xD3        //  7     0     sync word MSB

// CC1101_REG_SYNC0
#define CC1101_SYNC_WORD_LSB                          0x91        //  7     0     sync word LSB

// CC1101_REG_PKTLEN
#define CC1101_PACKET_LENGTH                          0xFF        //  7     0     packet length in bytes

// CC1101_REG_PKTCTRL1
#define CC1101_PQT                                    0x00  //  7     5     preamble quality threshold
#define CC1101_CRC_AUTOFLUSH_OFF                      0x00  //  3     3     automatic Rx FIFO flush on CRC check fail: disabled (default)
#define CC1101_CRC_AUTOFLUSH_ON                       0x08  //  3     3                                                enabled
#define CC1101_APPEND_STATUS_OFF                      0x00  //  2     2     append 2 status bytes to packet: disabled
#define CC1101_APPEND_STATUS_ON                       0x04  //  2     2                                      enabled (default)
#define CC1101_ADR_CHK_NONE                           0x00  //  1     0     address check: none (default)
#define CC1101_ADR_CHK_NO_BROADCAST                   0x01  //  1     0                    without broadcast
#define CC1101_ADR_CHK_SINGLE_BROADCAST               0x02  //  1     0                    broadcast address 0x00
#define CC1101_ADR_CHK_DOUBLE_BROADCAST               0x03  //  1     0                    broadcast addresses 0x00 and 0xFF
// CC1101_REG_PKTCTRL0
#define CC1101_WHITE_DATA_OFF                         0x00  //  6     6     data whitening: disabled
#define CC1101_WHITE_DATA_ON                          0x40  //  6     6                     enabled (default)
#define CC1101_PKT_FORMAT_NORMAL                      0x00  //  5     4     packet format: normal (FIFOs)
#define CC1101_PKT_FORMAT_SYNCHRONOUS                 0x10 //  5     4                    synchronous serial
#define CC1101_PKT_FORMAT_RANDOM                      0x20  //  5     4                    random transmissions
#define CC1101_PKT_FORMAT_ASYNCHRONOUS                0x30  //  5     4                    asynchronous serial
#define CC1101_CRC_OFF                                0x00  //  2     2     CRC disabled
#define CC1101_CRC_ON                                 0x04  //  2     2     CRC enabled (default)
#define CC1101_LENGTH_CONFIG_FIXED                    0x00  //  1     0     packet length: fixed
#define CC1101_LENGTH_CONFIG_VARIABLE                 0x01  //  1     0                    variable (default)
#define CC1101_LENGTH_CONFIG_INFINITE                 0x02  //  1     0                    infinite

// CC1101_REG_ADDR
#define CC1101_DEVICE_ADDR                            0x00        //  7     0     device address

// CC1101_REG_CHANNR
#define CC1101_CHAN                                   0x00        //  7     0     channel number

// CC1101_REG_FSCTRL1
#define CC1101_FREQ_IF                                0x0F        //  4     0     IF frequency setting; f_IF = (f(XOSC) / 2^10) * CC1101_FREQ_IF

// CC1101_REG_FSCTRL0
#define CC1101_FREQOFF                                0x00        //  7     0     base frequency offset (2s-compliment)

// CC1101_REG_FREQ2 + REG_FREQ1 + REG_FREQ0
#define CC1101_FREQ_MSB                               0x1E        //  5     0     base frequency setting: f_carrier = (f(XOSC) / 2^16) * FREQ
#define CC1101_FREQ_MID                               0xC4        //  7     0         where f(XOSC) = 26 MHz
#define CC1101_FREQ_LSB                               0xEC        //  7     0               FREQ = 3-byte value of FREQ registers

// CC1101_REG_MDMCFG4
#define CC1101_CHANBW_E                               0x80  //  7     6     channel bandwidth: BW_channel = f(XOSC) / (8 * (4 + CHANBW_M)*2^CHANBW_E) [Hz]
#define CC1101_CHANBW_M                               0x00  //  5     4         default value for 26 MHz crystal: 203 125 Hz
#define CC1101_DRATE_E                                0x0C  //  3     0     symbol rate: R_data = (((256 + DRATE_M) * 2^DRATE_E) / 2^28) * f(XOSC) [Baud]

// CC1101_REG_MDMCFG3
#define CC1101_DRATE_M                                0x22  //  7     0         default value for 26 MHz crystal: 115 051 Baud

// CC1101_REG_MDMCFG2
#define CC1101_DEM_DCFILT_OFF                         0x80  //  7     7     digital DC filter: disabled
#define CC1101_DEM_DCFILT_ON                          0x00  //  7     7                        enabled - only for data rates above 250 kBaud (default)
#define CC1101_MOD_FORMAT_2_FSK                       0x00  //  6     4     modulation format: 2-FSK (default)
#define CC1101_MOD_FORMAT_GFSK                        0x10  //  6     4                        GFSK
#define CC1101_MOD_FORMAT_ASK_OOK                     0x30  //  6     4                        ASK/OOK
#define CC1101_MOD_FORMAT_4_FSK                       0x40  //  6     4                        4-FSK
#define CC1101_MOD_FORMAT_MFSK                        0x70  //  6     4                        MFSK - only for data rates above 26 kBaud
#define CC1101_MANCHESTER_EN_OFF                      0x00  //  3     3     Manchester encoding: disabled (default)
#define CC1101_MANCHESTER_EN_ON                       0x08  //  3     3                          enabled
#define CC1101_SYNC_MODE_NONE                         0x00  //  2     0     synchronization: no preamble/sync
#define CC1101_SYNC_MODE_15_16                        0x01  //  2     0                      15/16 sync word bits
#define CC1101_SYNC_MODE_16_16                        0x02  //  2     0                      16/16 sync word bits (default)
#define CC1101_SYNC_MODE_30_32                        0x03  //  2     0                      30/32 sync word bits
#define CC1101_SYNC_MODE_NONE_THR                     0x04  //  2     0                      no preamble sync, carrier sense above threshold
#define CC1101_SYNC_MODE_15_16_THR                    0x05  //  2     0                      15/16 sync word bits, carrier sense above threshold
#define CC1101_SYNC_MODE_16_16_THR                    0x06  //  2     0                      16/16 sync word bits, carrier sense above threshold
#define CC1101_SYNC_MODE_30_32_THR                    0x07  //  2     0                      30/32 sync word bits, carrier sense above threshold

// CC1101_REG_MDMCFG1
#define CC1101_FEC_OFF                                0x00  //  7     7     forward error correction: disabled (default)
#define CC1101_FEC_ON                                 0x80  //  7     7                               enabled - only for fixed packet length
#define CC1101_NUM_PREAMBLE_2                         0x00  //  6     4     number of preamble bytes: 2
#define CC1101_NUM_PREAMBLE_3                         0x10  //  6     4                               3
#define CC1101_NUM_PREAMBLE_4                         0x20  //  6     4                               4 (default)
#define CC1101_NUM_PREAMBLE_6                         0x30  //  6     4                               6
#define CC1101_NUM_PREAMBLE_8                         0x40  //  6     4                               8
#define CC1101_NUM_PREAMBLE_12                        0x50  //  6     4                               12
#define CC1101_NUM_PREAMBLE_16                        0x60  //  6     4                               16
#define CC1101_NUM_PREAMBLE_24                        0x70  //  6     4                               24
#define CC1101_CHANSPC_E                              0x02  //  1     0     channel spacing: df_channel = (f(XOSC) / 2^18) * (256 + CHANSPC_M) * 2^CHANSPC_E [Hz]

// CC1101_REG_MDMCFG0
#define CC1101_CHANSPC_M                              0xF8  //  7     0         default value for 26 MHz crystal: 199 951 kHz

// CC1101_REG_DEVIATN
#define CC1101_DEVIATION_E                            0x40  //  6     4     frequency deviation: f_dev = (f(XOSC) / 2^17) * (8 + DEVIATION_M) * 2^DEVIATION_E [Hz]
#define CC1101_DEVIATION_M                            0x07  //  2     0     default value for 26 MHz crystal: +- 47 607 Hz
#define CC1101_MSK_PHASE_CHANGE_PERIOD                0x07  //  2     0     phase change symbol period fraction: 1 / (MSK_PHASE_CHANGE_PERIOD + 1)

// CC1101_REG_MCSM2
#define CC1101_RX_TIMEOUT_RSSI_OFF                    0x00  //  4     4     Rx timeout based on RSSI value: disabled (default)
#define CC1101_RX_TIMEOUT_RSSI_ON                     0x10  //  4     4                                     enabled
#define CC1101_RX_TIMEOUT_QUAL_OFF                    0x00  //  3     3     check for sync word on Rx timeout
#define CC1101_RX_TIMEOUT_QUAL_ON                     0x08  //  3     3     check for PQI set on Rx timeout
#define CC1101_RX_TIMEOUT_OFF                         0x07  //  2     0     Rx timeout: disabled (default)
#define CC1101_RX_TIMEOUT_MAX                         0x00  //  2     0                 max value (actual value depends on WOR_RES, EVENT0 and f(XOSC))

// CC1101_REG_MCSM1
#define CC1101_CCA_MODE_ALWAYS                        0x00  //  5     4     clear channel indication: always
#define CC1101_CCA_MODE_RSSI_THR                      0x10  //  5     4                               RSSI below threshold
#define CC1101_CCA_MODE_RX_PKT                        0x20  //  5     4                               unless receiving packet
#define CC1101_CCA_MODE_RSSI_THR_RX_PKT               0x30  //  5     4                               RSSI below threshold unless receiving packet (default)
#define CC1101_RXOFF_IDLE                             0x00  //  3     2     next mode after packet reception: idle (default)
#define CC1101_RXOFF_FSTXON                           0x04  //  3     2                                       FSTxOn
#define CC1101_RXOFF_TX                               0x08  //  3     2                                       Tx
#define CC1101_RXOFF_RX                               0x0C  //  3     2                                       Rx
#define CC1101_TXOFF_IDLE                             0x00  //  1     0     next mode after packet transmission: idle (default)
#define CC1101_TXOFF_FSTXON                           0x01  //  1     0                                          FSTxOn
#define CC1101_TXOFF_TX                               0x02  //  1     0                                          Tx
#define CC1101_TXOFF_RX                               0x03  //  1     0                                          Rx

// CC1101_REG_MCSM0
#define CC1101_FS_AUTOCAL_NEVER                       0x00  //  5     4     automatic calibration: never (default)
#define CC1101_FS_AUTOCAL_IDLE_TO_RXTX                0x10  //  5     4                            every transition from idle to Rx/Tx
#define CC1101_FS_AUTOCAL_RXTX_TO_IDLE                0x20  //  5     4                            every transition from Rx/Tx to idle
#define CC1101_FS_AUTOCAL_RXTX_TO_IDLE_4TH            0x30  //  5     4                            every 4th transition from Rx/Tx to idle
#define CC1101_PO_TIMEOUT_COUNT_1                     0x00  //  3     2     number of counter expirations before CHP_RDYN goes low: 1 (default)
#define CC1101_PO_TIMEOUT_COUNT_16                    0x04  //  3     2                                                             16
#define CC1101_PO_TIMEOUT_COUNT_64                    0x08  //  3     2                                                             64
#define CC1101_PO_TIMEOUT_COUNT_256                   0x0C  //  3     2                                                             256
#define CC1101_PIN_CTRL_OFF                           0x00  //  1     1     pin radio control: disabled (default)
#define CC1101_PIN_CTRL_ON                            0x02  //  1     1                        enabled
#define CC1101_XOSC_FORCE_OFF                         0x00  //  0     0     do not force XOSC to remain on in sleep (default)
#define CC1101_XOSC_FORCE_ON                          0x01  //  0     0     force XOSC to remain on in sleep

// CC1101_REG_FOCCFG
#define CC1101_FOC_BS_CS_GATE_OFF                     0x00  //  5     5     do not freeze frequency compensation until CS goes high
#define CC1101_FOC_BS_CS_GATE_ON                      0x20  //  5     5     freeze frequency compensation until CS goes high (default)
#define CC1101_FOC_PRE_K                              0x00  //  4     3     frequency compensation loop gain before sync word: K
#define CC1101_FOC_PRE_2K                             0x08  //  4     3                                                        2K
#define CC1101_FOC_PRE_3K                             0x10  //  4     3                                                        3K (default)
#define CC1101_FOC_PRE_4K                             0x18  //  4     3                                                        4K
#define CC1101_FOC_POST_K                             0x00  //  2     2     frequency compensation loop gain after sync word: same as FOC_PRE
#define CC1101_FOC_POST_K_2                           0x04  //  2     2                                                       K/2 (default)
#define CC1101_FOC_LIMIT_NO_COMPENSATION              0x00  //  1     0     frequency compensation saturation point: no compensation - required for ASK/OOK
#define CC1101_FOC_LIMIT_BW_CHAN_8                    0x01  //  1     0                                              +- BW_chan/8
#define CC1101_FOC_LIMIT_BW_CHAN_4                    0x02  //  1     0                                              +- BW_chan/4 (default)
#define CC1101_FOC_LIMIT_BW_CHAN_2                    0x03  //  1     0                                              +- BW_chan/2

// CC1101_REG_BSCFG
#define CC1101_BS_PRE_KI                              0x00  //  7     6     clock recovery integral gain before sync word: Ki
#define CC1101_BS_PRE_2KI                             0x40  //  7     6                                                    2Ki (default)
#define CC1101_BS_PRE_3KI                             0x80  //  7     6                                                    3Ki
#define CC1101_BS_PRE_4KI                             0xC0  //  7     6                                                    4Ki
#define CC1101_BS_PRE_KP                              0x00  //  5     4     clock recovery proportional gain before sync word: Kp
#define CC1101_BS_PRE_2KP                             0x10  //  5     4                                                        2Kp
#define CC1101_BS_PRE_3KP                             0x20  //  5     4                                                        3Kp (default)
#define CC1101_BS_PRE_4KP                             0x30  //  5     4                                                        4Kp
#define CC1101_BS_POST_KI                             0x00  //  3     3     clock recovery integral gain after sync word: same as BS_PRE
#define CC1101_BS_POST_KI_2                           0x08  //  3     3                                                   Ki/2 (default)
#define CC1101_BS_POST_KP                             0x00  //  2     2     clock recovery proportional gain after sync word: same as BS_PRE
#define CC1101_BS_POST_KP_1                           0x04  //  2     2                                                       Kp (default)
#define CC1101_BS_LIMIT_NO_COMPENSATION               0x00  //  1     0     data rate compensation saturation point: no compensation
#define CC1101_BS_LIMIT_3_125                         0x01  //  1     0                                              +- 3.125 %
#define CC1101_BS_LIMIT_6_25                          0x02  //  1     0                                              +- 6.25 %
#define CC1101_BS_LIMIT_12_5                          0x03  //  1     0                                              +- 12.5 %

// CC1101_REG_AGCCTRL2
#define CC1101_MAX_DVGA_GAIN_0                        0x00  //  7     6     reduce maximum available DVGA gain: no reduction (default)
#define CC1101_MAX_DVGA_GAIN_1                        0x40  //  7     6                                         disable top gain setting
#define CC1101_MAX_DVGA_GAIN_2                        0x80  //  7     6                                         disable top two gain setting
#define CC1101_MAX_DVGA_GAIN_3                        0xC0  //  7     6                                         disable top three gain setting
#define CC1101_LNA_GAIN_REDUCE_0_DB                   0x00  //  5     3     reduce maximum LNA gain by: 0 dB (default)
#define CC1101_LNA_GAIN_REDUCE_2_6_DB                 0x08  //  5     3                                 2.6 dB
#define CC1101_LNA_GAIN_REDUCE_6_1_DB                 0x10  //  5     3                                 6.1 dB
#define CC1101_LNA_GAIN_REDUCE_7_4_DB                 0x18  //  5     3                                 7.4 dB
#define CC1101_LNA_GAIN_REDUCE_9_2_DB                 0x20  //  5     3                                 9.2 dB
#define CC1101_LNA_GAIN_REDUCE_11_5_DB                0x28  //  5     3                                 11.5 dB
#define CC1101_LNA_GAIN_REDUCE_14_6_DB                0x30  //  5     3                                 14.6 dB
#define CC1101_LNA_GAIN_REDUCE_17_1_DB                0x38  //  5     3                                 17.1 dB
#define CC1101_MAGN_TARGET_24_DB                      0x00  //  2     0     average amplitude target for filter: 24 dB
#define CC1101_MAGN_TARGET_27_DB                      0x01  //  2     0                                          27 dB
#define CC1101_MAGN_TARGET_30_DB                      0x02  //  2     0                                          30 dB
#define CC1101_MAGN_TARGET_33_DB                      0x03  //  2     0                                          33 dB (default)
#define CC1101_MAGN_TARGET_36_DB                      0x04  //  2     0                                          36 dB
#define CC1101_MAGN_TARGET_38_DB                      0x05  //  2     0                                          38 dB
#define CC1101_MAGN_TARGET_40_DB                      0x06  //  2     0                                          40 dB
#define CC1101_MAGN_TARGET_42_DB                      0x07  //  2     0                                          42 dB

// CC1101_REG_AGCCTRL1
#define CC1101_AGC_LNA_PRIORITY_LNA2                  0x00  //  6     6     LNA priority setting: LNA2 first
#define CC1101_AGC_LNA_PRIORITY_LNA                   0x40  //  6     6                           LNA first (default)
#define CC1101_CARRIER_SENSE_REL_THR_OFF              0x00  //  5     4     RSSI relative change to assert carrier sense: disabled (default)
#define CC1101_CARRIER_SENSE_REL_THR_6_DB             0x10  //  5     4                                                   6 dB
#define CC1101_CARRIER_SENSE_REL_THR_10_DB            0x20  //  5     4                                                   10 dB
#define CC1101_CARRIER_SENSE_REL_THR_14_DB            0x30  //  5     4                                                   14 dB
#define CC1101_CARRIER_SENSE_ABS_THR                  0x00  //  3     0     RSSI threshold to assert carrier sense in 2s compliment, Thr = MAGN_TARGET + CARRIER_SENSE_ABS_TH [dB]

// CC1101_REG_AGCCTRL0
#define CC1101_HYST_LEVEL_NONE                        0x00  //  7     6     AGC hysteresis level: none
#define CC1101_HYST_LEVEL_LOW                         0x40  //  7     6                           low
#define CC1101_HYST_LEVEL_MEDIUM                      0x80  //  7     6                           medium (default)
#define CC1101_HYST_LEVEL_HIGH                        0xC0  //  7     6                           high
#define CC1101_WAIT_TIME_8_SAMPLES                    0x00  //  5     4     AGC wait time: 8 samples
#define CC1101_WAIT_TIME_16_SAMPLES                   0x10  //  5     4                    16 samples (default)
#define CC1101_WAIT_TIME_24_SAMPLES                   0x20  //  5     4                    24 samples
#define CC1101_WAIT_TIME_32_SAMPLES                   0x30  //  5     4                    32 samples
#define CC1101_AGC_FREEZE_NEVER                       0x00  //  3     2     freeze AGC gain: never (default)
#define CC1101_AGC_FREEZE_SYNC_WORD                   0x04  //  3     2                      when sync word is found
#define CC1101_AGC_FREEZE_MANUAL_A                    0x08  //  3     2                      manually freeze analog control
#define CC1101_AGC_FREEZE_MANUAL_AD                   0x0C  //  3     2                      manually freeze analog and digital control
#define CC1101_FILTER_LENGTH_8                        0x00  //  1     0     averaging length for channel filter: 8 samples
#define CC1101_FILTER_LENGTH_16                       0x01  //  1     0                                          16 samples (default)
#define CC1101_FILTER_LENGTH_32                       0x02  //  1     0                                          32 samples
#define CC1101_FILTER_LENGTH_64                       0x03  //  1     0                                          64 samples
#define CC1101_ASK_OOK_BOUNDARY_4_DB                  0x00  //  1     0     ASK/OOK decision boundary: 4 dB
#define CC1101_ASK_OOK_BOUNDARY_8_DB                  0x01  //  1     0                                8 dB (default)
#define CC1101_ASK_OOK_BOUNDARY_12_DB                 0x02  //  1     0                                12 dB
#define CC1101_ASK_OOK_BOUNDARY_16_DB                 0x03  //  1     0                                16 dB

// CC1101_REG_WOREVT1 + REG_WOREVT0
#define CC1101_EVENT0_TIMEOUT_MSB                     0x87        //  7     0     EVENT0 timeout: t_event0 = (750 / f(XOSC)) * EVENT0_TIMEOUT * 2^(5 * WOR_RES) [s]
#define CC1101_EVENT0_TIMEOUT_LSB                     0x6B        //  7     0         default value for 26 MHz crystal: 1.0 s

// CC1101_REG_WORCTRL
#define CC1101_RC_POWER_UP                            0x00  //  7     7     power up RC oscillator
#define CC1101_RC_POWER_DOWN                          0x80  //  7     7     power down RC oscillator
#define CC1101_EVENT1_TIMEOUT_4                       0x00  //  6     4     EVENT1 timeout: 4 RC periods
#define CC1101_EVENT1_TIMEOUT_6                       0x10 //  6     4                     6 RC periods
#define CC1101_EVENT1_TIMEOUT_8                       0x20  //  6     4                     8 RC periods
#define CC1101_EVENT1_TIMEOUT_12                      0x30  //  6     4                     12 RC periods
#define CC1101_EVENT1_TIMEOUT_16                      0x40  //  6     4                     16 RC periods
#define CC1101_EVENT1_TIMEOUT_24                      0x50  //  6     4                     24 RC periods
#define CC1101_EVENT1_TIMEOUT_32                      0x60  //  6     4                     32 RC periods
#define CC1101_EVENT1_TIMEOUT_48                      0x70  //  6     4                     48 RC periods (default)
#define CC1101_RC_CAL_OFF                             0x00  //  3     3     disable RC oscillator calibration
#define CC1101_RC_CAL_ON                              0x08  //  3     3     enable RC oscillator calibration (default)
#define CC1101_WOR_RES_1                              0x00  //  1     0     EVENT0 resolution: 1 period (default)
#define CC1101_WOR_RES_2_5                            0x01  //  1     0                        2^5 periods
#define CC1101_WOR_RES_2_10                           0x02  //  1     0                        2^10 periods
#define CC1101_WOR_RES_2_15                           0x03  //  1     0                        2^15 periods

// CC1101_REG_FREND1
#define CC1101_LNA_CURRENT                            0x01        //  7     6     front-end LNA PTAT current output adjustment
#define CC1101_LNA2MIX_CURRENT                        0x01        //  5     4     front-end PTAT output adjustment
#define CC1101_LODIV_BUF_CURRENT_RX                   0x01        //  3     2     Rx LO buffer current adjustment
#define CC1101_MIX_CURRENT                            0x02        //  1     0     mixer current adjustment

// CC1101_REG_FREND0
#define CC1101_LODIV_BUF_CURRENT_TX                   0x01        //  5     4     Tx LO buffer current adjustment
#define CC1101_PA_POWER                               0x00        //  2     0     set power amplifier power according to PATABLE

// CC1101_REG_FSCAL3
#define CC1101_CHP_CURR_CAL_OFF                       0x00  	//  5     4     disable charge pump calibration
#define CC1101_CHP_CURR_CAL_ON                        0x20  	//  5     4     enable charge pump calibration (default)
#define CC1101_FSCAL3                                 0x09      //  3     0     charge pump output current: I_out = I_0 * 2^(FSCAL3/4) [A]

// CC1101_REG_FSCAL2
#define CC1101_VCO_CORE_LOW                           0x00  	//  5     5     VCO: low (default)
#define CC1101_VCO_CORE_HIGH                          0x20  	//  5     5          high
#define CC1101_FSCAL2                                 0x0A      //  4     0     VCO current result/override

// CC1101_REG_FSCAL1
#define CC1101_FSCAL1                                 0x20        //  5     0     capacitor array setting for coarse VCO tuning

// CC1101_REG_FSCAL0
#define CC1101_FSCAL0                                 0x0D        //  6     0     frequency synthesizer calibration setting

// CC1101_REG_RCCTRL1
#define CC1101_RCCTRL1                                0x41        //  6     0     RC oscillator configuration

// CC1101_REG_RCCTRL0
#define CC1101_RCCTRL0                                0x00        //  6     0     RC oscillator configuration

// CC1101_REG_PTEST
#define CC1101_TEMP_SENS_IDLE_OFF                     0x7F        //  7     0     temperature sensor will not be available in idle mode (default)
#define CC1101_TEMP_SENS_IDLE_ON                      0xBF        //  7     0     temperature sensor will be available in idle mode

// CC1101_REG_TEST0
#define CC1101_VCO_SEL_CAL_OFF                        0x00  //  1     1     disable VCO selection calibration stage
#define CC1101_VCO_SEL_CAL_ON                         0x02  //  1     1     enable VCO selection calibration stage

// CC1101_REG_PARTNUM
#define CC1101_PARTNUM                                0x00

// CC1101_REG_VERSION
#define CC1101_VERSION                                0x14

// CC1101_REG_MARCSTATE
#define CC1101_MARC_STATE_SLEEP                       0x00        //  4     0     main radio control state: sleep
#define CC1101_MARC_STATE_IDLE                        0x01        //  4     0                               idle
#define CC1101_MARC_STATE_XOFF                        0x02        //  4     0                               XOFF
#define CC1101_MARC_STATE_VCOON_MC                    0x03        //  4     0                               VCOON_MC
#define CC1101_MARC_STATE_REGON_MC                    0x04        //  4     0                               REGON_MC
#define CC1101_MARC_STATE_MANCAL                      0x05        //  4     0                               MANCAL
#define CC1101_MARC_STATE_VCOON                       0x06        //  4     0                               VCOON
#define CC1101_MARC_STATE_REGON                       0x07        //  4     0                               REGON
#define CC1101_MARC_STATE_STARTCAL                    0x08        //  4     0                               STARTCAL
#define CC1101_MARC_STATE_BWBOOST                     0x09        //  4     0                               BWBOOST
#define CC1101_MARC_STATE_FS_LOCK                     0x0A        //  4     0                               FS_LOCK
#define CC1101_MARC_STATE_IFADCON                     0x0B        //  4     0                               IFADCON
#define CC1101_MARC_STATE_ENDCAL                      0x0C        //  4     0                               ENDCAL
#define CC1101_MARC_STATE_RX                          0x0D        //  4     0                               RX
#define CC1101_MARC_STATE_RX_END                      0x0E        //  4     0                               RX_END
#define CC1101_MARC_STATE_RX_RST                      0x0F        //  4     0                               RX_RST
#define CC1101_MARC_STATE_TXRX_SWITCH                 0x10        //  4     0                               TXRX_SWITCH
#define CC1101_MARC_STATE_RXFIFO_OVERFLOW             0x11        //  4     0                               RXFIFO_OVERFLOW
#define CC1101_MARC_STATE_FSTXON                      0x12        //  4     0                               FSTXON
#define CC1101_MARC_STATE_TX                          0x13        //  4     0                               TX
#define CC1101_MARC_STATE_TX_END                      0x14        //  4     0                               TX_END
#define CC1101_MARC_STATE_RXTX_SWITCH                 0x15        //  4     0                               RXTX_SWITCH
#define CC1101_MARC_STATE_TXFIFO_UNDERFLOW            0x16        //  4     0                               TXFIFO_UNDERFLOW

// CC1101_REG_WORTIME1 + REG_WORTIME0
#define CC1101_WORTIME_MSB                            0x00        //  7     0     WOR timer value
#define CC1101_WORTIME_LSB                            0x00        //  7     0

// CC1101_REG_PKTSTATUS
#define CC1101_CRC_OK                                 0x80  //  7     7     CRC check passed
#define CC1101_CRC_RADIO_ERROR                              0x00  //  7     7     CRC check failed
#define CC1101_CS                                     0x40  //  6     6     carrier sense
#define CC1101_PQT_REACHED                            0x20  //  5     5     preamble quality reached
#define CC1101_CCA                                    0x10  //  4     4     channel clear
#define CC1101_SFD                                    0x08  //  3     3     start of frame delimiter - sync word received
#define CC1101_GDO2_ACTIVE                            0x04  //  2     2     GDO2 is active/asserted
#define CC1101_GDO0_ACTIVE                            0x01  //  0     0     GDO0 is active/asserted

// common status codes

/*!
  \brief No error, method executed successfully.
*/
#define RADIO_ERR_NONE                                      0
/*!
  \brief There was an unexpected, unknown error. If you see this, something went incredibly wrong.
  Your STM32 may be possessed, contact your local exorcist to resolve this error.
*/
#define RADIO_ERR_UNKNOWN                                   -1
/*!
  \brief Radio chip was not found during initialization. This can be caused by specifying wrong chip type in the constructor
  or by a fault in your wiring (incorrect slave select pin).
*/
#define RADIO_ERR_CHIP_NOT_FOUND                            -2

/*!
  \brief Failed to allocate memory for temporary buffer. This can be cause by not enough RAM or by passing invalid pointer.
*/
#define RADIO_ERR_MEMORY_ALLOCATION_FAILED                  -3

/*!
  \brief Packet supplied to transmission method was longer than limit.
*/
#define RADIO_ERR_PACKET_TOO_LONG                           -4

/*!
  \brief Timed out waiting for transmission finish.
*/
#define RADIO_ERR_TX_TIMEOUT                                -5

/*!
  \brief Timed out waiting for incoming transmission.
*/
#define RADIO_ERR_RX_TIMEOUT                                -6

/*!
  \brief The calculated and expected CRCs of received packet do not match.
  This means that the packet was damaged during transmission and should be sent again.
*/
#define RADIO_ERR_CRC_MISMATCH                              -7

/*!
  \brief The supplied bandwidth value is invalid for this module.
*/
#define RADIO_ERR_INVALID_BANDWIDTH                         -8

/*!
  \brief The supplied spreading factor value is invalid for this module.
*/
#define RADIO_ERR_INVALID_SPREADING_FACTOR                  -9

/*!
  \brief The supplied coding rate value is invalid for this module.
*/
#define RADIO_ERR_INVALID_CODING_RATE                       -10

/*!
  \brief Internal only.
*/
#define RADIO_ERR_INVALID_BIT_RANGE                         -11

/*!
  \brief The supplied frequency value is invalid for this module.
*/
#define RADIO_ERR_INVALID_FREQUENCY                         -12

/*!
  \brief The supplied output power value is invalid for this module.
*/
#define RADIO_ERR_INVALID_OUTPUT_POWER                      -13

/*!
  \brief LoRa preamble was detected during channel activity detection.
  This means that there is some LoRa device currently transmitting in your channel.
*/
#define RADIO_PREAMBLE_DETECTED                             -14

/*!
  \brief No LoRa preambles were detected during channel activity detection. Your channel is free.
*/
#define RADIO_CHANNEL_FREE                                  -15

/*!
  \brief Real value in SPI register does not match the expected one. This can be caused by faulty SPI wiring.
*/
#define RADIO_ERR_SPI_WRITE_FAILED                          -16

/*!
  \brief The supplied current limit value is invalid.
*/
#define RADIO_ERR_INVALID_CURRENT_LIMIT                     -17

/*!
  \brief The supplied preamble length is invalid.
*/
#define RADIO_ERR_INVALID_PREAMBLE_LENGTH                   -18

/*!
  \brief The supplied gain value is invalid.
*/
#define RADIO_ERR_INVALID_GAIN                              -19

/*!
  \brief User tried to execute modem-exclusive method on a wrong modem.
  For example, this can happen when you try to change LoRa configuration when FSK modem is active.
*/
#define RADIO_ERR_WRONG_MODEM                               -20

/*!
  \brief The supplied number of RSSI samples is invalid.
*/
#define RADIO_ERR_INVALID_NUM_SAMPLES                       -21

/*!
  \brief The supplied RSSI offset is invalid.
*/
#define RADIO_ERR_INVALID_RSSI_OFFSET                       -22

/*!
  \brief The supplied encoding is invalid.
*/
#define RADIO_ERR_INVALID_ENCODING                          -23

/*!
  \brief Supplied number of broadcast addresses is invalid.
*/
#define RADIO_ERR_INVALID_NUM_BROAD_ADDRS                   -24
/*!
  \brief Data shaping of output spectrum is invalid.
*/
#define RADIO_ERR_INVALID_DATA_SHAPING						-25
/*!
  \brief The format of sync words is invalid. 
*/
#define RADIO_ERR_INVALID_SYNC_WORD							-26
/*!
  \brief The biterate setting is invalid. 
*/
#define RADIO_ERR_INVALID_BIT_RATE							-27
/*!
  \brief The receive bandwidth setting is invalid. 
*/
#define RADIO_ERR_INVALID_RX_BANDWIDTH						-28
/*!
  \brief The frequency deviation is invalid. 
*/
#define RADIO_ERR_INVALID_FREQUENCY_DEVIATION				-29
/*!
  \brief A simple assert macro, will return on error.
*/
#define RADIO_ASSERT(STATEVAR) { if((STATEVAR) != RADIO_ERR_NONE) { return(STATEVAR); } }

/*!
  \brief Macro to check variable is within constraints - this is commonly used to check parameter ranges.
*/
#define RADIO_CHECK_RANGE(VAR, MIN, MAX, ERR) { if(!(((VAR) >= (MIN)) && ((VAR) <= (MAX)))) { return(ERR); } }

#ifdef RADIO_DEBUG_MSG_ON
	#define RADIO_DEBUG_MSG(...) printf(__VA_ARGS__)
#else
	#define RADIO_DEBUG_MSG(...)
#endif

#ifdef RADIO_DEBUG_VERBOSE
	#define RADIO_VERBOSE_MSG(...) printf(__VA_ARGS__)
#else
	#define RADIO_VERBOSE_MSG(...)
#endif

int8_t CC1101_GoIdle(void);	//使CC1101进入待命IDLE模式
int8_t CC1101_SetPacketLengthMode(uint8_t mode, uint8_t len);	//数据包长度模式配置
int8_t CC1101_SetEncoding(uint8_t encoding);//设置编码模式：曼彻斯特编码、数据白化功能是否启用
int8_t CC1101_SetDataShaping(float sh);//设置调制方式：2FSK或GFSK调制方式
int8_t CC1101_SetCrcFiltering(bool crcOn);//设置CRC校验功能开启/关闭
int8_t CC1101_DisableSyncWordFiltering(bool requireCarrierSense);//关闭同步字过滤功能并决定是保留载波检测功能
int8_t CC1101_EnableSyncWordFiltering(uint8_t maxErrBits, bool requireCarrierSense);
											//启用同步字过滤功能并决定是启用载波检测功能
int8_t CC1101_DisableAddressFiltering(void);//取消地址过滤功能
int8_t CC1101_EnableAddressFiltering(uint8_t nodeAddr, uint8_t numBroadcastAddrs);//启用地址过滤，
											//同时设置本机地址和广播地址的数量
uint32_t CC1101_GetPacketLength(bool update);//读取包长度
uint8_t CC1101_GetLQI(void);//取得链接质量指数
float CC1101_GetRSSI(void);//取得RSSI(接收信号强度指示)
int8_t CC1101_SetPreambleLength(uint8_t preambleLength);//设置前导码长度
int8_t CC1101_SetSyncWord(uint8_t SyncH, uint8_t SyncL, uint8_t maxErrBits,
							bool requireCarrierSense);//设置同步字及启用同步字过滤功能
int8_t CC1101_SetOutputPower(int8_t power);//设置射频发射功率
int8_t CC1101_SetFrequency(float freq);	//设置射频工作频率
int8_t CC1101_SetBitRate(float br);//设置码率,单位kbps
int8_t CC1101_SetRxBandwidth(float rxBw);//设置接收机滤波器带宽,单位kHz
int8_t CC1101_SetFrequencyDeviation(float freqDev);//设置FSK频偏,单位kHz

int8_t CC1101_ReadDataFIFO(uint8_t* data, uint32_t* actual_len);//从FIFO读取接收到的包数据
void CC1101_StartReceive(void (*RxCallback)(void));//进入接收模式，准备接收数据包
int8_t CC1101_TransmitWithAddress(uint8_t* data, uint32_t len,
								uint8_t addr,void (*TxCallback)(void));//发送数据包，带地址
int8_t CC1101_Transmit(uint8_t* data, uint32_t len, void (*TxCallback)(void));
													//发送数据包，无地址
int8_t CC1101_Setup(float freq, float br, float freqDev, float rxBw,
					 int8_t power, uint8_t preambleLength);//CC1101初始化配置

#endif
