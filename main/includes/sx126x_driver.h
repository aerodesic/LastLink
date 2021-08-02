/*
 * sx126x_driver.h
 *
 */
#ifndef __sx126x_driver_h_included
#define __sx126x_driver_h_included

#include "stdbool.h"
#include "radio.h"

/* Command definitions */

#define SX126x_SetSleep                                  0x84
#define SX126x_SetSleep_bytes                              1
#define SX126x_SetSleep_Param                                0
#define SX126x_SetSleep_Param_bytes                            1
#define SX126x_SetSleep_Rtc                                      0x01
#define SX126x_SetSleep_WarmStart                                0x04

#define SX126x_SetStandby                                0x80
#define SX126x_SetStandby_bytes                            1
#define SX126x_SetStandby_Param                              0
#define SX126x_SetStandby_Param_bytes                          1
#define SX126x_SetStandby_RC                                     0x00
#define SX126x_SetStandby_XOSC                                   0x01

#define SX126x_SetFs                                     0xC1
#define SX126x_SetFs_bytes                                 0

#define SX126x_SetTx                                     0x83
#define SX126x_SetTx_bytes                                 3
#define SX126x_SetTx_Timeout                                 0
#define SX126x_SetTx_Timeout_bytes                             3

#define SX126x_SetRx                                     0x82
#define SX126x_SetRx_bytes                                 3
#define SX126x_SetRx_Timeout                                 0
#define SX126x_SetRx_Timeout_bytes                             3

#define SX126x_StopTimerOnPreamble                       0x9F
#define SX126x_StopTimerOnPreamble_bytes                   1
#define SX126x_StopTimerOnPreamble_disable                   0x00
#define SX126x_StopTimerOnPreamble_enable                    0x01

#define SX126x_SetRxDutyCycle                            0x94
#define SX126x_SetRxDutyCycle_bytes                        6
#define SX126x_SetRxDutyCycle_rxPeriod                       0
#define SX126x_SetRxDutyCycle_rxPeriod_bytes                   3
#define SX126x_SetRxDutyCycle_sleepPeriod                    3
#define SX126x_SetRxDutyCycle_sleepPeriod_bytes                3

#define SX126x_SetCad                                    0xC5
#define SX126x_SetCad_bytes                                0

#define SX126x_SetTxContinuousWave                       0xD1
#define SX126x_SetTxContinuousWave_bytes                   0

#define SX126x_SetTxInfinitePreamble                     0xD2
#define SX126x_SetTxInfinitePreamble_bytes                 0

#define SX126x_SetRegulatorMode                          0x96
#define SX126x_SetRegulatorMode_bytes                      1
#define SX126x_SetRegulatorMode_LDO                        0x00
#define SX126x_SetRegulaterMode_DC_LDO                     0x01

#define SX126x_Calibrate                                 0x89
#define SX126x_Calibrate_bytes                             1
#define SX126x_Calibrate_Param                               0
#define SX126x_Calibrate_Param_bytes                           1
#define SX126x_Calibrate_RC64k                                   0x01
#define SX126x_Calibrate_RC13M                                   0x02
#define SX126x_Calibrate_PLL                                     0x04
#define SX126x_Calibrate_ADC_pulse                               0x08
#define SX126x_Calibrate_ADC_bulkN                               0x10
#define SX126x_Calibrate_ADC_bulkP                               0x20
#define SX126x_Calibrate_Image                                   0x40
#define SX126x_Calibrate_RFU                                     0x80
#define SX126x_Calibrate_All                                     0x7F

#define SX126x_CalibrateImage                            0x98
#define SX126x_CalibrateImage_bytes                       2
#define SX126x_CalibrateImage_Freq1                         0
#define SX126x_CalibrateImage_Freq1_bytes                     1
#define SX126x_CalibrateImage_Freq2                         1
#define SX126x_CalibrateImage_Freq2_bytes                     1

#define SX126x_SetPaConfig                               0x95
#define SX126x_SetPaConfig_bytes                          4
#define SX126x_SetPaConfig_paDutyCycle                      0
#define SX126x_SetPaConfig_paDutyCycle_bytes                  1
#define SX126x_SetPaConfig_hpMax                            1
#define SX126x_SetPaConfig_hpMax_bytes                        1
#define SX126x_SetPaConfig_deviceSel                        2
#define SX126x_SetPaConfig_deviceSel_bytes                    1
#define SX126x_SetPaConfig_deviceSel_SX1261                     1
#define SX126x_SetPaConfig_deviceSel_SX1262                     0
#define SX126x_SetPaConfig_paLut                            3
#define SX126x_SetPaConfig_paLut_bytes                        1

#define SX126x_SetRxTxFallbackMode                       0x93
#define SX126x_SetRxTxFallbackMode_bytes                   1
#define SX126x_SetRxTxFallbackMode_Param                     0
#define SX126x_SetRxTxFallbackMode_Param_bytes                 1
#define SX126x_SetRxTxFallbackMode_Param_FS                      0x40
#define SX126x_SetRxTxFallbackMode_Param_STDBY_XOSC              0x30
#define SX126x_SetRxTxFallbackMode_Param_STDBY_RC                0x20

#define SX126x_WriteRegister                             0x0D

#define SX126x_ReadRegister                              0x1D

#define SX126x_WriteBuffer                               0x0E

#define SX126x_ReadBuffer                                0x1E

#define SX126x_REG_DIOx_OutputEnable                     0x0580
#define SX126x_REG_DIOx_InputEnable                      0x0583
#define SX126x_REG_DIOx_PullupControl                    0x0584
#define SX126x_REG_DIOxPulldownControl                   0x0585
#define SX126x_REG_Whitening_Initial_Value_MSB           0x06B8
#define SX126x_REG_Whitening_Initial_Value_LSB           0x06B9
#define SX126x_REG_CRC_MSB_InitialValue0                 0x06BC
#define SX126x_REG_CRC_LSB_InitialValue1                 0x06BD
#define SX126x_REG_CRC_MSB_polynomial_value0             0x06BE
#define SX126x_REG_CRC_LSB_polynomial_value1             0x06BF
#define SX126x_REG_SyncWord0                             0x06C0 // (FSK Only)
#define SX126x_REG_SyncWord1                             0x06C1 // (FSK Only)
#define SX126x_REG_SyncWord2                             0x06C2 // (FSK Only)
#define SX126x_REG_SyncWord3                             0x06C3 // (FSK Only)
#define SX126x_REG_SyncWord4                             0x06C4 // (FSK Only)
#define SX126x_REG_SyncWord5                             0x06C5 // (FSK Only)
#define SX126x_REG_SyncWord6                             0x06C6 // (FSK Only)
#define SX126x_REG_SyncWord7                             0x06C7 // (FSK Only)
#define SX126x_REG_NodeAddress                           0x06CD // (FSK Only)
#define SX126x_REG_Broadcast_Address                     0x06CE // (FSK Only)
#define SX126x_REG_IQ_Polarity_Setup                     0x0736
#define SX126x_REG_LoRa_Sync_Word_MSB                    0x0740 // (Set to 0x3444 for public network and 0x1424 for private network)
#define SX126x_REG_LoRa_Sync_Word_LSB                    0x0741
#define SX126x_REG_LoRa_Sync_Word_PUBLIC                   0x3444
#define SX126x_REG_LoRa_Sync_Word_PRIVATE                  0x1424
#define SX126x_REG_RandomNumberGen0                      0x0819
#define SX126x_REG_RandomNumberGen1                      0x081A
#define SX126x_REG_RandomNumberGen2                      0x081B
#define SX126x_REG_RandomNumberGen3                      0x081C
#define SX126x_REG_TxModulation                          0x0889
#define SX126x_REG_Rx_Gain                               0x08AC
#define SX126x_REG_TxClampConfig                         0x08D8  /* From app note */
#define SX126x_REG_OCP_Configuration                     0x08E7
#define SX126x_REG_OCP_Configuration_60mA                  0x18
#define SX126x_REG_OCP_Configuration_140mA                 0x38
#define SX126x_REG_RTC_Control                           0x0902
#define SX126x_REG_XTA_trim                              0x0911
#define SX126x_REG_XTB_trim                              0x0912
#define SX126x_REG_DIO3_output_voltage_control           0x0920
#define SX126x_REG_Event_Mask                            0x0944


#define SX126x_SetDioIrqParams                           0x08
#define SX126x_SetDioIrqParams_bytes                     8
#define SX126x_SetDioIrqParams_IrqMask                     0
#define SX126x_SetDioIrqParams_IrqMask_bytes                 2
#define SX126x_SetDioIrqParams_DIO0                        2
#define SX126x_SetDioIrqParams_DIO0_bytes                    2
#define SX126x_SetDioIrqParams_DIO1                        4
#define SX126x_SetDioIrqParams_DIO1_bytes                    2
#define SX126x_SetDioIrqParams_DIO2                        6
#define SX126x_SetDioIrqParams_DIO2_bytes                    2

#define SX126x_IrqFlags_TxDone                           0x0001   /* Tx packet complete */
#define SX126x_IrqFlags_RxDone                           0x0002   /* Rx packet complete */
#define SX126x_IrqFlags_PreambleDetected                 0x0004   /* A packet preamble has been detected */
#define SX126x_IrqFlags_SyncWordValid                    0x0008   /* The preamble sync word is valid */
#define SX126x_IrqFlags_HeaderValid                      0x0010   /* Complete header check valid */
#define SX126x_IrqFlags_HeaderErr                        0x0020   /* An error in the header */
#define SX126x_IrqFlags_CrcErr                           0x0040   /* Crc error at message end */
#define SX126x_IrqFlags_CadDone                          0x0080   /* Carrier Detect phase complete */
#define SX126x_IrqFlags_CadDetected                      0x0100   /* Carrier detected during CAD phase */
#define SX126x_IrqFlags_Timeout                          0x0200   /* Rx or Tx timeout */

#define SX126x_GetIrqStatus                              0x12
#define SX126x_GetIrqStatus_bytes                        3
#define SX126x_GetIrqStatus_Status                         0
#define SX126x_GetIrqStatus_Status_bytes                     1
#define SX126x_GetIrqStatus_Status_Chipmode                    0x70
#define SX126x_GetIrqStatus_Status_CommandStatus               0x0E
#define SX126x_GetIrqStatus_IrqFlags                       1
#define SX126x_GetIrqStatus_IrqFlags_bytes                   2

#define SX126x_ClearIrqStatus                            0x02
#define SX126x_ClearIrqStatus_bytes                        2
#define SX126x_ClearIrqStatus_IrqFlags                       0
#define SX126x_ClearIrqStatus_IrqFlags_bytes                   2

#define SX126x_SetDio2AsRfSwitchCtrl                     0x9D
#define SX126x_SetDio2AsRfSwitchCtrl_bytes                 1
#define SX126x_SetDio2AsRfSwitchCtrl_enable                  0
#define SX126x_SetDio2AsRfSwitchCtrl_enable_bytes              1
#define SX126x_SetDio2AsRfSwitchCtrl_enabled                     0x01
#define SX126x_SetDio2AsRfSwitchCtrl_disabled                    0x00



#define SX126x_SetDio3AsTcxoCtrl                         0x97
#define SX126x_SetDio3AsTcxoCtrl_bytes                     4
#define SX126x_SetDio3AsTcxoCtrl_tcxoVoltage                 0
#define SX126x_SetDio3AsTcxoCtrl_tcxoVoltage_bytes             1
#define SX126x_SetDio3AsTcxoCtrl_tcxoVoltage_1_6v                0x00
#define SX126x_SetDio3AsTcxoCtrl_tcxoVoltage_1_7v                0x01
#define SX126x_SetDio3AsTcxoCtrl_tcxoVoltage_1_8v                0x02
#define SX126x_SetDio3AsTcxoCtrl_tcxoVoltage_2_2v                0x03
#define SX126x_SetDio3AsTcxoCtrl_tcxoVoltage_2_4v                0x04
#define SX126x_SetDio3AsTcxoCtrl_tcxoVoltage_2_7v                0x05
#define SX126x_SetDio3AsTcxoCtrl_tcxoVoltage_3_0v                0x06
#define SX126x_SetDio3AsTcxoCtrl_tcxoVoltage_3_3v                0x07
#define SX126x_SetDio3AsTcxoCtrl_delay                      1
#define SX126x_SetDio3AsTcxoCtrl_delay_bytes                  3

#define SX126x_SetRfFrequency                            0x86
#define SX126x_SetRfFrequency_bytes                        4
#define SX126x_SetRfFrequency_FREQ                           0
#define SX126x_SetRfFrequency_FREQ_bytes                       4

#define SX126x_SetPacketType                             0x8A
#define SX126x_SetPacketType_bytes                         1
#define SX126x_SetPacketType_PacketType                      0
#define SX126x_SetPacketType_PacketType_bytes                  1
#define SX126x_SetPacketType_PacketType_GFSK                     0x00
#define SX126x_SetPacketType_PacketType_LORA                     0x01

#define SX126x_GetPacketType                             0x11
#define SX126x_GetPacketType_bytes                         2
#define SX126x_GetPacketType_Status                          0
#define SX126x_GetPacketType_Status_bytes                      1
#define SX126x_GetPacketType_Status_Chipmode                     0x70
#define SX126x_GetPacketType_Status_CommandStatus                0x0E
#define SX126x_GetPacketType_PacketType                      1
#define SX126x_GetPacketType_PacketType_bytes                  1

#define SX126x_SetTxParams                               0x8E
#define SX126x_SetTxParams_bytes                         2
#define SX126x_SetTxParams_TxPower                         0
#define SX126x_SetTxParams_TxPower_bytes                     1 
#define SX126x_SetTxParams_TxPower_LowMin                      -17
#define SX126x_SetTxParams_TxPower_LowMax                      14
#define SX126x_SetTxParams_TxPower_LowStep                     1
#define SX126x_SetTxParams_TxPower_HighMin                     -9
#define SX126x_SetTxParams_TxPower_HighMax                     22
#define SX126x_SetTxParams_TxPower_HighStep                    1
#define SX126x_SetTxParams_TxRamp                          1
#define SX126x_SetTxParams_TxRamp_bytes                      1
#define SX126x_SetTxParams_TxRamp_10uS                         0      /* 10 uS   */
#define SX126x_SetTxParams_TxRamp_20uS                         1      /* 20 uS   */
#define SX126x_SetTxParams_TxRamp_40uS                         2      /* 40 uS   */
#define SX126x_SetTxParams_TxRamp_80uS                         3      /* 80 uS   */
#define SX126x_SetTxParams_TxRamp_200uS                        4      /* 200 uS  */
#define SX126x_SetTxParams_TxRamp_800uS                        5      /* 800 uS  */
#define SX126x_SetTxParams_TxRamp_1700uS                       6      /* 1700 uS */
#define SX126x_SetTxParams_TxRamp_3400uS                       7      /* 3400 uS */

#define SX126x_SetModulationParams                       0x8B
#define SX126x_SetModulationParams_bytes                   8
#define SX126x_SetModulationParams_SF                        0
#define SX126x_SetModulationParams_SF_bytes                    1
#define SX126x_SetModulationParams_BW                        1
#define SX126x_SetModulationParams_BW_bytes                    1
#define SX126x_SetModulationParams_BW7800                        0x00  /* Funny ... */
#define SX126x_SetModulationParams_BW10400                       0x08  /*  these codes */
#define SX126x_SetModulationParams_BW15600                       0x01  /*  are not the same */
#define SX126x_SetModulationParams_BW20800                       0x09  /*  as the */
#define SX126x_SetModulationParams_BW31250                       0x02  /*  external */
#define SX126x_SetModulationParams_BW41700                       0x0A  /*  BW codes... */
#define SX126x_SetModulationParams_BW62500                       0x03
#define SX126x_SetModulationParams_BW125000                      0x04
#define SX126x_SetModulationParams_BW250000                      0x05
#define SX126x_SetModulationParams_BW500000                      0x06
#define SX126x_SetModulationParams_CR                        2
#define SX126x_SetModulationParams_CR_bytes                    1
#define SX126x_SetModulationParams_CR5                           0x01
#define SX126x_SetModulationParams_CR6                           0x02
#define SX126x_SetModulationParams_CR7                           0x03
#define SX126x_SetModulationParams_CR8                           0x04
#define SX126x_SetModulationParams_LdOpt                     3
#define SX126x_SetModulationParams_LdOpt_bytes                 1
#define SX126x_SetModulationParams_LdOpt_OFF                     0
#define SX126x_SetModulationParams_LdOpt_ON                      1

#define SX126x_SetPacketParams                           0x8C
#define SX126x_SetPacketParams_bytes                       9
#define SX126x_SetPacketParams_PreambleLength                 0
#define SX126x_SetPacketParams_PreambleLength_bytes             2
#define SX126x_SetPacketParams_HeaderType                     2
#define SX126x_SetPacketParams_HeaderType_bytes                 1
#define SX126x_SetPacketParams_HeaderType_Variable                0
#define SX126x_SetPacketParams_HeaderType_Fixed                   1
#define SX126x_SetPacketParams_PayloadLength                  3
#define SX126x_SetPacketParams_PayloadLength_bytes              1
#define SX126x_SetPacketParams_CrcType                        4
#define SX126x_SetPacketParams_CrcType_bytes                    1
#define SX126x_SetPacketParams_CrcType_Off                        0
#define SX126x_SetPacketParams_CrcType_On                         1
#define SX126x_SetPacketParams_InvertIQ                       5
#define SX126x_SetPacketParams_InvertIQ_bytes                   1
#define SX126x_SetPacketParams_InvertIQ_Off                       0
#define SX126x_SetPacketParams_InvertIQ_On                        1

#define SX126x_SetCadParams                              0x88
#define SX126x_SetCadParams_bytes                          7
#define SX126x_SetCadParams_cadSymbolNum                     0
#define SX126x_SetCadParams_cadSymbolNum_bytes                 1
#define SX126x_SetCadPraams_cadSymbolNum_CAD_ON_1_SYMB           0x00
#define SX126x_SetCadPraams_cadSymbolNum_CAD_ON_2_SYMB           0x01
#define SX126x_SetCadPraams_cadSymbolNum_CAD_ON_4_SYMB           0x02
#define SX126x_SetCadPraams_cadSymbolNum_CAD_ON_8_SYMB           0x03
#define SX126x_SetCadPraams_cadSymbolNum_CAD_ON_16_SYMB          0x04
#define SX126x_SetCadParams_cadDetPeak                       1
#define SX126x_SetCadParams_cadDetPeak_bytes                   1
#define SX126x_SetCadParams_cadDetMin                        2
#define SX126x_SetCadParams_cadDetMin_bytes                    1
#define SX126x_SetCadParams_cadExitMode                      4
#define SX126x_SetCadParams_cadExitMode_bytes                  1
#define SX126x_SetCadParams_cadExitMode_CAD_ONLY                 0x00
#define SX126x_SetCadParams_cadExitMode_CAD_RX                   0x01
#define SX126x_SetCadParams_cadTimeout                       5
#define SX126x_SetCadParams_cadTimeout_bytes                   3

#define SX126x_SetBufferBaseAddress                      0x8F
#define SX126x_SetBufferBaseAddress_bytes                  2
#define SX126x_SetBufferBaseAddress_TX                       0
#define SX126x_SetBufferBaseAddress_TX_bytes                   1
#define SX126x_SetBufferBaseAddress_RX                       1 
#define SX126x_SetBufferBaseAddress_RX_bytes                   1

#define SX126x_SetLoRaSymbNumTimeout                     0xA0
#define SX126x_SetLoRaSymbNumTimeout_bytes                 1
#define SX126x_SetLoRaSymbNumTimeout_SymbNum                 0
#define SX126x_SetLoRaSymbNumTimeout_SymbNum_bytes             1

#define SX126x_GetStatus                                 0xC0
#define SX126x_GetStatus_bytes                             1
#define SX126x_GetStatus_Status                              0
#define SX126x_GetStatus_Status_bytes                          1
#define SX126x_GetStatus_Status_Chipmode                         0x70
#define SX126x_GetStatus_Status_Chipmode_unused                    0
#define SX126x_GetStatus_Status_Chipmode_RFU                       1
#define SX126x_GetStatus_Status_Chipmode_STBY_RC                   2
#define SX126x_GetStatus_Status_Chipmode_STBY_XOSC                 3
#define SX126x_GetStatus_Status_Chipmode_FS                        4
#define SX126x_GetStatus_Status_Chipmode_RX                        5
#define SX126x_GetStatus_Status_Chipmode_TX                        6
#define SX126x_GetStatus_Status_CommandStatus                    0x0E
#define SX126x_GetStatus_Status_CommandStatus_reserved             0
#define SX126x_GetStatus_Status_CommandStatus_RFU                  1
#define SX126x_GetStatus_Status_CommandStatus_DataAvailable        2
#define SX126x_GetStatus_Status_CommandStatus_CommandTimeout       3
#define SX126x_GetStatus_Status_CommandStatus_CommandError         4
#define SX126x_GetStatus_Status_CommandStatus_CommandFailed        5
#define SX126x_GetStatus_Status_CommandStatus_TXDone               6

#define SX126x_GetRssiInst                               0x15

#define SX126x_GetRxBufferStatus                         0x13
#define SX126x_GetRxBufferStatus_bytes                     3
#define SX126x_GetRxBufferStatus_Status                      0
#define SX126x_GetRxBufferStatus_Status_bytes                  1
#define SX126x_GetRxBufferStatus_Status_Chipmode                 0x70
#define SX126x_GetRxBufferStatus_Status_CommandStatus            0x0E
#define SX126x_GetRxBufferStatus_PayloadLengthRx             1
#define SX126x_GetRxBufferStatus_PayloadLengthRx_bytes         1
#define SX126x_GetRxBufferStatus_RxStartBufferPointer        2
#define SX126x_GetRxBufferStatus_RxStartBufferPointer_bytes    1

#define SX126x_GetPacketStatus                           0x14
#define SX126x_GetPacketStatus_bytes                       4
#define SX126x_GetPacketStatus_Status                        0
#define SX126x_GetPacketStatus_Status_bytes                    1
#define SX126x_GetPacketStatus_Status_PacketSent                 0x01
#define SX126x_GetPacketStatus_Status_PacketReceived             0x02
#define SX126x_GetPacketStatus_Status_AbortError                 0x04
#define SX126x_GetPacketStatus_Status_LengthError                0x08
#define SX126x_GetPacketStatus_Status_CrcError                   0x10
#define SX126x_GetPacketStatus_Status_AddressError               0x20
#define SX126x_GetPacketStatus_Status_SyncError                  0x40
#define SX126x_GetPacketStatus_Status_PreambleError              0x80
#define SX126x_GetPacketStatus_RssiPkt                       1
#define SX126x_GetPacketStatus_RssiPkt_bytes                   1
#define SX126x_GetPacketStatus_SnrPkt                        2
#define SX126x_GetPacketStatus_SnrPkt_bytes                    1
#define SX126x_GetPacketStatus_SignalRssiPkt                 3
#define SX126x_GetPacketStatus_SignalRssiPkt_bytes             1

#define SX126x_GetDeviceErrors                           0x17
#define SX126x_GetDeviceErrors_bytes                       3
#define SX126x_GetDeviceErrors_Status                        0
#define SX126x_GetDeviceErrors_Status_bytes                  1
#define SX126x_GetDeviceErrors_Status_Chipmode                 0x70
#define SX126x_GetDeviceErrors_Status_CommandStatus            0x0E
#define SX126x_GetDeviceErrors_OpErrors                      1
#define SX126x_GetDeviceErrors_OpErrors_bytes                  2
#define SX126x_GetDeviceErrors_OpErrors_RC64K_CALIB_ERR          0x001 // RC64k calibration failed
#define SX126x_GetDeviceErrors_OpErrors_RC13M_CALIB_ERR          0x002 // RC13M calibration failed
#define SX126x_GetDeviceErrors_OpErrors_PLL_CALIB_ERR            0x004 // PLL calibration failed
#define SX126x_GetDeviceErrors_OpErrors_ADC_CALIB_ERR            0x008 // ADC calibration failed
#define SX126x_GetDeviceErrors_OpErrors_IMG_CALIB_ERR            0x010 // IMG calibration failed
#define SX126x_GetDeviceErrors_OpErrors_XOSC_START_ERR           0x020 // XOSC failed to start
#define SX126x_GetDeviceErrors_OpErrors_PLL_LOCK_ERR             0x040 // PLL failed to lock
#define SX126x_GetDeviceErrors_OpErrors_RFU                      0x080 // RFU
#define SX126x_GetDeviceErrors_OpErrors_PA_RAMP_ERR              0x100 // PA ramping failed

#define SX126x_ClearDeviceErrors                         0x07
#define SX126x_ClearDeviceErrors_bytes                     2

#define SX126x_GetStats                                  0x10

#define SX126x_ResetStats                                0x00



bool sx126x_create(radio_t* radio);

#endif /* __sx126x_driver_h_included */
