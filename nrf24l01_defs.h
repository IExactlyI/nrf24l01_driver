/**
  ******************************************************************************
  * @file           : nrf24l01_defs.h
  * @date           : 11 August 2017
  * @brief          : This header file contain all defined parameter and value
  *                   that used in NRF24L01 driver.
  * @author         : Burin Sapsiri <burin.coding@gmail.com>
  *
  ******************************************************************************
*/

#ifndef _NRF24L01_DEFS_H
#define _NRF24L01_DEFS_H

#define NRF24L01_MAX_PAYLOAD                        32

/**
 * @brief Command
 */
#define NRF24L01_CMD_R_REGISTER                     0x00
#define NRF24L01_CMD_W_REGISTER                     0x20
#define NRF24L01_CMD_R_RX_PAYLOAD                   0x61
#define NRF24L01_CMD_W_TX_PAYLOAD                   0xA0
#define NRF24L01_CMD_FLUSH_TX                       0xE1
#define NRF24L01_CMD_FLUSH_RX                       0xE2
#define NRF24L01_CMD_REUSE_TX_PL                    0xE3
#define NRF24L01_CMD_ACTIVATE                       0x50
#define NRF24L01_CMD_R_RX_PL_WID                    0x60
#define NRF24L01_CMD_W_ACK_PAYLOAD                  0xA8
#define NRF24L01_CMD_W_TX_PAYLOAD_NO_ACK            0xB0
#define NRF24L01_CMD_NOP                            0xFF

/**
 * @brief Command mask
 */
#define NRF24L01_CMD_R_REGISTER_ADDR_MASK           0x1F
#define NRF24L01_CMD_W_REGISTER_ADDR_MASK           0x1F
#define NRF24L01_CMD_W_ACK_PAYLOAD_PIPE_MASK        0x07


/**
 * @brief Register
 */
#define NRF24L01_REG_CONFIG                         0x00
#define NRF24L01_REG_EN_AA                          0x01
#define NRF24L01_REG_EN_RXADDR                      0x02
#define NRF24L01_REG_SETUP_AW                       0x03
#define NRF24L01_REG_SETUP_RETR                     0x04
#define NRF24L01_REG_RF_CH                          0x05
#define NRF24L01_REG_RF_SETUP                       0x06
#define NRF24L01_REG_STATUS                         0x07
#define NRF24L01_REG_OBSERVE_TX                     0x08
#define NRF24L01_REG_CD                             0x09
#define NRF24L01_REG_RX_ADDR_P0                     0x0A
#define NRF24L01_REG_RX_ADDR_P1                     0x0B
#define NRF24L01_REG_RX_ADDR_P2                     0x0C
#define NRF24L01_REG_RX_ADDR_P3                     0x0D
#define NRF24L01_REG_RX_ADDR_P4                     0x0E
#define NRF24L01_REG_RX_ADDR_P5                     0x0F
#define NRF24L01_REG_TX_ADDR                        0x10
#define NRF24L01_REG_RX_PW_P0                       0x11
#define NRF24L01_REG_RX_PW_P1                       0x12
#define NRF24L01_REG_RX_PW_P2                       0x13
#define NRF24L01_REG_RX_PW_P3                       0x14
#define NRF24L01_REG_RX_PW_P4                       0x15
#define NRF24L01_REG_RX_PW_P5                       0x16
#define NRF24L01_REG_FIFO_STATUS                    0x17
#define NRF24L01_REG_DYNPD                          0x1C
#define NRF24L01_REG_FEATURE                        0x1D


/**
 * @brief Register mask
 */
#define NRF24L01_REG_STATUS_RX_DR_MASK              0x40
#define NRF24L01_REG_STATUS_TX_DS_MASK              0x20
#define NRF24L01_REG_STATUS_MAX_RT_MASK             0x10
#define NRF24L01_REG_STATUS_RX_P_NO_MASK            0x0E
#define NRF24L01_REG_STATUS_TX_FULL_MASK            0x01

#define NRF24L01_REG_OBSERVE_TX_PLOS_CNT_MASK       0xF0
#define NRF24L01_REG_OBSERVE_TX_ARC_CNT_MASK        0x0F


/**
 * @brief Define value
 */
#define NRF24L01_DEFVAL_CONFIG_RX_DR                0x40
#define NRF24L01_DEFVAL_CONFIG_TX_DS                0x20
#define NRF24L01_DEFVAL_CONFIG_MAX_RT               0x10
#define NRF24L01_DEFVAL_CONFIG_EN_CRC               0x08
#define NRF24L01_DEFVAL_CONFIG_CRC0                 0x04
#define NRF24L01_DEFVAL_CONFIG_PWR_UP               0x02
#define NRF24L01_DEFVAL_CONFIG_PRIM_RX_PRX          0x01
#define NRF24L01_DEFVAL_CONFIG_PRIM_RX_PTX          0x00

#define NRF24L01_DEFVAL_REG_RF_SETUP_RF_DR_1MBPS    0x00
#define NRF24L01_DEFVAL_REG_RF_SETUP_RF_DR_2MBPS    0x08

#define NRF24L01_DEFVAL_REG_RF_SETUP_RF_PWR_M18DBM  0x00
#define NRF24L01_DEFVAL_REG_RF_SETUP_RF_PWR_M12DBM  0x02
#define NRF24L01_DEFVAL_REG_RF_SETUP_RF_PWR_M6DBM   0x04
#define NRF24L01_DEFVAL_REG_RF_SETUP_RF_PWR_0DBM    0x06

#define NRF24L01_DEFVAL_REG_RF_SETUP_LNA_GAIN_OFF   0x00
#define NRF24L01_DEFVAL_REG_RF_SETUP_LNA_GAIN_ON    0x01

#define NRF24L01_DEFVAL_REG_STATUS_RX_P_NO_PIPE0    0x00
#define NRF24L01_DEFVAL_REG_STATUS_RX_P_NO_PIPE1    0x02
#define NRF24L01_DEFVAL_REG_STATUS_RX_P_NO_PIPE2    0x04
#define NRF24L01_DEFVAL_REG_STATUS_RX_P_NO_PIPE3    0x06
#define NRF24L01_DEFVAL_REG_STATUS_RX_P_NO_PIPE4    0x08
#define NRF24L01_DEFVAL_REG_STATUS_RX_P_NO_PIPE5    0x0A
#define NRF24L01_DEFVAL_REG_STATUS_RX_P_NO_EMPTY    0x0E



#define NRF24L01_DEFVAL_REG_CONFIG_RESET            0x08
#define NRF24L01_DEFVAL_REG_EN_AA_RESET             0x3F
#define NRF24L01_DEFVAL_REG_EN_RXADDR_RESET         0x03
#define NRF24L01_DEFVAL_REG_SETUP_AW_RESET          0x03
#define NRF24L01_DEFVAL_REG_SETUP_RETR_RESET        0x03
#define NRF24L01_DEFVAL_REG_RF_CH_RESET             0x02
#define NRF24L01_DEFVAL_REG_RF_SETUP_RESET          0x0F
#define NRF24L01_DEFVAL_REG_RX_ADDR_P0_0_RESET      0xE7
#define NRF24L01_DEFVAL_REG_RX_ADDR_P0_1_RESET      0xE7
#define NRF24L01_DEFVAL_REG_RX_ADDR_P0_2_RESET      0xE7
#define NRF24L01_DEFVAL_REG_RX_ADDR_P0_3_RESET      0xE7
#define NRF24L01_DEFVAL_REG_RX_ADDR_P0_4_RESET      0xE7
#define NRF24L01_DEFVAL_REG_RX_ADDR_P1_0_RESET      0xC2
#define NRF24L01_DEFVAL_REG_RX_ADDR_P1_1_RESET      0xC2
#define NRF24L01_DEFVAL_REG_RX_ADDR_P1_2_RESET      0xC2
#define NRF24L01_DEFVAL_REG_RX_ADDR_P1_3_RESET      0xC2
#define NRF24L01_DEFVAL_REG_RX_ADDR_P1_4_RESET      0xC2
#define NRF24L01_DEFVAL_REG_RX_ADDR_P2_RESET        0xC3
#define NRF24L01_DEFVAL_REG_RX_ADDR_P3_RESET        0xC4
#define NRF24L01_DEFVAL_REG_RX_ADDR_P4_RESET        0xC5
#define NRF24L01_DEFVAL_REG_RX_ADDR_P5_RESET        0xC6
#define NRF24L01_DEFVAL_REG_TX_ADDR_0_RESET         0xE7
#define NRF24L01_DEFVAL_REG_TX_ADDR_1_RESET         0xE7
#define NRF24L01_DEFVAL_REG_TX_ADDR_2_RESET         0xE7
#define NRF24L01_DEFVAL_REG_TX_ADDR_3_RESET         0xE7
#define NRF24L01_DEFVAL_REG_TX_ADDR_4_RESET         0xE7
#define NRF24L01_DEFVAL_REG_RX_PW_P0_RESET          0x00
#define NRF24L01_DEFVAL_REG_RX_PW_P1_RESET          0x00
#define NRF24L01_DEFVAL_REG_RX_PW_P2_RESET          0x00
#define NRF24L01_DEFVAL_REG_RX_PW_P3_RESET          0x00
#define NRF24L01_DEFVAL_REG_RX_PW_P4_RESET          0x00
#define NRF24L01_DEFVAL_REG_RX_PW_P5_RESET          0x00
#define NRF24L01_DEFVAL_REG_DYNPD_RESET             0x00
#define NRF24L01_DEFVAL_REG_FEATURE_RESET           0x00


#endif /* _NRF24L01_DEFS_H */
