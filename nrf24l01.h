/**
 ******************************************************************************
 * @file           : nrf24l01.h
 * @date           : 11 August 2017
 * @brief          : This is header file for nrf24l01.c file.
 * @author         : Burin Sapsiri <burin.coding@gmail.com>
 *
 ******************************************************************************
 * @attention
 * Copyright (C) 2017  Burin Sapsiri <burin.coding@gmail.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 3 of the License.
 *
 ******************************************************************************
 */

#ifndef _NRF24L01_H
#define _NRF24L01_H

#include <stdint.h>

struct nrf24l01_t_;

typedef enum {
    NRF24L01_FALSE = 0,
    NRF24L01_TRUE = !NRF24L01_FALSE
}nrf24l01_bool_t;

typedef enum {
    NRF24L01_FAIL = 0,
    NRF24L01_SUCCESS = 1,
}nrf24l01_result_t;

typedef enum {
    NRF24L01_MODE_IDLE = 0,
    NRF24L01_MODE_RX,
    NRF24L01_MODE_TX,
}nrf24l01_mode_t;

typedef enum {
    NRF24L01_RATE_1MBPS = 0,
    NRF24L01_RATE_2MBPS,
}nrf24l01_rate_t;

typedef enum {
    NRF24L01_TX_PWR_M18_DBM = 0,
    NRF24L01_TX_PWR_M12_DBM,
    NRF24L01_TX_PWR_M6_DBM,
    NRF24L01_TX_PWR_0_DBM
}nrf24l01_tx_pwr_t;

typedef enum {
    NRF24L01_LNA_GAIN_OFF = 0,
    NRF24L01_LNA_GAIN_ON
}nrf24l01_lna_gain_t;

typedef enum {
    NRF24L01_PRIM_TX = 0,
    NRF24L01_PRIM_RX
}nrf24l01_prim_t;

typedef enum {
    NRF24L01_CRC_1_BYTE = 0,
    NRF24L01_CRC_2_BYTES
}nrf24l01_crc_type_t;

typedef enum {
    NRF24L01_PIPE0 = 0x01,
    NRF24L01_PIPE1 = 0x02,
    NRF24L01_PIPE2 = 0x04,
    NRF24L01_PIPE3 = 0x08,
    NRF24L01_PIPE4 = 0x10,
    NRF24L01_PIPE5 = 0x20,
}nrf24l01_pipe_t;

typedef enum {
    NRF24L01_RETRIES_DELAY_250US = 0,
    NRF24L01_RETRIES_DELAY_500US,
    NRF24L01_RETRIES_DELAY_750US,
    NRF24L01_RETRIES_DELAY_1000US,
    NRF24L01_RETRIES_DELAY_1250US,
    NRF24L01_RETRIES_DELAY_1500US,
    NRF24L01_RETRIES_DELAY_1750US,
    NRF24L01_RETRIES_DELAY_2000US,
    NRF24L01_RETRIES_DELAY_2250US,
    NRF24L01_RETRIES_DELAY_2500US,
    NRF24L01_RETRIES_DELAY_2750US,
    NRF24L01_RETRIES_DELAY_3000US,
    NRF24L01_RETRIES_DELAY_3250US,
    NRF24L01_RETRIES_DELAY_3500US,
    NRF24L01_RETRIES_DELAY_3750US,
    NRF24L01_RETRIES_DELAY_4000US,
}nrf24l01_retries_delay_t;

typedef void(*nrf24l01_spi_read_write_blocking_cb_t)(struct nrf24l01_t_ *p_hndl, const uint8_t *p_wr_data, uint8_t *p_rd_data, uint16_t len);
typedef void(*nrf24l01_spi_write_blocking_cb_t)(struct nrf24l01_t_ *p_hndl, const uint8_t *p_data, uint16_t len);
typedef void(*nrf24l01_spi_read_blocking_cb_t)(struct nrf24l01_t_ *p_hndl, uint8_t *p_data, uint16_t len);
                                            
typedef void(*nrf24l01_cs_select_cb_t)(struct nrf24l01_t_ *p_hndl);
typedef void(*nrf24l01_cs_deselect_cb_t)(struct nrf24l01_t_ *p_hndl);
typedef void(*nrf24l01_ce_set_cb_t)(struct nrf24l01_t_ *p_hndl);
typedef void(*nrf24l01_ce_unset_cb_t)(struct nrf24l01_t_ *p_hndl);
typedef void(*nrf24l01_ce_pulse_cb_t)(struct nrf24l01_t_ *p_hndl);

typedef struct {
    nrf24l01_rate_t         air_data_rate;
    nrf24l01_tx_pwr_t       tx_pwr;
    
    nrf24l01_lna_gain_t     lna_gain;
}nrf24l01_rf_setup_t;

/**
 * @brief This structure arrange following register "CONFIG" of NRF24L01.
 */
typedef struct {
    nrf24l01_prim_t         prim : 1;
    
    nrf24l01_bool_t         pwr_up : 1;
    
    nrf24l01_crc_type_t     crc_type : 1;
    nrf24l01_bool_t         crc_enabled : 1;
    
    nrf24l01_bool_t         max_rt_irq : 1;
    nrf24l01_bool_t         tx_ds_irq : 1;
    nrf24l01_bool_t         rx_dr_irq : 1;

    uint8_t                 __reserved : 1;
}nrf24l01_chip_config_t;

/**
 * @brief This structure arrange following register "STATUS" of NRF24L01.
 */
typedef struct {
    uint8_t                 tx_full : 1;
    uint8_t                 rx_p_no : 3;
    uint8_t                 max_rt : 1;
    uint8_t                 tx_ds : 1;
    uint8_t                 rx_dr : 1;
    
    uint8_t                 __reserved : 1;
}nrf24l01_chip_status_t;

typedef struct {
    uint8_t                 retr_count : 4;
    uint8_t                 lost_count : 4;
}nrf24l01_observe_tx_t;

typedef struct {
    nrf24l01_spi_read_write_blocking_cb_t               spi_rw_cb;
    nrf24l01_spi_write_blocking_cb_t                    spi_w_cb;
    nrf24l01_spi_read_blocking_cb_t                     spi_r_cb;
    nrf24l01_cs_select_cb_t                             cs_select_cb;
    nrf24l01_cs_deselect_cb_t                           cs_deselect_cb;
    nrf24l01_ce_set_cb_t                                ce_set_cb;
    nrf24l01_ce_unset_cb_t                              ce_unset_cb;
    nrf24l01_ce_pulse_cb_t                              ce_pulse;
}nrf24l01_ops_t;

typedef struct nrf24l01_t_ {
    const nrf24l01_ops_t                                *p_ops;
    
    nrf24l01_mode_t                                     current_mode;

    void                                                *user_data;
}nrf24l01_t;

/**
 * @brief nrf24l01_init
 * @param p_hndl
 * @param p_ops
 */
void nrf24l01_init(nrf24l01_t *p_hndl,
                   const nrf24l01_ops_t *p_ops);

/**
 * @brief nrf24l01_default_setup
 * @param p_hndl
 * @return
 */
nrf24l01_result_t nrf24l01_default_setup(nrf24l01_t *p_hndl);

/**
 * @brief nrf24l01_set_user_data
 * @param p_hndl
 * @param user_data
 */
void nrf24l01_set_user_data(nrf24l01_t *p_hndl,
                            void *user_data);

/**
 * @brief nrf24l01_get_user_data
 * @param p_hndl
 * @return
 */
void* nrf24l01_get_user_data(nrf24l01_t *p_hndl);

/**
 * @brief nrf24l01_setup_rf
 * @param p_hndl
 * @param p_setup
 * @return
 */
nrf24l01_result_t nrf24l01_setup_rf(nrf24l01_t *p_hndl,
                                    const nrf24l01_rf_setup_t *p_setup);

/**
 * @brief nrf24l01_set_rx_addr
 * @param p_hndl
 * @param pipe
 * @param addr
 * @param addr_len
 * @return
 */
nrf24l01_result_t nrf24l01_set_rx_addr(nrf24l01_t *p_hndl,
                                       nrf24l01_pipe_t pipe,
                                       const uint8_t *addr, 
                                       uint8_t addr_len);

/**
 * @brief nrf24l01_set_tx_addr
 * @param p_hndl
 * @param addr
 * @param addr_len
 * @return
 */
nrf24l01_result_t nrf24l01_set_tx_addr(nrf24l01_t *p_hndl,
                                       const uint8_t *addr,
                                       uint8_t addr_len);

/**
 * @brief nrf24l01_set_channel
 * @param p_hndl
 * @param ch
 * @return
 */
nrf24l01_result_t nrf24l01_set_channel(nrf24l01_t *p_hndl,
                                       uint8_t ch);

/**
 * @brief nrf24l01_set_mode
 * @param p_hndl
 * @param mode
 * @return
 */
nrf24l01_result_t nrf24l01_set_mode(nrf24l01_t *p_hndl,
                                    nrf24l01_mode_t mode);


/**
 * @brief nrf24l01_power_up
 * @param p_hndl
 * @return
 */
nrf24l01_result_t nrf24l01_power_up(nrf24l01_t *p_hndl);

/**
 * @brief nrf24l01_power_down
 * @param p_hndl
 * @return
 */
nrf24l01_result_t nrf24l01_power_down(nrf24l01_t *p_hndl);

/**
 * @brief nrf24l01_set_auto_ack
 * @param p_hndl
 * @param pipe
 * @param enabled
 * @return
 */
nrf24l01_result_t nrf24l01_set_auto_ack(nrf24l01_t *p_hndl,
                                        nrf24l01_pipe_t pipe,
                                        nrf24l01_bool_t enabled);

/**
 * @brief nrf24l01_set_retries
 * @param p_hndl
 * @param delay
 * @param count
 * @return
 */
nrf24l01_result_t nrf24l01_set_retries(nrf24l01_t *p_hndl,
                                       nrf24l01_retries_delay_t delay, 
                                       uint8_t count);

/**
 * @brief nrf24l01_set_crc
 * @param p_hndl
 * @param crc
 * @param enabled
 * @return
 */
nrf24l01_result_t nrf24l01_set_crc(nrf24l01_t *p_hndl,
                                   nrf24l01_crc_type_t crc,
                                   nrf24l01_bool_t enabled);

/**
 * @brief nrf24l01_set_payload_width
 * @param p_hndl
 * @param pipe
 * @param width
 * @return
 */
nrf24l01_result_t nrf24l01_set_payload_width(nrf24l01_t *p_hndl,
                                             nrf24l01_pipe_t pipe,
                                             uint8_t width);

/**
 * @brief nrf24l01_get_observe_tx
 * @param p_hndl
 * @return
 */
nrf24l01_observe_tx_t nrf24l01_get_observe_tx(nrf24l01_t *p_hndl);

/**
 * @brief nrf24l01_is_available
 * @param p_hndl
 * @param p_pipe_no
 * @return
 */
nrf24l01_bool_t nrf24l01_is_available(nrf24l01_t *p_hndl, uint8_t *p_pipe_no);

/**
 * @brief nrf24l01_send_blocking
 * @param p_hndl
 * @param p_data
 * @param len
 * @return
 */
nrf24l01_result_t nrf24l01_send_blocking(nrf24l01_t *p_hndl,
                                         const uint8_t *p_data,
                                         uint8_t len);

                                         /**
 * @brief nrf24l01_read_blocking
 * @param p_hndl
 * @param p_data
 * @param len
 * @return
 */
nrf24l01_result_t nrf24l01_read_blocking(nrf24l01_t *p_hndl,
                                         uint8_t *p_data,
                                         uint8_t len);

/**
 * @brief nrf24l01_irq
 * @param p_hndl
 */
void nrf24l01_irq(nrf24l01_t *p_hndl);


#endif /* _NRF24L01_H */
