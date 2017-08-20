/**
  ******************************************************************************
  * @file           : nrf24l01.c
  * @date           : 11 August 2017
  * @brief          : This file implements API driver for NRF24L01 chip.
  * @author         : Burin Sapsiri <burin.coding@gmail.com>
  *
  ******************************************************************************
*/

#include "nrf24l01.h"
#include "nrf24l01_defs.h"

#ifdef ONLINE_TEST_ENABLE
    #include "test_module.h"
#endif

#ifdef DEBUG_ENABLE
    #include "debug.h"
#endif

/**
 * @brief Private define
 */
#define MODIFY_BIT(reg, n, x)               reg ^= (-x ^ reg) & (1 << n)
#define MODIFY_REG(reg, mask, data)         do { \
                                                reg &= (~mask); \
                                                reg |= data; \
                                            }while(0);
#define UNUSED(x)                           ((void)x)

#define TX_NOT_FINISHED                     0U
#define TX_FINISH_SUCCESS                   1U
#define TX_FINISH_FAIL                      2U

#ifndef ONLINE_TEST_ENABLE
    #define ONLINE_CHECK_ERR(...)
    #define ONLINE_TEST_EQUAL(...)
    #define ONLINE_TEST_NOT_EQUAL(...)
    #define ONLINE_TEST_BYTE_EQUAL(...)
    #define ONLINE_TEST_BYTE_NOT_EQUAL(...)
    #define ONLINE_ASSERT_ERROR(...)
    #define ONLINE_TEST_STRING_EQUAL(...)
#endif

#ifndef DEBUG_ENABLE
    #define DEBUG_PRINT(...)
#endif

/**
 * @brief Private typedef
 */
typedef enum {
    IRQ_RX_DR = 0,
    IRQ_TX_DS,
    IRQ_MAX_RT,
}irq_type_t;


/**
 * @brief Extern variable
 */


/**
 * @brief Private variable
 */


/**
 * @brief Private function prototype;
 */

static void spi_read_write_blocking(nrf24l01_t *p_hndl,
                                    const uint8_t *p_wr_data,
                                    uint8_t *p_rd_data,
                                    uint16_t len);

static void spi_write_blocking(nrf24l01_t *p_hndl,
                               const uint8_t *p_wr_data,
                               uint16_t len);

static void spi_read_blocking(nrf24l01_t *p_hndl,
                              uint8_t *p_rd_data,
                              uint16_t len);

static uint8_t read_chip_status_blocking(nrf24l01_t *p_hndl);

static uint8_t read_chip_blocking(nrf24l01_t *p_hndl,
                                  uint8_t cmd,
                                  uint8_t *p_data,
                                  uint8_t len);

static uint8_t write_chip_blocking(nrf24l01_t *p_hndl,
                                   uint8_t cmd,
                                   const uint8_t *p_data,
                                   uint8_t len);

static uint8_t read_register_blocking(nrf24l01_t *p_hndl,
                                   uint8_t reg,
                                   uint8_t *p_data,
                                   uint8_t len);
static uint8_t write_register_blocking(nrf24l01_t *p_hndl,
                                    uint8_t reg,
                                    const uint8_t *p_data,
                                    uint8_t len);

static uint8_t read_payload_blocking(nrf24l01_t *p_hndl,
                                  uint8_t *p_data,
                                  uint8_t len);

static uint8_t write_payload_blocking(nrf24l01_t *p_hndl,
                                   const uint8_t *p_data,
                                   uint8_t len);

static uint8_t flush_tx_fifo_blocking(nrf24l01_t *p_hndl);
static uint8_t flush_rx_fifo_blocking(nrf24l01_t *p_hndl);

static void set_irq(nrf24l01_t *p_hndl,
                    irq_type_t type,
                    nrf24l01_bool_t enabled);

static void set_prim(nrf24l01_t *p_hndl,
                     nrf24l01_prim_t prim);

static void cs_select(nrf24l01_t *p_hndl);
static void cs_deselect(nrf24l01_t *p_hndl);
static void ce_set(nrf24l01_t *p_hndl);
static void ce_unset(nrf24l01_t *p_hndl);
static void ce_pulse(nrf24l01_t *p_hndl);

static nrf24l01_result_t default_setup(nrf24l01_t *p_hndl);
static nrf24l01_result_t factory_setup(nrf24l01_t *p_hndl);

static void rx_dr_irq_handler(nrf24l01_t *p_hndl);
static void tx_ds_irq_handler(nrf24l01_t *p_hndl);
static void max_rt_irq_handler(nrf24l01_t *p_hndl);


/**
 * @brief Callback function prototype
 */


/**
 * @addtogroup Public function
 * @{
 */
void nrf24l01_init(nrf24l01_t *p_hndl,
                   const nrf24l01_ops_t *p_ops)
{
    ONLINE_TEST_NOT_EQUAL(p_hndl, NULL);
    ONLINE_TEST_NOT_EQUAL(p_ops, NULL);
    ONLINE_TEST_NOT_EQUAL(p_ops->spi_rw_cb, NULL);
    ONLINE_TEST_NOT_EQUAL(p_ops->spi_w_cb, NULL);
    ONLINE_TEST_NOT_EQUAL(p_ops->spi_r_cb, NULL);
    ONLINE_TEST_NOT_EQUAL(p_ops->cs_select_cb, NULL);
    ONLINE_TEST_NOT_EQUAL(p_ops->cs_deselect_cb, NULL);
    ONLINE_TEST_NOT_EQUAL(p_ops->ce_set_cb, NULL);
    ONLINE_TEST_NOT_EQUAL(p_ops->ce_unset_cb, NULL);
    ONLINE_TEST_NOT_EQUAL(p_ops->ce_pulse, NULL);

    p_hndl->p_ops = p_ops;
    
    p_hndl->current_mode = NRF24L01_MODE_IDLE;

    cs_deselect(p_hndl);
    ce_unset(p_hndl);

    default_setup(p_hndl);
}

nrf24l01_result_t nrf24l01_default_setup(nrf24l01_t *p_hndl)
{
    ONLINE_TEST_NOT_EQUAL(p_hndl, NULL);

    return default_setup(p_hndl);
}

void nrf24l01_set_user_data(nrf24l01_t *p_hndl,
                            void *user_data)
{
    p_hndl->user_data = user_data;
}

void* nrf24l01_get_user_data(nrf24l01_t *p_hndl)
{
    return p_hndl->user_data;
}

nrf24l01_result_t nrf24l01_setup_rf(nrf24l01_t *p_hndl,
                                    const nrf24l01_rf_setup_t *p_setup)
{
    uint8_t data = 0;

    if (p_setup->air_data_rate == NRF24L01_RATE_1MBPS) {
        data |= NRF24L01_DEFVAL_REG_RF_SETUP_RF_DR_1MBPS;
    } else if (p_setup->air_data_rate == NRF24L01_RATE_2MBPS) {
        data |= NRF24L01_DEFVAL_REG_RF_SETUP_RF_DR_2MBPS;
    }

    if (p_setup->tx_pwr == NRF24L01_TX_PWR_M18_DBM) {
        data |= NRF24L01_DEFVAL_REG_RF_SETUP_RF_PWR_M18DBM;
    } else if (p_setup->tx_pwr == NRF24L01_TX_PWR_M12_DBM) {
        data |= NRF24L01_DEFVAL_REG_RF_SETUP_RF_PWR_M12DBM;
    } else if (p_setup->tx_pwr == NRF24L01_TX_PWR_M6_DBM) {
        data |= NRF24L01_DEFVAL_REG_RF_SETUP_RF_PWR_M6DBM;
    } else if (p_setup->tx_pwr == NRF24L01_TX_PWR_0_DBM) {
        data |= NRF24L01_DEFVAL_REG_RF_SETUP_RF_PWR_0DBM;
    }

    if (p_setup->lna_gain == NRF24L01_LNA_GAIN_OFF) {
        data |= NRF24L01_DEFVAL_REG_RF_SETUP_LNA_GAIN_OFF;
    } else if (p_setup->lna_gain == NRF24L01_LNA_GAIN_ON) {
        data |= NRF24L01_DEFVAL_REG_RF_SETUP_LNA_GAIN_ON;
    }

    write_register_blocking(p_hndl, NRF24L01_REG_RF_SETUP, &data, 1);

    return NRF24L01_SUCCESS;
}

nrf24l01_result_t nrf24l01_set_rx_addr(nrf24l01_t *p_hndl,
                                       nrf24l01_pipe_t pipe,
                                       const uint8_t *addr,
                                       uint8_t addr_len)
{
    uint8_t tmp_addr = 0;

    ONLINE_TEST_NOT_EQUAL(addr, NULL);

    /* Allow 2 to 5 bytes address for pipe 0 to 1, otherwise allow only 1 byte address. */
    ONLINE_CHECK_ERR((((pipe == NRF24L01_PIPE0) || (pipe == NRF24L01_PIPE1)) && (addr_len >= 2) && (addr_len <= 5)) ||
                     ((pipe != NRF24L01_PIPE0) && (pipe != NRF24L01_PIPE1) && (addr_len == 1)));

    if ((pipe != NRF24L01_PIPE0) &&
        (pipe != NRF24L01_PIPE1)) {

        tmp_addr = addr[0];
    }

    if (pipe == NRF24L01_PIPE0) {
        write_register_blocking(p_hndl, NRF24L01_REG_RX_ADDR_P0, addr, addr_len);
    } else if (pipe == NRF24L01_PIPE1) {
        write_register_blocking(p_hndl, NRF24L01_REG_RX_ADDR_P1, addr, addr_len);
    } else if (pipe == NRF24L01_PIPE2) {
        write_register_blocking(p_hndl, NRF24L01_REG_RX_ADDR_P2, &tmp_addr, 1);
    } else if (pipe == NRF24L01_PIPE3) {
        write_register_blocking(p_hndl, NRF24L01_REG_RX_ADDR_P3, &tmp_addr, 1);
    } else if (pipe == NRF24L01_PIPE4) {
        write_register_blocking(p_hndl, NRF24L01_REG_RX_ADDR_P4, &tmp_addr, 1);
    } else if (pipe == NRF24L01_PIPE5) {
        write_register_blocking(p_hndl, NRF24L01_REG_RX_ADDR_P5, &tmp_addr, 1);
    }

    return NRF24L01_SUCCESS;
}

nrf24l01_result_t nrf24l01_set_tx_addr(nrf24l01_t *p_hndl,
                                       const uint8_t *addr,
                                       uint8_t addr_len)
{
    write_register_blocking(p_hndl, NRF24L01_REG_TX_ADDR, addr, addr_len);

    return NRF24L01_SUCCESS;
}

nrf24l01_result_t nrf24l01_set_channel(nrf24l01_t *p_hndl,
                                       uint8_t ch)
{
    write_register_blocking(p_hndl, NRF24L01_REG_RF_CH, &ch, 1);

    return NRF24L01_SUCCESS;
}

nrf24l01_result_t nrf24l01_set_mode(nrf24l01_t *p_hndl,
                                    nrf24l01_mode_t mode)
{  
    if (mode == NRF24L01_MODE_IDLE) {
        ce_unset(p_hndl);
        p_hndl->current_mode = NRF24L01_MODE_IDLE;
    } else if (mode == NRF24L01_MODE_RX) {
        set_prim(p_hndl, NRF24L01_PRIM_RX);
        ce_set(p_hndl);
        p_hndl->current_mode = NRF24L01_MODE_RX;
    } else if (mode == NRF24L01_MODE_TX) {
        ce_unset(p_hndl);
        set_prim(p_hndl, NRF24L01_PRIM_TX);
        p_hndl->current_mode = NRF24L01_MODE_TX;
    }

    return NRF24L01_SUCCESS;
}

nrf24l01_result_t nrf24l01_power_up(nrf24l01_t *p_hndl)
{
    uint8_t config_reg_value = 0;

    read_register_blocking(p_hndl, NRF24L01_REG_CONFIG, &config_reg_value, 1);

    config_reg_value |= NRF24L01_DEFVAL_CONFIG_PWR_UP;

    write_register_blocking(p_hndl, NRF24L01_REG_CONFIG, &config_reg_value, 1);

    return NRF24L01_SUCCESS;
}

nrf24l01_result_t nrf24l01_power_down(nrf24l01_t *p_hndl)
{
    uint8_t config_reg_value = 0;

    read_register_blocking(p_hndl, NRF24L01_REG_CONFIG, &config_reg_value, 1);

    config_reg_value &= ~(NRF24L01_DEFVAL_CONFIG_PWR_UP);

    write_register_blocking(p_hndl, NRF24L01_REG_CONFIG, &config_reg_value, 1);

    return NRF24L01_SUCCESS;
}

nrf24l01_result_t nrf24l01_set_auto_ack(nrf24l01_t *p_hndl,
                                        nrf24l01_pipe_t pipe,
                                        nrf24l01_bool_t enabled)
{
    uint8_t en_aa = 0;

    read_register_blocking(p_hndl, NRF24L01_REG_EN_AA, &en_aa, 1);

    if (enabled == NRF24L01_TRUE) {
        en_aa |= (uint8_t)pipe;
    } else {
        en_aa &= ~((uint8_t)pipe);
    }

    write_register_blocking(p_hndl, NRF24L01_REG_EN_AA, &en_aa, 1);

    return NRF24L01_SUCCESS;
}

nrf24l01_result_t nrf24l01_set_retries(nrf24l01_t *p_hndl,
                                       nrf24l01_retries_delay_t delay,
                                       uint8_t count)
{
    uint8_t setup_retr = 0;

    setup_retr |= ((uint8_t)delay << 4) | count;

    write_register_blocking(p_hndl, NRF24L01_REG_SETUP_RETR, &setup_retr, 1);

    return NRF24L01_SUCCESS;
}

nrf24l01_result_t nrf24l01_set_crc(nrf24l01_t *p_hndl,
                                   nrf24l01_crc_type_t crc,
                                   nrf24l01_bool_t enabled)
{
    uint8_t config = 0;

    read_register_blocking(p_hndl, NRF24L01_REG_CONFIG, &config, 1);

    if (crc == NRF24L01_CRC_1_BYTE) {
        config &= ~(NRF24L01_DEFVAL_CONFIG_CRC0);
    } else if (crc == NRF24L01_CRC_2_BYTES) {
        config |= NRF24L01_DEFVAL_CONFIG_CRC0;
    }

    if (enabled == NRF24L01_TRUE) {
        config |= NRF24L01_DEFVAL_CONFIG_EN_CRC;
    } else {
        config &= ~(NRF24L01_DEFVAL_CONFIG_EN_CRC);
    }

    write_register_blocking(p_hndl, NRF24L01_REG_CONFIG, &config, 1);

    return NRF24L01_SUCCESS;
}

nrf24l01_result_t nrf24l01_set_payload_width(nrf24l01_t *p_hndl,
                                             nrf24l01_pipe_t pipe,
                                             uint8_t width)
{
    if (pipe == NRF24L01_PIPE0) {
        write_register_blocking(p_hndl, NRF24L01_REG_RX_PW_P0, &width, 1);
    } else if (pipe == NRF24L01_PIPE1) {
        write_register_blocking(p_hndl, NRF24L01_REG_RX_PW_P1, &width, 1);
    } else if (pipe == NRF24L01_PIPE2) {
        write_register_blocking(p_hndl, NRF24L01_REG_RX_PW_P2, &width, 1);
    } else if (pipe == NRF24L01_PIPE3) {
        write_register_blocking(p_hndl, NRF24L01_REG_RX_PW_P3, &width, 1);
    } else if (pipe == NRF24L01_PIPE4) {
        write_register_blocking(p_hndl, NRF24L01_REG_RX_PW_P4, &width, 1);
    } else if (pipe == NRF24L01_PIPE5) {
        write_register_blocking(p_hndl, NRF24L01_REG_RX_PW_P5, &width, 1);
    }
    
    return NRF24L01_SUCCESS;
}

nrf24l01_observe_tx_t nrf24l01_get_observe_tx(nrf24l01_t *p_hndl)
{
    uint8_t observe_tx = 0;
    nrf24l01_observe_tx_t ret = {0};
    
    read_register_blocking(p_hndl, NRF24L01_REG_OBSERVE_TX, &observe_tx, 1);
    
    ret.lost_count = (observe_tx & NRF24L01_REG_OBSERVE_TX_PLOS_CNT_MASK) >> 4;
    ret.retr_count = (observe_tx & NRF24L01_REG_OBSERVE_TX_ARC_CNT_MASK);
    
    return ret;
}

nrf24l01_bool_t nrf24l01_is_available(nrf24l01_t *p_hndl, uint8_t *p_pipe_no)
{
    uint8_t status = 0;
    uint8_t rx_p_no = 0;
    nrf24l01_bool_t ret = NRF24L01_FALSE;

    status = read_chip_status_blocking(p_hndl);
    
    rx_p_no = status & NRF24L01_REG_STATUS_RX_P_NO_MASK;
    
    if (rx_p_no != NRF24L01_DEFVAL_REG_STATUS_RX_P_NO_EMPTY) {
        ret = NRF24L01_TRUE;
        
        if (p_pipe_no != NULL) {
            if (rx_p_no == NRF24L01_DEFVAL_REG_STATUS_RX_P_NO_PIPE0) {
                *p_pipe_no = 0;
            } else if (rx_p_no == NRF24L01_DEFVAL_REG_STATUS_RX_P_NO_PIPE1) {
                *p_pipe_no = 1;
            } else if (rx_p_no == NRF24L01_DEFVAL_REG_STATUS_RX_P_NO_PIPE2) {
                *p_pipe_no = 2;
            } else if (rx_p_no == NRF24L01_DEFVAL_REG_STATUS_RX_P_NO_PIPE3) {
                *p_pipe_no = 3;
            } else if (rx_p_no == NRF24L01_DEFVAL_REG_STATUS_RX_P_NO_PIPE4) {
                *p_pipe_no = 4;
            } else if (rx_p_no == NRF24L01_DEFVAL_REG_STATUS_RX_P_NO_PIPE5) {
                *p_pipe_no = 5;
            }
        }
    } else {
        ret = NRF24L01_FALSE;
    }
    
    return ret;
}

nrf24l01_result_t nrf24l01_send_blocking(nrf24l01_t *p_hndl,
                                         const uint8_t *p_data,
                                         uint8_t len)
{
    uint8_t status = 0;
    uint8_t clear_irq = 0;
    UNUSED(status);
    UNUSED(clear_irq);

    ONLINE_CHECK_ERR(len <= NRF24L01_MAX_PAYLOAD);
    
    if (p_hndl->current_mode != NRF24L01_MODE_TX) {
        return NRF24L01_FAIL;
    }

    write_payload_blocking(p_hndl, p_data, len);
    
    /* Start transmission. */
    ce_pulse(p_hndl);
    
    status = read_chip_status_blocking(p_hndl);
    
    /* Wait for TX_DS or MAX_RT flag is set. */
    while (((status & NRF24L01_REG_STATUS_MAX_RT_MASK) == 0) &&
           ((status & NRF24L01_REG_STATUS_TX_DS_MASK) == 0)) {
        status = read_chip_status_blocking(p_hndl);
    }
    
    /* If transmission fail, the payload is still in TX FIFO. Flush it. */
    if ((status & NRF24L01_REG_STATUS_MAX_RT_MASK) != 0) {
        flush_tx_fifo_blocking(p_hndl);
    }

    /* Clear TX_DS and MAX_RT pendding bits. */
    clear_irq = NRF24L01_REG_STATUS_TX_DS_MASK | NRF24L01_REG_STATUS_MAX_RT_MASK;
    write_register_blocking(p_hndl, NRF24L01_REG_STATUS, &clear_irq, 1);
    
    if ((status & NRF24L01_REG_STATUS_TX_DS_MASK) != 0) {
        return NRF24L01_SUCCESS;
    } else {
        return NRF24L01_FAIL;
    }
}

nrf24l01_result_t nrf24l01_read_blocking(nrf24l01_t *p_hndl,
                                         uint8_t *p_data,
                                         uint8_t len)
{
    read_payload_blocking(p_hndl, p_data, len);
    
    return NRF24L01_SUCCESS;
}

void nrf24l01_irq(nrf24l01_t *p_hndl)
{
    UNUSED(p_hndl);
}

/**
 * @}
 */



/**
 * @addtogroup Private function
 * @{
 */

static void spi_read_write_blocking(nrf24l01_t *p_hndl,
                                    const uint8_t *p_wr_data,
                                    uint8_t *p_rd_data,
                                    uint16_t len)
{
    ONLINE_TEST_NOT_EQUAL(p_hndl, NULL);
    ONLINE_TEST_NOT_EQUAL(p_hndl->p_ops, NULL);
    ONLINE_TEST_NOT_EQUAL(p_hndl->p_ops->spi_rw_cb, NULL);

    p_hndl->p_ops->spi_rw_cb(p_hndl, p_wr_data, p_rd_data, len);
}

static void spi_write_blocking(nrf24l01_t *p_hndl,
                               const uint8_t *p_wr_data,
                               uint16_t len)
{
    ONLINE_TEST_NOT_EQUAL(p_hndl, NULL);
    ONLINE_TEST_NOT_EQUAL(p_hndl->p_ops, NULL);
    ONLINE_TEST_NOT_EQUAL(p_hndl->p_ops->spi_w_cb, NULL);

    p_hndl->p_ops->spi_w_cb(p_hndl, p_wr_data, len);
}

static void spi_read_blocking(nrf24l01_t *p_hndl,
                              uint8_t *p_rd_data,
                              uint16_t len)
{
    ONLINE_TEST_NOT_EQUAL(p_hndl, NULL);
    ONLINE_TEST_NOT_EQUAL(p_hndl->p_ops, NULL);
    ONLINE_TEST_NOT_EQUAL(p_hndl->p_ops->spi_r_cb, NULL);

    p_hndl->p_ops->spi_r_cb(p_hndl, p_rd_data, len);
}

static uint8_t read_chip_status_blocking(nrf24l01_t *p_hndl)
{
    uint8_t c_status = 0;
    uint8_t cmd = 0;

    ONLINE_TEST_NOT_EQUAL(p_hndl, NULL);

    cs_select(p_hndl);

    cmd = NRF24L01_CMD_NOP;

    /* Write NOP command to chip. And receive chip status back. */
    spi_read_write_blocking(p_hndl, &cmd, &c_status, 1);

    cs_deselect(p_hndl);

    return c_status;
}

static uint8_t read_chip_blocking(nrf24l01_t *p_hndl,
                                  uint8_t cmd,
                                  uint8_t *p_data,
                                  uint8_t len)
{
    uint8_t c_status = 0;

    ONLINE_TEST_NOT_EQUAL(p_hndl, NULL);

    /* If data length > 0, p_data must not be NULL. */
    ONLINE_CHECK_ERR((len == 0) || (p_data != NULL));

    cs_select(p_hndl);

    /* Write command and register address to chip. And receive chip status back. */
    spi_read_write_blocking(p_hndl, &cmd, &c_status, 1);

    if (len > 0) {
        /* Read register data from chip. */
        spi_read_blocking(p_hndl, p_data, len);
    }

    cs_deselect(p_hndl);

    return c_status;
}

static uint8_t write_chip_blocking(nrf24l01_t *p_hndl,
                                   uint8_t cmd,
                                   const uint8_t *p_data,
                                   uint8_t len)
{
    uint8_t c_status = 0;

    ONLINE_TEST_NOT_EQUAL(p_hndl, NULL);

    /* If data length > 0, p_data must not be NULL. */
    ONLINE_CHECK_ERR((len == 0) || (p_data != NULL));

    cs_select(p_hndl);

    /* Write command and register address to chip. And receive chip status back. */
    spi_read_write_blocking(p_hndl, &cmd, &c_status, 1);

    if (len > 0) {
        /* Write register data to chip. */
        spi_write_blocking(p_hndl, p_data, len);
    }

    cs_deselect(p_hndl);

    return c_status;
}

static uint8_t read_register_blocking(nrf24l01_t *p_hndl,
                                      uint8_t reg,
                                      uint8_t *p_data,
                                      uint8_t len)
{
    uint8_t cmd = 0;
    uint8_t c_status = 0;

    ONLINE_TEST_NOT_EQUAL(p_hndl, NULL);
    ONLINE_TEST_NOT_EQUAL(p_data, NULL);

    cmd = NRF24L01_CMD_R_REGISTER | (reg & NRF24L01_CMD_R_REGISTER_ADDR_MASK);

    c_status = read_chip_blocking(p_hndl, cmd, p_data, len);

    return c_status;
}

static uint8_t write_register_blocking(nrf24l01_t *p_hndl,
                                       uint8_t reg,
                                       const uint8_t *p_data,
                                       uint8_t len)
{
    uint8_t cmd = 0;
    uint8_t c_status = 0;

    ONLINE_TEST_NOT_EQUAL(p_hndl, NULL);
    ONLINE_TEST_NOT_EQUAL(p_data, NULL);

    cmd = NRF24L01_CMD_W_REGISTER | (reg & NRF24L01_CMD_W_REGISTER_ADDR_MASK);

    c_status = write_chip_blocking(p_hndl, cmd, p_data, len);

    return c_status;
}

static uint8_t read_payload_blocking(nrf24l01_t *p_hndl,
                                     uint8_t *p_data,
                                     uint8_t len)
{
    uint8_t cmd = 0;
    uint8_t c_status = 0;

    ONLINE_TEST_NOT_EQUAL(p_hndl, NULL);
    ONLINE_TEST_NOT_EQUAL(p_data, NULL);
    ONLINE_CHECK_ERR(len <= NRF24L01_MAX_PAYLOAD);

    cmd = NRF24L01_CMD_R_RX_PAYLOAD;

    c_status = read_chip_blocking(p_hndl, cmd, p_data, len);

    return c_status;
}

static uint8_t write_payload_blocking(nrf24l01_t *p_hndl,
                                      const uint8_t *p_data,
                                      uint8_t len)
{
    uint8_t cmd = 0;
    uint8_t c_status = 0;

    ONLINE_TEST_NOT_EQUAL(p_hndl, NULL);
    ONLINE_TEST_NOT_EQUAL(p_data, NULL);
    ONLINE_CHECK_ERR(len <= NRF24L01_MAX_PAYLOAD);

    cmd = NRF24L01_CMD_W_TX_PAYLOAD;

    c_status = write_chip_blocking(p_hndl, cmd, p_data, len);

    return c_status;
}

static uint8_t flush_tx_fifo_blocking(nrf24l01_t *p_hndl)
{
    uint8_t cmd = 0;
    uint8_t c_status = 0;

    ONLINE_TEST_NOT_EQUAL(p_hndl, NULL);

    cmd = NRF24L01_CMD_FLUSH_TX;

    c_status = write_chip_blocking(p_hndl, cmd, NULL, 0);

    return c_status;
}

static uint8_t flush_rx_fifo_blocking(nrf24l01_t *p_hndl)
{
    uint8_t cmd = 0;
    uint8_t c_status = 0;

    ONLINE_TEST_NOT_EQUAL(p_hndl, NULL);

    cmd = NRF24L01_CMD_FLUSH_RX;

    c_status = write_chip_blocking(p_hndl, cmd, NULL, 0);

    return c_status;
}

static void set_irq(nrf24l01_t *p_hndl,
                    irq_type_t type,
                    nrf24l01_bool_t enabled)
{
    uint8_t config_reg_value = 0;

    read_register_blocking(p_hndl, NRF24L01_REG_CONFIG, &config_reg_value, 1);

    if (type == IRQ_RX_DR) {
        if (enabled == NRF24L01_FALSE) {
            config_reg_value |= NRF24L01_DEFVAL_CONFIG_RX_DR;
        } else {
            config_reg_value &= ~(NRF24L01_DEFVAL_CONFIG_RX_DR);
        }
    } else if (type == IRQ_TX_DS) {
        if (enabled == NRF24L01_FALSE) {
            config_reg_value |= NRF24L01_DEFVAL_CONFIG_TX_DS;
        } else {
            config_reg_value &= ~(NRF24L01_DEFVAL_CONFIG_TX_DS);
        }
    } else if (type == IRQ_MAX_RT) {
        if (enabled == NRF24L01_FALSE) {
            config_reg_value |= NRF24L01_DEFVAL_CONFIG_MAX_RT;
        } else {
            config_reg_value &= ~(NRF24L01_DEFVAL_CONFIG_MAX_RT);
        }
    }

    write_register_blocking(p_hndl, NRF24L01_REG_CONFIG, &config_reg_value, 1);
}

static void set_prim(nrf24l01_t *p_hndl,
                     nrf24l01_prim_t prim)
{
    uint8_t config_reg_value = 0;

    read_register_blocking(p_hndl, NRF24L01_REG_CONFIG, &config_reg_value, 1);

    if (prim == NRF24L01_PRIM_RX) {
        config_reg_value |= NRF24L01_DEFVAL_CONFIG_PRIM_RX_PRX;
    } else if (prim == NRF24L01_PRIM_TX) {
        config_reg_value &= ~(NRF24L01_DEFVAL_CONFIG_PRIM_RX_PRX);
    }

    write_register_blocking(p_hndl, NRF24L01_REG_CONFIG, &config_reg_value, 1);
}

static void cs_select(nrf24l01_t *p_hndl)
{
    p_hndl->p_ops->cs_select_cb(p_hndl);
}

static void cs_deselect(nrf24l01_t *p_hndl)
{
    p_hndl->p_ops->cs_deselect_cb(p_hndl);
}

static void ce_set(nrf24l01_t *p_hndl)
{
    p_hndl->p_ops->ce_set_cb(p_hndl);
}

static void ce_unset(nrf24l01_t *p_hndl)
{
    p_hndl->p_ops->ce_unset_cb(p_hndl);
}

static void ce_pulse(nrf24l01_t *p_hndl)
{
    p_hndl->p_ops->ce_pulse(p_hndl);
}

static nrf24l01_result_t default_setup(nrf24l01_t *p_hndl)
{
    factory_setup(p_hndl);

    /* Use IRQ only for RX. */
    set_irq(p_hndl, IRQ_RX_DR, NRF24L01_TRUE);
    set_irq(p_hndl, IRQ_TX_DS, NRF24L01_FALSE);
    set_irq(p_hndl, IRQ_MAX_RT, NRF24L01_FALSE);

    return NRF24L01_SUCCESS;
}

static nrf24l01_result_t factory_setup(nrf24l01_t *p_hndl)
{
    uint8_t data = 0;
    uint8_t data_addr[5] = {0};

    /* Write all register default value to chip. */
    data = NRF24L01_DEFVAL_REG_CONFIG_RESET;
    write_register_blocking(p_hndl, NRF24L01_REG_CONFIG, &data, 1);

    data = NRF24L01_DEFVAL_REG_EN_AA_RESET;
    write_register_blocking(p_hndl, NRF24L01_REG_EN_AA, &data, 1);

    data = NRF24L01_DEFVAL_REG_EN_RXADDR_RESET;
    write_register_blocking(p_hndl, NRF24L01_REG_EN_RXADDR, &data, 1);

    data = NRF24L01_DEFVAL_REG_SETUP_AW_RESET;
    write_register_blocking(p_hndl, NRF24L01_REG_SETUP_AW, &data, 1);

    data = NRF24L01_DEFVAL_REG_SETUP_RETR_RESET;
    write_register_blocking(p_hndl, NRF24L01_REG_SETUP_RETR, &data, 1);

    data  = NRF24L01_DEFVAL_REG_RF_CH_RESET;
    write_register_blocking(p_hndl, NRF24L01_REG_RF_CH, &data, 1);

    data = NRF24L01_DEFVAL_REG_RF_SETUP_RESET;
    write_register_blocking(p_hndl, NRF24L01_REG_RF_SETUP, &data, 1);

    data_addr[0] = NRF24L01_DEFVAL_REG_RX_ADDR_P0_0_RESET;
    data_addr[1] = NRF24L01_DEFVAL_REG_RX_ADDR_P0_1_RESET;
    data_addr[2] = NRF24L01_DEFVAL_REG_RX_ADDR_P0_2_RESET;
    data_addr[3] = NRF24L01_DEFVAL_REG_RX_ADDR_P0_3_RESET;
    data_addr[4] = NRF24L01_DEFVAL_REG_RX_ADDR_P0_4_RESET;
    write_register_blocking(p_hndl, NRF24L01_REG_RX_ADDR_P0, data_addr, 5);

    data_addr[0] = NRF24L01_DEFVAL_REG_RX_ADDR_P1_0_RESET;
    data_addr[1] = NRF24L01_DEFVAL_REG_RX_ADDR_P1_1_RESET;
    data_addr[2] = NRF24L01_DEFVAL_REG_RX_ADDR_P1_2_RESET;
    data_addr[3] = NRF24L01_DEFVAL_REG_RX_ADDR_P1_3_RESET;
    data_addr[4] = NRF24L01_DEFVAL_REG_RX_ADDR_P1_4_RESET;
    write_register_blocking(p_hndl, NRF24L01_REG_RX_ADDR_P1, data_addr, 5);

    data = NRF24L01_DEFVAL_REG_RX_ADDR_P2_RESET;
    write_register_blocking(p_hndl, NRF24L01_REG_RX_ADDR_P2, &data, 1);

    data = NRF24L01_DEFVAL_REG_RX_ADDR_P3_RESET;
    write_register_blocking(p_hndl, NRF24L01_REG_RX_ADDR_P3, &data, 1);

    data = NRF24L01_DEFVAL_REG_RX_ADDR_P4_RESET;
    write_register_blocking(p_hndl, NRF24L01_REG_RX_ADDR_P4, &data, 1);

    data = NRF24L01_DEFVAL_REG_RX_ADDR_P5_RESET;
    write_register_blocking(p_hndl, NRF24L01_REG_RX_ADDR_P5, &data, 1);

    data_addr[0] = NRF24L01_DEFVAL_REG_TX_ADDR_0_RESET;
    data_addr[1] = NRF24L01_DEFVAL_REG_TX_ADDR_1_RESET;
    data_addr[2] = NRF24L01_DEFVAL_REG_TX_ADDR_2_RESET;
    data_addr[3] = NRF24L01_DEFVAL_REG_TX_ADDR_3_RESET;
    data_addr[4] = NRF24L01_DEFVAL_REG_TX_ADDR_4_RESET;
    write_register_blocking(p_hndl, NRF24L01_REG_TX_ADDR, data_addr, 5);

    data = NRF24L01_DEFVAL_REG_RX_PW_P0_RESET;
    write_register_blocking(p_hndl, NRF24L01_REG_RX_PW_P0, &data, 1);

    data = NRF24L01_DEFVAL_REG_RX_PW_P1_RESET;
    write_register_blocking(p_hndl, NRF24L01_REG_RX_PW_P1, &data, 1);

    data = NRF24L01_DEFVAL_REG_RX_PW_P2_RESET;
    write_register_blocking(p_hndl, NRF24L01_REG_RX_PW_P2, &data, 1);

    data = NRF24L01_DEFVAL_REG_RX_PW_P3_RESET;
    write_register_blocking(p_hndl, NRF24L01_REG_RX_PW_P3, &data, 1);

    data = NRF24L01_DEFVAL_REG_RX_PW_P4_RESET;
    write_register_blocking(p_hndl, NRF24L01_REG_RX_PW_P4, &data, 1);

    data = NRF24L01_DEFVAL_REG_RX_PW_P5_RESET;
    write_register_blocking(p_hndl, NRF24L01_REG_RX_PW_P5, &data, 1);

    data = NRF24L01_DEFVAL_REG_DYNPD_RESET;
    write_register_blocking(p_hndl, NRF24L01_REG_DYNPD, &data, 1);

    data = NRF24L01_DEFVAL_REG_FEATURE_RESET;
    write_register_blocking(p_hndl, NRF24L01_REG_FEATURE, &data, 1);

    /* Flush TX and RX FIFO. */
    flush_tx_fifo_blocking(p_hndl);
    flush_rx_fifo_blocking(p_hndl);

    /* Clear IRQ pendding bits. */
    data = NRF24L01_REG_STATUS_RX_DR_MASK | NRF24L01_REG_STATUS_TX_DS_MASK | NRF24L01_REG_STATUS_MAX_RT_MASK;
    write_register_blocking(p_hndl, NRF24L01_REG_STATUS, &data, 1);

    return NRF24L01_SUCCESS;
}

static void rx_dr_irq_handler(nrf24l01_t *p_hndl)
{

}

static void tx_ds_irq_handler(nrf24l01_t *p_hndl)
{

}

static void max_rt_irq_handler(nrf24l01_t *p_hndl)
{

}


/**
 * @}
 */


/**
 * @addtogroup Callback function
 * @{
 */

/**
 * @}
 */


#ifdef UNITTEST_ENABLE

#include <string.h>
#include <hal_gpio.h>
#include <hal_spi.h>
#include <hal_system.h>
#include "debug.h"
#include "nrf24l01.h"

#define CS_PIN                              16
#define CE_PIN                              17
#define SPI_CH                              0
#define LED_PIN                             31

#define SPI_SPEED                           4000000UL
#define RF_CHANNEL                          50
#define WAKING_UP_DELAY                     5
#define TX_PERIOD                           50

#define RETRY_DELAY                         NRF24L01_RETRIES_DELAY_500US
#define RETRY_COUNT                         15

static nrf24l01_t unittest_hndl;
static hal_spi_t *p_unittest_spi_hndl;

static void UNITTEST_NRF24L01_SPI_READ_WRITE(nrf24l01_t *p_hndl,
                                             const uint8_t *p_w,
                                             uint8_t *p_r,
                                             uint16_t len);
static void UNITTEST_NRF24L01_SPI_WRITE(nrf24l01_t *p_hndl,
                                        const uint8_t *p_w,
                                        uint16_t len);
static void UNITTEST_NRF24L01_SPI_READ(nrf24l01_t *p_hndl,
                                       uint8_t *p_r,
                                       uint16_t len);
static void UNITTEST_NRF24L01_CS_SELECT(nrf24l01_t *p_hndl);
static void UNITTEST_NRF24L01_CS_DESELECT(nrf24l01_t *p_hndl);
static void UNITTEST_NRF24L01_CE_SET(nrf24l01_t *p_hndl);
static void UNITTEST_NRF24L01_CE_UNSET(nrf24l01_t *p_hndl);
static void UNITTEST_NRF24L01_CE_PULSE(nrf24l01_t *p_hndl);

static uint8_t device0_addr[5] = {0x00, 0x00, 0x00, 0x00, 0x00};
static uint8_t device1_addr[5] = {0x00, 0x00, 0x00, 0x00, 0x01};

static const nrf24l01_ops_t unittest_ops = {
    .spi_rw_cb = UNITTEST_NRF24L01_SPI_READ_WRITE,
    .spi_w_cb = UNITTEST_NRF24L01_SPI_WRITE,
    .spi_r_cb = UNITTEST_NRF24L01_SPI_READ,
    .cs_select_cb = UNITTEST_NRF24L01_CS_SELECT,
    .cs_deselect_cb = UNITTEST_NRF24L01_CS_DESELECT,
    .ce_set_cb = UNITTEST_NRF24L01_CE_SET,
    .ce_unset_cb = UNITTEST_NRF24L01_CE_UNSET,
    .ce_pulse = UNITTEST_NRF24L01_CE_PULSE,
};

void UNITTEST_NRF24L01(void)
{
    uint8_t rx0_addr[5] = {0};
    uint8_t tx_addr[5] = {0};
    nrf24l01_chip_config_t conf = {0};
    uint8_t tx_data[32];
    nrf24l01_result_t result = NRF24L01_SUCCESS;
    nrf24l01_observe_tx_t observe = {0};

    DEBUG_PRINT("************************************************************\r\n");
    DEBUG_PRINT("* UNITTEST_NRF24L01 started.                               *\r\n");
    DEBUG_PRINT("************************************************************\r\n");

    hal_gpio_export(CS_PIN);
    hal_gpio_dir(CS_PIN, HAL_GPIO_OUTPUT);
    hal_gpio_pull(CS_PIN, HAL_GPIO_NOPULL);

    hal_gpio_export(CE_PIN);
    hal_gpio_dir(CE_PIN, HAL_GPIO_OUTPUT);
    hal_gpio_pull(CE_PIN, HAL_GPIO_NOPULL);

    p_unittest_spi_hndl = hal_spi(SPI_CH);
    hal_spi_init(p_unittest_spi_hndl);
    hal_spi_set_speed(p_unittest_spi_hndl, SPI_SPEED);
    DEBUG_PRINT("SPI Inited.\r\n");

    nrf24l01_init(&unittest_hndl,
                  &unittest_ops);
    DEBUG_PRINT("Inited.\r\n");

    nrf24l01_power_up(&unittest_hndl);

    hal_system_delay(WAKING_UP_DELAY);

    tx_addr[0] = 0xE2;
    tx_addr[1] = 0xE2;
    tx_addr[2] = 0xE2;
    tx_addr[3] = 0xE2;
    tx_addr[4] = 0xE2;
    nrf24l01_set_tx_addr(&unittest_hndl, tx_addr, 5);
    nrf24l01_set_rx_addr(&unittest_hndl, NRF24L01_PIPE0, tx_addr, 5);

    nrf24l01_set_channel(&unittest_hndl, RF_CHANNEL);

    nrf24l01_rf_setup_t setup;
    setup.tx_pwr = NRF24L01_TX_PWR_0_DBM;
    setup.air_data_rate = NRF24L01_RATE_1MBPS;
    setup.lna_gain = NRF24L01_LNA_GAIN_OFF;
    nrf24l01_setup_rf(&unittest_hndl, &setup);

    nrf24l01_set_crc(&unittest_hndl, NRF24L01_CRC_2_BYTES, NRF24L01_TRUE);
    
    nrf24l01_set_retries(&unittest_hndl, RETRY_DELAY, RETRY_COUNT);
    
    read_register_blocking(&unittest_hndl, NRF24L01_REG_CONFIG, (uint8_t*)&conf, 1);
    DEBUG_PRINT("RX_DR=%d\r\n", conf.rx_dr_irq);
    DEBUG_PRINT("TX_DS=%d\r\n", conf.tx_ds_irq);
    DEBUG_PRINT("MAX_RT=%d\r\n", conf.max_rt_irq);
    DEBUG_PRINT("EN_CRC=%d\r\n", conf.crc_enabled);
    DEBUG_PRINT("CRCO=%d\r\n", conf.crc_type);
    DEBUG_PRINT("PWR_UP=%d\r\n", conf.pwr_up);
    DEBUG_PRINT("PRIM_RX=%d\r\n", conf.prim);

    read_register_blocking(&unittest_hndl, NRF24L01_REG_RX_ADDR_P0, rx0_addr, 5);
    DEBUG_PRINT("RX_ADDR_P0 : %.2x %.2x %.2x %.2x %.2x.\r\n",
                rx0_addr[0],
                rx0_addr[1],
                rx0_addr[2],
                rx0_addr[3],
                rx0_addr[4]
                );

    read_register_blocking(&unittest_hndl, NRF24L01_REG_TX_ADDR, tx_addr, 5);
    DEBUG_PRINT("TX_ADDR : %.2x %.2x %.2x %.2x %.2x.\r\n",
                tx_addr[0],
                tx_addr[1],
                tx_addr[2],
                tx_addr[3],
                tx_addr[4]
                );

    while (1) {
        result = nrf24l01_send_blocking(&unittest_hndl, tx_data, 32);
    
        observe = nrf24l01_get_observe_tx(&unittest_hndl);
        
        if (result == NRF24L01_SUCCESS) {
            DEBUG_PRINT("Sending success. \r\n");
        } else {
            DEBUG_PRINT("Sending fail. \r\n");
        }
        
        DEBUG_PRINT("LOST : %d, RETR : %d\r\n\r\n", observe.lost_count, observe.retr_count);
        
        hal_system_delay(100);
    }
}

/**
 * @brief Port function.
 */
static void UNITTEST_NRF24L01_SPI_READ_WRITE(nrf24l01_t *p_hndl,
                                             const uint8_t *p_w,
                                             uint8_t *p_r,
                                             uint16_t len)
{
    hal_spi_read_write_blocking(p_unittest_spi_hndl, p_w, p_r, len);
}

static void UNITTEST_NRF24L01_SPI_WRITE(nrf24l01_t *p_hndl,
                                        const uint8_t *p_w,
                                        uint16_t len)
{
    hal_spi_write_blocking(p_unittest_spi_hndl, p_w, len);
}

static void UNITTEST_NRF24L01_SPI_READ(nrf24l01_t *p_hndl,
                                       uint8_t *p_r,
                                       uint16_t len)
{
    hal_spi_read_blocking(p_unittest_spi_hndl, p_r, len);
}

static void UNITTEST_NRF24L01_CS_SELECT(nrf24l01_t *p_hndl)
{
    UNUSED(p_hndl);
    hal_gpio_output(CS_PIN, HAL_GPIO_LOW);
}

static void UNITTEST_NRF24L01_CS_DESELECT(nrf24l01_t *p_hndl)
{
    UNUSED(p_hndl);
    hal_gpio_output(CS_PIN, HAL_GPIO_HIGH);
}

static void UNITTEST_NRF24L01_CE_SET(nrf24l01_t *p_hndl)
{
    UNUSED(p_hndl);
    hal_gpio_output(CE_PIN, HAL_GPIO_HIGH);
}

static void UNITTEST_NRF24L01_CE_UNSET(nrf24l01_t *p_hndl)
{
    UNUSED(p_hndl);
    hal_gpio_output(CE_PIN, HAL_GPIO_LOW);
}

static void UNITTEST_NRF24L01_CE_PULSE(nrf24l01_t *p_hndl)
{
    volatile uint32_t i = 0;

    hal_gpio_output(CE_PIN, HAL_GPIO_HIGH);
    
    for (i=0; i<100; i++) {
    }
    
    hal_gpio_output(CE_PIN, HAL_GPIO_LOW);
}

#endif /* UNITTEST_ENABLE */
