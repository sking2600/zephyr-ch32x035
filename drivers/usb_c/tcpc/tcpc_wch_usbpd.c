/*
 * Copyright (c) 2024 Nanjing Qinheng Microelectronics Co., Ltd.
 * Copyright (c) 2026 Scott King
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT wch_ch32_usbpd

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(tcpc_wch_usbpd, CONFIG_USBC_LOG_LEVEL);

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/usb_c/usbc_tcpc.h>
#include <soc.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/drivers/clock_control.h>

/* Defines for register access compatibility */
#ifndef USBPD
#define USBPD ((USBPD_TypeDef *)DT_INST_REG_ADDR(0))
#endif

#define PD_HEADER_CNT(header)  (((header) >> 12) & 0x7)

struct tcpc_wch_config {
    uint32_t base;
    const struct pinctrl_dev_config *pcfg;
    const struct device *clock_dev;
    uint32_t pclk_id; 
};


struct tcpc_wch_data {
    /* Callback for alerts */
    tcpc_alert_handler_cb_t alert_cb;
    void *alert_data;

    enum tc_rp_value rp_val;
    uint8_t __aligned(4) tx_buf[256];
    uint8_t __aligned(4) rx_buf[256];
};

static void tcpc_wch_isr(const struct device *dev);

static int tcpc_wch_init(const struct device *dev)
{
    const struct tcpc_wch_config *cfg = dev->config;
    int ret;

    LOG_INF("Initializing WCH TCPC");

    /* Configure pins */
    ret = pinctrl_apply_state(cfg->pcfg, PINCTRL_STATE_DEFAULT);
    if (ret < 0) {
        LOG_ERR("Failed to configure TCPC pins");
        return ret;
    }

    /* Enable peripheral clock */
    if (!device_is_ready(cfg->clock_dev)) {
        LOG_ERR("Clock control device not ready");
        return -ENODEV;
    }

    ret = clock_control_on(cfg->clock_dev, (clock_control_subsys_t)(uintptr_t)cfg->pclk_id);
    if (ret < 0) {
        LOG_ERR("Could not enable TCPC clock");
        return ret;
    }

    /* Reset the PD peripheral */
    USBPD_TypeDef *pd = (USBPD_TypeDef *)cfg->base;
    pd->USBPD_CONFIG = PD_ALL_CLR | PD_RST_EN;
    pd->USBPD_CONFIG &= ~PD_RST_EN;

    /* Configure DMA */
    /* Point to RX buffer by default */
    struct tcpc_wch_data *data = dev->data;
    pd->USBPD_DMA = (uint32_t)data->rx_buf;

    /* Connect IRQ */
    IRQ_CONNECT(DT_INST_IRQN(0), 0,
                tcpc_wch_isr, DEVICE_DT_INST_GET(0), 0);
    irq_enable(DT_INST_IRQN(0));

    return 0;
}



static enum tc_cc_voltage_state tcpc_wch_measure_cc(USBPD_TypeDef *pd, int cc_idx, uint16_t port_val)
{
    volatile uint16_t *port_reg = (cc_idx == 1) ? (uint16_t *)&pd->PORT_CC1 : (uint16_t *)&pd->PORT_CC2;
    uint16_t base = port_val & ~(CC_CE);
    bool is_rd = (base & CC_PD); /* We are Sink */
    // bool is_rp = (base & CC_PU_Mask); /* We are Source */

    /* If disconnected (OPEN), returns OPEN */
    /* Implementation based on CMP thresholds */
    
    if (is_rd) {
        /* WE ARE SINK (Rd asserted) */
        /* Check > 0.22V (vSafe5V / Connected) */
        *port_reg = base | CC_CMP_22;
        k_busy_wait(5);
        if (!(*port_reg & PA_CC_AI)) {
            return TC_CC_VOLT_OPEN;
        }

        /* Check > 0.66V (1.5A) */
        *port_reg = base | CC_CMP_66;
        k_busy_wait(5);
        if (!(*port_reg & PA_CC_AI)) {
            return TC_CC_VOLT_RP_DEF;
        }

        /* Check > 1.23V (3.0A) */
        *port_reg = base | CC_CMP_123;
        k_busy_wait(5);
        if (!(*port_reg & PA_CC_AI)) {
            return TC_CC_VOLT_RP_1A5;
        }
        
        return TC_CC_VOLT_RP_3A0;

    } else {
        /* WE ARE SOURCE (Rp asserted) or Open */
        /* Check < 0.22V (Ra) */
        *port_reg = base | CC_CMP_22;
        k_busy_wait(5);
        if (!(*port_reg & PA_CC_AI)) {
            return TC_CC_VOLT_RA;
        }

        /* Check Open vs Rd */
        /* Ideally Open is > ~1.6V (depends on Rp). */
        *port_reg = base | CC_CMP_123; /* Max threshold */
        k_busy_wait(5);
        
        /* If voltage > 1.23V, it COUlD be Open or 3A Rd. 
           Assuming 3A Rp is not used or distinguishing via other means. 
           For Default/1.5A, Rd < 1.23V. */
        if (*port_reg & PA_CC_AI) {
            return TC_CC_VOLT_OPEN;
        }

        return TC_CC_VOLT_RD;
    }
}

static int tcpc_wch_get_cc(const struct device *dev, enum tc_cc_voltage_state *cc1,
                                enum tc_cc_voltage_state *cc2)
{
    const struct tcpc_wch_config *cfg = dev->config;
    USBPD_TypeDef *pd = (USBPD_TypeDef *)cfg->base;
    uint16_t p1 = pd->PORT_CC1;
    uint16_t p2 = pd->PORT_CC2;

    *cc1 = tcpc_wch_measure_cc(pd, 1, p1);
    *cc2 = tcpc_wch_measure_cc(pd, 2, p2);
    
    /* Restore settings (disable CMP to save power or default state?) */
    pd->PORT_CC1 = p1;
    pd->PORT_CC2 = p2;

    return 0;
}

static int tcpc_wch_select_rp_value(const struct device *dev, enum tc_rp_value rp)
{
    struct tcpc_wch_data *data = dev->data;

    data->rp_val = rp;
    return 0;
}

static int tcpc_wch_get_rp_value(const struct device *dev, enum tc_rp_value *rp)
{
    struct tcpc_wch_data *data = dev->data;

    *rp = data->rp_val;
    return 0;
}

static int tcpc_wch_set_cc(const struct device *dev, enum tc_cc_pull pull)
{
    const struct tcpc_wch_config *cfg = dev->config;
    struct tcpc_wch_data *data = dev->data;
    USBPD_TypeDef *pd = (USBPD_TypeDef *)cfg->base;
    uint16_t cc_cfg = 0;

    switch (pull) {
    case TC_CC_RA:
    case TC_CC_RD:
        cc_cfg = CC_PD;
        break;
    case TC_CC_RP:
        switch (data->rp_val) {
        case TC_RP_USB:
            cc_cfg = CC_PU_80;
            break;
        case TC_RP_1A5:
            cc_cfg = CC_PU_180;
            break;
        case TC_RP_3A0:
            cc_cfg = CC_PU_330;
            break;
        default:
            cc_cfg = CC_PU_80;
            break;
        }
        break;
    case TC_CC_OPEN:
        cc_cfg = 0;
        break;
    default:
        return -EINVAL;
    }

    /* Apply to both CC1 and CC2 */
    
    pd->PORT_CC1 = (pd->PORT_CC1 & ~(CC_PD | CC_PU_Mask)) | cc_cfg;
    pd->PORT_CC2 = (pd->PORT_CC2 & ~(CC_PD | CC_PU_Mask)) | cc_cfg;

    return 0;
}

static int tcpc_wch_set_vconn(const struct device *dev, bool enable)
{
    return -ENOTSUP;
}

static int tcpc_wch_set_vconn_cb(const struct device *dev, tcpc_vconn_control_cb_t vconn_cb)
{
    return -ENOTSUP;
}

static int tcpc_wch_vconn_discharge(const struct device *dev, bool enable)
{
    return -ENOTSUP;
}

static int tcpc_wch_set_vconn_discharge_cb(const struct device *dev, tcpc_vconn_discharge_cb_t cb)
{
    return -ENOTSUP;
}

static int tcpc_wch_set_roles(const struct device *dev, enum tc_power_role power_role,
                                   enum tc_data_role data_role)
{
    return 0;
}

static int tcpc_wch_get_rx_pending_msg(const struct device *dev, struct pd_msg *msg)
{
    struct tcpc_wch_data *data = dev->data;
    /* Determine length from cached or register? */
    /* The ISR should have read the length. Use the buffer content. */
    /* Header is first 2 bytes */
    uint16_t header = *(uint16_t *)data->rx_buf;
    memcpy(&msg->header, &header, 2);
    msg->len = PD_HEADER_CNT(header) * 4; 
    
    memcpy(msg->data, &data->rx_buf[2], msg->len);
    msg->type = PD_PACKET_SOP; // Configuring specifically for SOP for now. 
    
    return 0;
}

static int tcpc_wch_set_rx_enable(const struct device *dev, bool enable)
{
    const struct tcpc_wch_config *cfg = dev->config;
    USBPD_TypeDef *pd = (USBPD_TypeDef *)cfg->base;

    if (enable) {
        /* Enable RX interrupts */
        pd->USBPD_CONFIG |= IE_RX_ACT | IE_RX_RESET | IE_PD_IO;
    } else {
        pd->USBPD_CONFIG &= ~(IE_RX_ACT | IE_RX_RESET | IE_PD_IO);
    }
    return 0;
}

static int tcpc_wch_transmit_data(const struct device *dev, struct pd_msg *msg)
{
    const struct tcpc_wch_config *cfg = dev->config;
    struct tcpc_wch_data *data = dev->data;
    USBPD_TypeDef *pd = (USBPD_TypeDef *)cfg->base;

    /* Copy msg to tx_buf */
    /* Format: Header (2 bytes) + Data Objects */
    memcpy(data->tx_buf, &msg->header, 2);
    memcpy(&data->tx_buf[2], msg->data, msg->len);
    
    uint16_t total_len = 2 + msg->len;

    /* Set DMA to TX buffer */
    pd->USBPD_DMA = (uint32_t)data->tx_buf;
    pd->BMC_TX_SZ = total_len;

    /* Enable TX */
    pd->USBPD_CONTROL |= PD_TX_EN; // Enable TX mode
    pd->USBPD_CONFIG |= IE_TX_END; // Enable TX complete interrupt
    pd->USBPD_CONTROL |= BMC_START;

    return 0;
}


static int tcpc_wch_set_cc_polarity(const struct device *dev, enum tc_cc_polarity polarity)
{
    const struct tcpc_wch_config *cfg = dev->config;
    USBPD_TypeDef *pd = (USBPD_TypeDef *)cfg->base;

    if (polarity == TC_POLARITY_CC1) {
        /* CC1 is the CC line */
        pd->USBPD_CONFIG &= ~CC_SEL; 
    } else {
        /* CC2 is the CC line */
        pd->USBPD_CONFIG |= CC_SEL;
    }

    return 0;
}

static void tcpc_wch_isr(const struct device *dev)
{
    const struct tcpc_wch_config *cfg = dev->config;
    struct tcpc_wch_data *data = dev->data;
    USBPD_TypeDef *pd = (USBPD_TypeDef *)cfg->base;
    uint16_t status = pd->USBPD_STATUS;

    /* Clear interrupts handled */

    if (status & IF_TX_END) {
        /* TX Complete */
        pd->USBPD_CONTROL &= ~PD_TX_EN; // Switch back to RX?
        pd->USBPD_DMA = (uint32_t)data->rx_buf; // Reset DMA to RX buffer
        pd->USBPD_STATUS = IF_TX_END; // Clear flag
        
        if (data->alert_cb) {
            data->alert_cb(dev, data->alert_data, TCPC_ALERT_TRANSMIT_MSG_SUCCESS);
        }
    }

    if (status & IF_RX_ACT) {
        /* RX Complete */
        pd->USBPD_STATUS = IF_RX_ACT; // Clear flag
        /* Check for errors? BUF_ERR? */
        
        if (data->alert_cb) {
             /* Assuming SOP for now */
            data->alert_cb(dev, data->alert_data, TCPC_ALERT_MSG_STATUS);
        }
    }
    
    if (status & IF_RX_RESET) {
        pd->USBPD_STATUS = IF_RX_RESET;
        if (data->alert_cb) {
             data->alert_cb(dev, data->alert_data, TCPC_ALERT_HARD_RESET_RECEIVED);
        }
    }
}


static int tcpc_wch_dump_std_reg(const struct device *dev)
{
    const struct tcpc_wch_config *cfg = dev->config;
    USBPD_TypeDef *pd = (USBPD_TypeDef *)cfg->base;

    LOG_INF("TCPC Registers:");
    LOG_INF("CONFIG:  %04x", pd->USBPD_CONFIG);
    LOG_INF("CONTROL: %02x", pd->USBPD_CONTROL);
    LOG_INF("STATUS:  %02x", pd->USBPD_STATUS);
    LOG_INF("CC1:     %04x", pd->PORT_CC1);
    LOG_INF("CC2:     %04x", pd->PORT_CC2);
    LOG_INF("DMA:     %04x", pd->USBPD_DMA);
    return 0;
}

static int tcpc_wch_set_alert_handler_cb(const struct device *dev,
                                              tcpc_alert_handler_cb_t handler,
                                              void *data)
{
    struct tcpc_wch_data *data_ptr = dev->data;
    data_ptr->alert_cb = handler;
    data_ptr->alert_data = data;
    return 0;
}

static const struct tcpc_driver_api tcpc_wch_driver_api = {
    .init = tcpc_wch_init,
    .get_cc = tcpc_wch_get_cc,
    .select_rp_value = tcpc_wch_select_rp_value,
    .get_rp_value = tcpc_wch_get_rp_value,
    .set_cc = tcpc_wch_set_cc,
    .set_vconn = tcpc_wch_set_vconn,
    .set_vconn_cb = (void *)tcpc_wch_set_vconn_cb,
    .vconn_discharge = tcpc_wch_vconn_discharge,
    .set_vconn_discharge_cb = (void *)tcpc_wch_set_vconn_discharge_cb,
    .set_roles = tcpc_wch_set_roles,
    .get_rx_pending_msg = tcpc_wch_get_rx_pending_msg,
    .set_rx_enable = tcpc_wch_set_rx_enable,
    .set_cc_polarity = tcpc_wch_set_cc_polarity,
    .transmit_data = tcpc_wch_transmit_data,
    .dump_std_reg = tcpc_wch_dump_std_reg,
    .set_alert_handler_cb = tcpc_wch_set_alert_handler_cb,
};


PINCTRL_DT_INST_DEFINE(0);

static const struct tcpc_wch_config tcpc_wch_cfg = {
    .base = DT_INST_REG_ADDR(0),
    .pcfg = PINCTRL_DT_INST_DEV_CONFIG_GET(0),
    .clock_dev = DEVICE_DT_GET(DT_INST_CLOCKS_CTLR(0)),
    .pclk_id = DT_INST_CLOCKS_CELL_BY_IDX(0, 0, id),
};

static struct tcpc_wch_data tcpc_wch_data;

DEVICE_DT_INST_DEFINE(0, &tcpc_wch_init, NULL,
                      &tcpc_wch_data, &tcpc_wch_cfg,
                      POST_KERNEL, CONFIG_USBC_TCPC_INIT_PRIORITY,
                      &tcpc_wch_driver_api);

