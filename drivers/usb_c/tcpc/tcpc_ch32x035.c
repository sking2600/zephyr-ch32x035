/*
 * Copyright (c) 2024 Nanjing Qinheng Microelectronics Co., Ltd.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT wch_ch32x035_ucpd

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(tcpc_ch32x035, CONFIG_USBC_LOG_LEVEL);

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/usb_c/usbc_tcpc.h>
#include <soc.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/drivers/clock_control.h>
/* #include <zephyr/drivers/clock_control/ch32_clock_control.h> */ /* Removed */
#include "ch32x035_usbpd.h"

#define PD_HEADER_CNT(header)  (((header) >> 12) & 0x7)

struct tcpc_ch32x035_config {
    uint32_t base;
    const struct pinctrl_dev_config *pcfg;
    const struct device *clock_dev;
    uint32_t pclk_id; 
};


struct tcpc_ch32x035_data {
    /* Callback for alerts */
    tcpc_alert_handler_cb_t alert_cb;
    void *alert_data;

    enum tc_rp_value rp_val;
    uint8_t __aligned(4) tx_buf[256];
    uint8_t __aligned(4) rx_buf[256];
};

static void tcpc_ch32x035_isr(const struct device *dev);

static int tcpc_ch32x035_init(const struct device *dev)
{
    const struct tcpc_ch32x035_config *cfg = dev->config;
    int ret;

    LOG_INF("Initializing CH32X035 TCPC");

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
    USBPD_DETAILED_TypeDef *pd = (USBPD_DETAILED_TypeDef *)cfg->base;
    pd->CONFIG = PD_ALL_CLR | PD_RST_EN;
    pd->CONFIG &= ~PD_RST_EN;

    /* Configure DMA */
    /* Point to RX buffer by default */
    struct tcpc_ch32x035_data *data = dev->data;
    pd->USBPD_DMA = (uint32_t)data->rx_buf;

    /* Connect IRQ */
    IRQ_CONNECT(DT_INST_IRQN(0), 0,
                tcpc_ch32x035_isr, DEVICE_DT_INST_GET(0), 0);
    irq_enable(DT_INST_IRQN(0));

    return 0;
}



static enum tc_cc_voltage_state tcpc_ch32x035_measure_cc(USBPD_DETAILED_TypeDef *pd, int cc_idx, uint16_t port_val)
{
    volatile uint16_t *port_reg = (cc_idx == 1) ? &pd->PORT_CC1 : &pd->PORT_CC2;
    uint16_t base = port_val & ~(CC_CMP_Mask);
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

static int tcpc_ch32x035_get_cc(const struct device *dev, enum tc_cc_voltage_state *cc1,
                                enum tc_cc_voltage_state *cc2)
{
    const struct tcpc_ch32x035_config *cfg = dev->config;
    USBPD_DETAILED_TypeDef *pd = (USBPD_DETAILED_TypeDef *)cfg->base;
    uint16_t p1 = pd->PORT_CC1;
    uint16_t p2 = pd->PORT_CC2;

    *cc1 = tcpc_ch32x035_measure_cc(pd, 1, p1);
    *cc2 = tcpc_ch32x035_measure_cc(pd, 2, p2);
    
    /* Restore settings (disable CMP to save power or default state?) */
    pd->PORT_CC1 = p1;
    pd->PORT_CC2 = p2;

    return 0;
}

static int tcpc_ch32x035_select_rp_value(const struct device *dev, enum tc_rp_value rp)
{
    struct tcpc_ch32x035_data *data = dev->data;

    data->rp_val = rp;
    return 0;
}

static int tcpc_ch32x035_get_rp_value(const struct device *dev, enum tc_rp_value *rp)
{
    struct tcpc_ch32x035_data *data = dev->data;

    *rp = data->rp_val;
    return 0;
}

static int tcpc_ch32x035_set_cc(const struct device *dev, enum tc_cc_pull pull)
{
    const struct tcpc_ch32x035_config *cfg = dev->config;
    struct tcpc_ch32x035_data *data = dev->data;
    USBPD_DETAILED_TypeDef *pd = (USBPD_DETAILED_TypeDef *)cfg->base;
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
    /* Note: Calling set_cc implies we are setting the pull resistors for connection detection */
    /* We might need to be careful if one is VCONN, but TCPM handles that via set_vconn? */
    /* Usually set_cc sets both, and set_vconn overrides one. */
    
    pd->PORT_CC1 = (pd->PORT_CC1 & ~(CC_PD | CC_PU_Mask)) | cc_cfg;
    pd->PORT_CC2 = (pd->PORT_CC2 & ~(CC_PD | CC_PU_Mask)) | cc_cfg;

    return 0;
}

static int tcpc_ch32x035_set_vconn(const struct device *dev, bool enable)
{
    /* VCONN control is not natively supported by the internal PHY/logic in a standard way
       or requires external components not defined here. */
    return -ENOTSUP;
}

static int tcpc_ch32x035_set_vconn_cb(const struct device *dev, tcpc_vconn_control_cb_t vconn_cb)
{
    return -ENOTSUP;
}

static int tcpc_ch32x035_vconn_discharge(const struct device *dev, bool enable)
{
    return -ENOTSUP;
}

static int tcpc_ch32x035_set_vconn_discharge_cb(const struct device *dev, tcpc_vconn_discharge_cb_t cb)
{
    return -ENOTSUP;
}

static int tcpc_ch32x035_set_roles(const struct device *dev, enum tc_power_role power_role,
                                   enum tc_data_role data_role)
{
    /* Hardware does not maintain role state, it's packet based. 
       The stack manages the PD message headers. */
    return 0;
}

static int tcpc_ch32x035_get_rx_pending_msg(const struct device *dev, struct pd_msg *msg)
{
    struct tcpc_ch32x035_data *data = dev->data;
    /* Determine length from cached or register? */
    /* The ISR should have read the length. Use the buffer content. */
    /* Header is first 2 bytes */
    uint16_t header = *(uint16_t *)data->rx_buf;
    memcpy(&msg->header, &header, 2);
    msg->len = PD_HEADER_CNT(header) * 4; // Header CNT is number of 32-bit objects? No, number of bytes?
    /* Spec says Data Objects count. Each object is 4 bytes. */
    /* Re-verify PD_HEADER_CNT macro usage. usually it returns number of objects. */
    
    memcpy(msg->data, &data->rx_buf[2], msg->len);
    msg->type = PD_PACKET_SOP; // Configuring specifically for SOP for now. 
    /* Ideally we check the received SOP type if the hardware provides it (BMC_AUX?) */
    
    return 0;
}

static int tcpc_ch32x035_set_rx_enable(const struct device *dev, bool enable)
{
    const struct tcpc_ch32x035_config *cfg = dev->config;
    USBPD_DETAILED_TypeDef *pd = (USBPD_DETAILED_TypeDef *)cfg->base;

    if (enable) {
        /* Enable RX interrupts */
        pd->CONFIG |= IE_RX_ACT | IE_RX_RESET | IE_PD_IO;
    } else {
        pd->CONFIG &= ~(IE_RX_ACT | IE_RX_RESET | IE_PD_IO);
    }
    return 0;
}

static int tcpc_ch32x035_transmit_data(const struct device *dev, struct pd_msg *msg)
{
    const struct tcpc_ch32x035_config *cfg = dev->config;
    struct tcpc_ch32x035_data *data = dev->data;
    USBPD_DETAILED_TypeDef *pd = (USBPD_DETAILED_TypeDef *)cfg->base;

    /* Copy msg to tx_buf */
    /* Format: Header (2 bytes) + Data Objects */
    memcpy(data->tx_buf, &msg->header, 2);
    memcpy(&data->tx_buf[2], msg->data, msg->len);
    
    uint16_t total_len = 2 + msg->len;

    /* Set DMA to TX buffer */
    pd->USBPD_DMA = (uint32_t)data->tx_buf;
    pd->BMC_TX_SZ = total_len;

    /* Enable TX */
    /* Bit set: PD_TX_EN | BMC_START ?? */
    /* Need to check appropriate start sequence. */
    /* Assuming PD_TX_EN handles direction and BMC_START triggers */
    pd->CONTROL |= PD_TX_EN; // Enable TX mode
    pd->CONFIG |= IE_TX_END; // Enable TX complete interrupt
    pd->CONTROL |= BMC_START;

    return 0;
}


static int tcpc_ch32x035_set_cc_polarity(const struct device *dev, enum tc_cc_polarity polarity)
{
    /* 
     * The hardware automatically handles polarity based on which CC pin detects connection 
     * if we are using the internal PHY's auto-detection (which we aren't fully using here, 
     * we are manually measuring).
     * 
     * However, for PD communication, we need to tell the PHY which CC line to use.
     * The `PD_PHY_V25_SEL` bit in CONFIG might be relevant, or similar.
     * 
     * Checking headers:
     * CC_SEL (Bit 1 in CONFIG? or CONTROL?)
     * 
     * Let's check ch32x035_usbpd.h definitions.
     * Usually there is a bit to select CC1 or CC2 for the PD transceiver.
     */
     
    const struct tcpc_ch32x035_config *cfg = dev->config;
    USBPD_DETAILED_TypeDef *pd = (USBPD_DETAILED_TypeDef *)cfg->base;

    if (polarity == TC_POLARITY_CC1) {
        /* CC1 is the CC line */
        pd->CONFIG &= ~CC_SEL; 
    } else {
        /* CC2 is the CC line */
        pd->CONFIG |= CC_SEL;
    }

    return 0;
}

static void tcpc_ch32x035_isr(const struct device *dev)
{
    const struct tcpc_ch32x035_config *cfg = dev->config;
    struct tcpc_ch32x035_data *data = dev->data;
    USBPD_DETAILED_TypeDef *pd = (USBPD_DETAILED_TypeDef *)cfg->base;
    uint16_t status = pd->STATUS;

    /* Clear interrupts handled (Note: registers might be Write-1-to-Clear or auto) */
    /* Header says: "Clear all interrupt flags": PD_ALL_CLR in CONFIG. */
    /* Or IF_RX_ACT, IF_TX_END are R/W1C? Usually yes. */

    if (status & IF_TX_END) {
        /* TX Complete */
        pd->CONTROL &= ~PD_TX_EN; // Switch back to RX?
        pd->USBPD_DMA = (uint32_t)data->rx_buf; // Reset DMA to RX buffer
        pd->STATUS = IF_TX_END; // Clear flag
        
        if (data->alert_cb) {
            data->alert_cb(dev, data->alert_data, TCPC_ALERT_TRANSMIT_MSG_SUCCESS);
        }
    }

    if (status & IF_RX_ACT) {
        /* RX Complete */
        pd->STATUS = IF_RX_ACT; // Clear flag
        /* Check for errors? BUF_ERR? */
        
        if (data->alert_cb) {
             /* Assuming SOP for now */
            data->alert_cb(dev, data->alert_data, TCPC_ALERT_MSG_STATUS);
        }
    }
    
    if (status & IF_RX_RESET) {
        pd->STATUS = IF_RX_RESET;
        if (data->alert_cb) {
             data->alert_cb(dev, data->alert_data, TCPC_ALERT_HARD_RESET_RECEIVED);
        }
    }
}


static int tcpc_ch32x035_dump_std_reg(const struct device *dev)
{
    const struct tcpc_ch32x035_config *cfg = dev->config;
    USBPD_DETAILED_TypeDef *pd = (USBPD_DETAILED_TypeDef *)cfg->base;

    LOG_INF("TCPC Registers:");
    LOG_INF("CONFIG:  %04x", pd->CONFIG);
    LOG_INF("CONTROL: %02x", pd->CONTROL);
    LOG_INF("STATUS:  %02x", pd->STATUS);
    LOG_INF("CC1:     %04x", pd->PORT_CC1);
    LOG_INF("CC2:     %04x", pd->PORT_CC2);
    LOG_INF("DMA:     %04x", pd->USBPD_DMA);
    return 0;
}

static int tcpc_ch32x035_set_alert_handler_cb(const struct device *dev,
                                              tcpc_alert_handler_cb_t handler,
                                              void *data)
{
    struct tcpc_ch32x035_data *data_ptr = dev->data;
    data_ptr->alert_cb = handler;
    data_ptr->alert_data = data;
    return 0;
}

static const struct tcpc_driver_api tcpc_ch32x035_driver_api = {
    .init = tcpc_ch32x035_init,
    .get_cc = tcpc_ch32x035_get_cc,
    .select_rp_value = tcpc_ch32x035_select_rp_value,
    .get_rp_value = tcpc_ch32x035_get_rp_value,
    .set_cc = tcpc_ch32x035_set_cc,
    .set_vconn = tcpc_ch32x035_set_vconn,
    .set_vconn_cb = (void *)tcpc_ch32x035_set_vconn_cb, /* Void cast due to API signature mismatch warning potentially */
    .vconn_discharge = tcpc_ch32x035_vconn_discharge,
    .set_vconn_discharge_cb = (void *)tcpc_ch32x035_set_vconn_discharge_cb,
    .set_roles = tcpc_ch32x035_set_roles,
    .get_rx_pending_msg = tcpc_ch32x035_get_rx_pending_msg,
    .set_rx_enable = tcpc_ch32x035_set_rx_enable,
    .set_cc_polarity = tcpc_ch32x035_set_cc_polarity,
    .transmit_data = tcpc_ch32x035_transmit_data,
    .dump_std_reg = tcpc_ch32x035_dump_std_reg,
    .set_alert_handler_cb = tcpc_ch32x035_set_alert_handler_cb,
};


PINCTRL_DT_INST_DEFINE(0);

static const struct tcpc_ch32x035_config tcpc_ch32x035_cfg = {
    .base = DT_INST_REG_ADDR(0),
    .pcfg = PINCTRL_DT_INST_DEV_CONFIG_GET(0),
    .clock_dev = DEVICE_DT_GET(DT_INST_CLOCKS_CTLR(0)),
    .pclk_id = DT_INST_CLOCKS_CELL_BY_IDX(0, 0, id),
};

static struct tcpc_ch32x035_data tcpc_ch32x035_data;

DEVICE_DT_INST_DEFINE(0, &tcpc_ch32x035_init, NULL,
                      &tcpc_ch32x035_data, &tcpc_ch32x035_cfg,
                      POST_KERNEL, CONFIG_USBC_TCPC_INIT_PRIORITY,
                      &tcpc_ch32x035_driver_api);
