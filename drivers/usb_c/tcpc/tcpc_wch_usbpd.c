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

/* Register Access */
#ifndef USBPD
#define USBPD ((USBPD_TypeDef *)DT_INST_REG_ADDR(0))
#endif

/* Constants from ch32x035_usbpd.h */
#define UPD_TMR_TX_48M    (80-1)
#define UPD_TMR_RX_48M    (120-1)

/* USBPD->CONFIG */
#define PD_ALL_CLR          (1<<1)
#define CC_SEL              (1<<2)
#define PD_DMA_EN           (1<<3)
#define PD_RST_EN           (1<<4)
#define IE_PD_IO            (1<<10)
#define IE_RX_BIT           (1<<11)
#define IE_RX_BYTE          (1<<12)
#define IE_RX_ACT           (1<<13)
#define IE_RX_RESET         (1<<14)
#define IE_TX_END           (1<<15)

/* USBPD->CONTROL */
#define PD_TX_EN            (1<<0)
#define BMC_START           (1<<1)

/* USBPD->TX_SEL */
#define TX_SEL1_SYNC1       (0<<0)
#define TX_SEL2_SYNC1       (0<<2)
#define TX_SEL2_SYNC3       (1<<2)
#define TX_SEL3_SYNC1       (0<<4)
#define TX_SEL3_SYNC3       (1<<4)
#define TX_SEL4_SYNC2       (0<<6)
#define TX_SEL4_SYNC3       (1<<6)
#define TX_SEL4_RST2        (2<<6)

#define UPD_SOP0          ( TX_SEL1_SYNC1 | TX_SEL2_SYNC1 | TX_SEL3_SYNC1 | TX_SEL4_SYNC2 )

/* USBPD->STATUS */
#define BMC_AUX_INVALID     (0<<0)
#define IF_RX_ACT           (1<<5)
#define IF_RX_RESET         (1<<6)
#define IF_TX_END           (1<<7)

/* USBPD->PORT_CC */
#define PA_CC_AI            (1<<0)
#define CC_PD               (1<<1)
#define CC_PU_Mask          (3<<2)
#define CC_PU_80            (3<<2)
#define CC_PU_180           (2<<2)
#define CC_PU_330           (1<<2)
#define CC_LVE              (1<<4)
#define CC_CMP_Mask         (7<<5)
#define CC_CMP_22           (2<<5)
#define CC_CMP_66           (5<<5)
#define CC_CMP_123          (7<<5)

#define USBPD_IN_HVT        (1<<9)
#define USBPD_PHY_V33       (1<<8)

/* Message Types */
#define DEF_TYPE_GOODCRC           0x01

/* PD Header Construction Helpers */
#define PD_HEADER_SPEC_REV_2_0     (1 << 6)
#define PD_HEADER_DATA_ROLE_UFP    (0 << 5)
#define PD_HEADER_MSG_ID_MASK      0x0E

/* PD Status Masks */
#define PD_STATUS_SOP_MASK         0x03
#define PD_STATUS_SOP0_RECEIVED    0x01

#define PD_HEADER_CNT(header)  (((header) >> 12) & 0x7)
#define PD_HEADER_TYPE(header) ((header) & 0x1F)

struct tcpc_wch_config {
    uint32_t base;
    const struct pinctrl_dev_config *pcfg;
    const struct device *clock_dev;
    uint32_t pclk_id; 
};

struct tcpc_wch_data {
    tcpc_alert_handler_cb_t alert_cb;
    void *alert_data;

    enum tc_rp_value rp_val;
    uint8_t __aligned(4) tx_buf[256];
    uint8_t __aligned(4) rx_buf[256];
    uint8_t ack_buf[2]; 
};

static void tcpc_wch_isr(const struct device *dev);

static int tcpc_wch_init(const struct device *dev)
{
    const struct tcpc_wch_config *cfg = dev->config;
    struct tcpc_wch_data *data = dev->data;
    int ret;

    printk("Initializing WCH TCPC (Rev 2 - EXAM Based)\n");
    // LOG_INF("Initializing WCH TCPC (Rev 2 - EXAM Based)");

    /* 1. Clocks */
    if (!device_is_ready(cfg->clock_dev)) {
        LOG_ERR("Clock device not ready");
        return -ENODEV;
    }
    ret = clock_control_on(cfg->clock_dev, (clock_control_subsys_t)(uintptr_t)cfg->pclk_id);
    if (ret < 0) {
        LOG_ERR("Failed to enable TCPC clock");
        return ret;
    }

    /* Enable AFIO Clock - Manually via RCC for now as it's critical */
    /* RCC_APB2PCENR_AFIOEN is typically bit 0 in APB2PCENR */
    #define RCC_APB2ENR_AFIOEN (1 << 0)
    RCC->APB2PCENR |= RCC_APB2ENR_AFIOEN;

    /* 2. Pinctrl */
    ret = pinctrl_apply_state(cfg->pcfg, PINCTRL_STATE_DEFAULT);
    if (ret < 0) {
        LOG_ERR("Failed to configure Pinctrl");
        return ret;
    }

    /* 3. AFIO Configuration */
    /* CH32L103: USBPD_IN_HVT is in AFIO->CR[9] */
    AFIO->CR |= USBPD_IN_HVT;

    /* 4. USBPD Configuration */
    USBPD_TypeDef *pd = (USBPD_TypeDef *)cfg->base;
    
    pd->USBPD_CONFIG = PD_DMA_EN | PD_FILT_ED;
    pd->USBPD_STATUS = 0xFF; /* Clear all status */
    
    /* Setup DMA pointer to RX buffer */
    pd->USBPD_DMA = (uint32_t)data->rx_buf;
    
    /* Reset PHY */
    pd->USBPD_CONFIG |= PD_ALL_CLR | PD_RST_EN;
    pd->USBPD_CONFIG &= ~PD_RST_EN;
    
    /* Default to RX Mode */
    pd->USBPD_CONFIG |= IE_RX_ACT | IE_RX_RESET | PD_DMA_EN | PD_FILT_ED;
    pd->USBPD_CONTROL &= ~PD_TX_EN;
    pd->BMC_CLK_CNT = UPD_TMR_RX_48M;
    pd->USBPD_CONTROL |= BMC_START;

    IRQ_CONNECT(DT_INST_IRQN(0), 0, tcpc_wch_isr, DEVICE_DT_INST_GET(0), 0);
    irq_enable(DT_INST_IRQN(0));

    return 0;
}

static enum tc_cc_voltage_state tcpc_wch_measure_cc_sink(USBPD_TypeDef *pd, int cc_idx)
{
    volatile uint16_t *port_reg = (cc_idx == 1) ? (uint16_t *)&pd->PORT_CC1 : (uint16_t *)&pd->PORT_CC2;
    
    /* We assume we are in Sink mode (CC_PD set) */
    /* EXAM Logic:
       USBPD->PORT_CC1 &= ~( CC_CMP_Mask|PA_CC_AI );
       USBPD->PORT_CC1 |= CC_CMP_22;
       Delay_Us(2);
       if( USBPD->PORT_CC1 & PA_CC_AI ) -> Connected */

    /* Clear CMP mask and AI bit logic handled by HW/SW? 
       EXAM does: *port_reg &= ~(CC_CMP_Mask | PA_CC_AI); 
                  *port_reg |= CC_CMP_22; */
    
    uint16_t base = *port_reg;
    base &= ~(CC_CMP_Mask | PA_CC_AI);
    *port_reg = base | CC_CMP_22;
    
    k_busy_wait(5); /* Delay > 2us */
    
    if (*port_reg & PA_CC_AI) {
        /* Voltage > 0.22V */
        
        /* Check 0.66V */
        *port_reg = base | CC_CMP_66;
        k_busy_wait(5);
        if (*port_reg & PA_CC_AI) {
            /* Voltage > 0.66V */
            
             /* Check 1.23V */
            *port_reg = base | CC_CMP_123;
            k_busy_wait(5);
             if (*port_reg & PA_CC_AI) {
                 return TC_CC_VOLT_RP_3A0;
             }
             return TC_CC_VOLT_RP_1A5;
        }
        return TC_CC_VOLT_RP_DEF; 
    }

    return TC_CC_VOLT_OPEN;
}

static int tcpc_wch_get_cc(const struct device *dev, enum tc_cc_voltage_state *cc1,
                                enum tc_cc_voltage_state *cc2)
{
    const struct tcpc_wch_config *cfg = dev->config;
    USBPD_TypeDef *pd = (USBPD_TypeDef *)cfg->base;
    
    *cc1 = tcpc_wch_measure_cc_sink(pd, 1);
    *cc2 = tcpc_wch_measure_cc_sink(pd, 2);

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
    USBPD_TypeDef *pd = (USBPD_TypeDef *)cfg->base;
    uint16_t cc_cfg = 0;

    switch (pull) {
    case TC_CC_RA: /* Not fully supported in simplistic sink init */
    case TC_CC_RD:
        cc_cfg = CC_PD | CC_CMP_22; /* Set Rd and default CMP level */
        break;
    case TC_CC_OPEN:
        cc_cfg = 0;
        break;
    default:
        /* Source mode not fully implemented in this overhaul yet */
        return -ENOTSUP;
    }

    /* Update both CC lines preserving other bits if necessary? 
       EXAM just overwrites or sets bits.
       PD_SINK_Init: PORT_CC1 = CC_CMP_66 | CC_PD;
    */
    
    pd->PORT_CC1 = cc_cfg;
    pd->PORT_CC2 = cc_cfg;

    return 0;
}

static int tcpc_wch_set_rx_enable(const struct device *dev, bool enable)
{
    const struct tcpc_wch_config *cfg = dev->config;
    struct tcpc_wch_data *data = dev->data;
    USBPD_TypeDef *pd = (USBPD_TypeDef *)cfg->base;

    if (enable) {
        pd->USBPD_CONFIG |= PD_ALL_CLR;
        pd->USBPD_CONFIG &= ~PD_ALL_CLR;
        pd->USBPD_CONFIG |= IE_RX_ACT | IE_RX_RESET | PD_DMA_EN | PD_FILT_ED;
        pd->USBPD_DMA = (uint32_t)data->rx_buf;
        pd->USBPD_CONTROL &= ~PD_TX_EN;
        pd->BMC_CLK_CNT = UPD_TMR_RX_48M;
        pd->USBPD_CONTROL |= BMC_START;
    } else {
        pd->USBPD_CONTROL &= ~BMC_START;
    }
    return 0;
}


static int tcpc_wch_get_rx_pending_msg(const struct device *dev, struct pd_msg *msg)
{
    struct tcpc_wch_data *data = dev->data;
    /* Bytes 0,1 are header in little endian */
    uint16_t header = data->rx_buf[0] | (data->rx_buf[1] << 8);
    
    msg->header.raw_value = header;
    msg->len = PD_HEADER_CNT(header) * 4;
    msg->type = PD_HEADER_TYPE(header);
    
    if (msg->len > 0) {
        memcpy(msg->data, &data->rx_buf[2], msg->len);
    }
    return 0;
}

static int tcpc_wch_transmit_data(const struct device *dev, struct pd_msg *msg)
{
    const struct tcpc_wch_config *cfg = dev->config;
    struct tcpc_wch_data *data = dev->data;
    USBPD_TypeDef *pd = (USBPD_TypeDef *)cfg->base;

    /* Prepare Data */
    data->tx_buf[0] = msg->header.raw_value & 0xFF;
    data->tx_buf[1] = (msg->header.raw_value >> 8) & 0xFF;
    if (msg->len > 0) {
        memcpy(&data->tx_buf[2], msg->data, msg->len);
    }

    uint16_t total_len = 2 + msg->len;

    /* Setup DMA */
    pd->USBPD_DMA = (uint32_t)data->tx_buf;
    pd->TX_SEL = UPD_SOP0; /* Assuming SOP0 for now */
    pd->BMC_TX_SZ = total_len;
    pd->BMC_CLK_CNT = UPD_TMR_TX_48M;

    if ((pd->USBPD_CONFIG & CC_SEL) == CC_SEL) {
        pd->PORT_CC2 |= CC_LVE;
    } else {
        pd->PORT_CC1 |= CC_LVE;
    }

    /* Enable TX */
    pd->USBPD_CONTROL |= PD_TX_EN;
    pd->USBPD_STATUS &= BMC_AUX_INVALID; /* Clear AUX */
    pd->USBPD_CONFIG |= IE_TX_END;
    pd->USBPD_CONTROL |= BMC_START;

    return 0;
}

static int tcpc_wch_set_cc_polarity(const struct device *dev, enum tc_cc_polarity polarity)
{
    const struct tcpc_wch_config *cfg = dev->config;
    USBPD_TypeDef *pd = (USBPD_TypeDef *)cfg->base;

    if (polarity == TC_POLARITY_CC1) {
        pd->USBPD_CONFIG &= ~CC_SEL; 
    } else {
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

    // printk("ISR: %04x\n", status); /* Debugging */

    /* Handle RX Activity */
    if (status & IF_RX_ACT) {
        pd->USBPD_STATUS = IF_RX_ACT; /* Clear flag */
        
        /* Check if it's a SOP0 packet (SOP0=0x01 in STATUS[1:0] ?) */
        /* EXAM checks: ( USBPD->STATUS & MASK_PD_STAT ) == PD_RX_SOP0
           MASK_PD_STAT = 0x03, PD_RX_SOP0 = 0x01 */
        
        /* Check if it's a SOP0 packet */
        if ((status & PD_STATUS_SOP_MASK) == PD_STATUS_SOP0_RECEIVED) {
             
             uint8_t rx_len = pd->BMC_BYTE_CNT;
             uint8_t msg_type = data->rx_buf[0] & 0x1F;
             
             if (rx_len != 6 || msg_type != DEF_TYPE_GOODCRC) {
                 /* We need to send GoodCRC */
                 k_busy_wait(30); /* EXAM delay 30us */
                 
                 /* Prepare GoodCRC */
                 /* Header: Type = GoodCRC(1), SpecRev=2.0, Role=UFP */
                 data->ack_buf[0] = DEF_TYPE_GOODCRC | PD_HEADER_SPEC_REV_2_0 | PD_HEADER_DATA_ROLE_UFP;
                 
                 /* Byte 1: MessageID from received msg + PRRole (Sink=0) */
                 data->ack_buf[1] = (data->rx_buf[1] & PD_HEADER_MSG_ID_MASK); /* Copy MsgID */
                 
                 /* Send ACK */
                 /* We need to use PD_Phy_SendPack equivalent without blocking strictly?
                    EXAM calls PD_Phy_SendPack inside ISR. 
                    It sets DMA to AckBuf, Length=2, Start TX. */
                 
                 /* Critical: Update DMA to Point to ACK Buf temporarily */
                 pd->USBPD_DMA = (uint32_t)data->ack_buf;
                 pd->TX_SEL = UPD_SOP0;
                 pd->BMC_TX_SZ = 2;
                 
                 pd->USBPD_CONFIG |= IE_TX_END;
                 
                 /* Enable CC_LVE for TX */
                 if ((pd->USBPD_CONFIG & CC_SEL) == CC_SEL) {
                    pd->PORT_CC2 |= CC_LVE;
                 } else {
                    pd->PORT_CC1 |= CC_LVE;
                 }
                 
                 pd->USBPD_CONTROL |= PD_TX_EN | BMC_START;
                 
                 /* We don't notify stack yet. We notify after TX_END or just notify RX now?
                    Stack expects message. 
                    We should notify stack of the RECEIVED message. 
                 */
                 if (data->alert_cb) {
                     data->alert_cb(dev, data->alert_data, TCPC_ALERT_MSG_STATUS);
                 }
                 
                 return; /* Return, wait for TX_END */
             } else {
                 /* It IS a GoodCRC. We probably just finished sending something. 
                    Or the other side sent GoodCRC (weird if we didn't send).
                    If we received GoodCRC, it acknowledges our TX.
                 */
                 if (data->alert_cb) {
                    /* TX Success! */
                    /* Note: Typically Zephyr stack handles TX Success via GoodCRC match. 
                       But here we just say MSG_STATUS? 
                       Wait, TCPC_ALERT_TRANSMIT_MSG_SUCCESS is for when WE sent a message and got GoodCRC.
                    */
                    // data->alert_cb(dev, data->alert_data, TCPC_ALERT_TRANSMIT_MSG_SUCCESS);
                    /* We'll handle TX success logic if we were transmitting. */
                 }
             }
        }
    }

    if (status & IF_TX_END) {
        /* TX Complete */
        pd->USBPD_STATUS = IF_TX_END;
        
        /* Disable CC_LVE */
        pd->PORT_CC1 &= ~CC_LVE;
        pd->PORT_CC2 &= ~CC_LVE;
        
        pd->USBPD_CONTROL &= ~PD_TX_EN;
        
        /* Switch back to RX DMA */
        pd->USBPD_DMA = (uint32_t)data->rx_buf;
        pd->BMC_CLK_CNT = UPD_TMR_RX_48M;
        pd->USBPD_CONTROL |= BMC_START; /* Re-enable RX */
        
        if (data->alert_cb) {
            data->alert_cb(dev, data->alert_data, TCPC_ALERT_TRANSMIT_MSG_SUCCESS);
        }
    }

    if (status & IF_RX_RESET) {
        pd->USBPD_STATUS = IF_RX_RESET;
        if (data->alert_cb) {
             data->alert_cb(dev, data->alert_data, TCPC_ALERT_HARD_RESET_RECEIVED);
        }
    }
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

/* API Structs */
static const struct tcpc_driver_api tcpc_wch_driver_api = {
    .init = tcpc_wch_init,
    .get_cc = tcpc_wch_get_cc,
    .select_rp_value = tcpc_wch_select_rp_value,
    .get_rp_value = tcpc_wch_get_rp_value,
    .set_cc = tcpc_wch_set_cc,
    .get_rx_pending_msg = tcpc_wch_get_rx_pending_msg,
    .set_rx_enable = tcpc_wch_set_rx_enable,
    .set_cc_polarity = tcpc_wch_set_cc_polarity,
    .transmit_data = tcpc_wch_transmit_data,
    .set_alert_handler_cb = tcpc_wch_set_alert_handler_cb,
};

#define TCPC_WCH_INIT(n)                                                        \
    PINCTRL_DT_INST_DEFINE(n);                                                  \
    static const struct tcpc_wch_config tcpc_wch_cfg_##n = {                    \
        .base = DT_INST_REG_ADDR(n),                                            \
        .pcfg = PINCTRL_DT_INST_DEV_CONFIG_GET(n),                              \
        .clock_dev = DEVICE_DT_GET(DT_INST_CLOCKS_CTLR(n)),                     \
        .pclk_id = DT_INST_CLOCKS_CELL_BY_IDX(n, 0, id),                        \
    };                                                                          \
    static struct tcpc_wch_data tcpc_wch_data_##n;                              \
    DEVICE_DT_INST_DEFINE(n, &tcpc_wch_init, NULL,                              \
                          &tcpc_wch_data_##n, &tcpc_wch_cfg_##n,                \
                          POST_KERNEL, CONFIG_USBC_TCPC_INIT_PRIORITY,          \
                          &tcpc_wch_driver_api);

DT_INST_FOREACH_STATUS_OKAY(TCPC_WCH_INIT)
