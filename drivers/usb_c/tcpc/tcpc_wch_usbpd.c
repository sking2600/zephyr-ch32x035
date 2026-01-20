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
#include <zephyr/drivers/adc.h>

/* Register Access */
#ifndef USBPD
#define USBPD ((USBPD_TypeDef *)DT_INST_REG_ADDR(0))
#endif

/* Constants from ch32x035_usbpd.h */
#define UPD_TMR_TX_96M    (160-1)
#define UPD_TMR_RX_96M    (240-1)
#define UPD_TMR_TX_72M    (120-1)
#define UPD_TMR_RX_72M    (240-1)  /* 72MHz / 240 = 300kHz (USB PD BMC Rate) */
#define UPD_TMR_TX_48M    (80-1)
#define UPD_TMR_RX_48M    (120-1)

/* USBPD->CONFIG */
#define PD_FILT_ED          (1<<0)
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
#define PA_CC_AI            (1<<0)    /* CC port comparator analog input (read-only result) */
#define CC_PD               (1<<1)    /* CC port pull-down resistor enable */
#define CC_PU_Mask          (3<<2)    /* CC port pull-up current mask */
#define CC_PU_80            (3<<2)    /* 80uA */
#define CC_PU_180           (2<<2)    /* 180uA */
#define CC_PU_330           (1<<2)    /* 330uA */
#define CC_LVE              (1<<4)    /* CC port output low voltage enable */
#define CC_CMP_Mask         (7<<5)    /* Voltage comparator threshold mask (bits 5-7) */
#define CC_CMP_22           (2<<5)    /* 0.22V threshold */
#define CC_CMP_45           (3<<5)    /* 0.45V threshold */
#define CC_CMP_55           (4<<5)    /* 0.55V threshold */
#define CC_CMP_66           (5<<5)    /* 0.66V threshold */
#define CC_CMP_95           (6<<5)    /* 0.95V threshold */
#define CC_CMP_123          (7<<5)    /* 1.23V threshold */


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
    
    /* VBUS ADC */
    struct adc_dt_spec vbus_adc;
    uint32_t vbus_divider;
};

struct tcpc_wch_data {
    tcpc_alert_handler_cb_t alert_cb;
    void *alert_data;
    
    struct k_work_delayable vbus_check;
    struct k_work alert_work;  /* Work item for ISR-safe alert dispatch */
    enum tcpc_alert alert_type; /* Alert type to pass to callback */

    enum tc_rp_value rp_val;
    uint8_t __aligned(4) tx_buf[256];
    uint8_t __aligned(4) rx_buf[256];
    uint8_t ack_buf[2]; 
    
    /* State Tracking */
    enum tc_power_role role;
    enum pd_rev_type rev;
    bool tx_pending_stack; /* True if TX was initiated by the USBC stack */
    bool rx_pending;       /* True if a new RX message is available */
    
    uint32_t vbus_mv_last;
    uint32_t power_status; /* Cache for TCPC_POWER_STATUS */
    uint32_t isr_count;
    uint16_t bmc_clk_cnt;     /* TX BMC Clock (300kHz) */
    uint16_t bmc_clk_rx_cnt;  /* RX BMC Clock (400kHz for WCH) */
    
    const struct device *dev; /* Back pointer */
};

static void tcpc_wch_isr(const struct device *dev);
static void tcpc_wch_vbus_check(struct k_work *work);
static void tcpc_wch_alert_worker(struct k_work *work);

static int tcpc_wch_init(const struct device *dev)
{
    const struct tcpc_wch_config *cfg = dev->config;
    struct tcpc_wch_data *data = dev->data;
    int ret;

    printk("Initializing WCH TCPC (Rev 2 - EXAM Based)\n");
    // LOG_INF("Initializing WCH TCPC (Rev 2 - EXAM Based)");
    
    data->dev = dev;

    /* 1. Clocks */
    if (!device_is_ready(cfg->clock_dev)) {
        LOG_ERR("Clock device not ready");
        printk("Clock device not ready\n");
        return -ENODEV;
    }
    ret = clock_control_on(cfg->clock_dev, (clock_control_subsys_t)(uintptr_t)cfg->pclk_id);
    if (ret < 0) {
        LOG_ERR("Failed to enable TCPC clock");
        printk("Failed to enable TCPC clock: %d\n", ret);
        return ret;
    }

    /* Calculate BMC Clock Count: (SystemCoreClock / 300000) - 1 */
    uint32_t sys_clk = sys_clock_hw_cycles_per_sec();
    data->bmc_clk_cnt = (sys_clk / 300000) - 1;
    data->bmc_clk_rx_cnt = (sys_clk / 400000) - 1;
    printk("TCPC: SysClk=%d Hz, BMC_TX=%d, BMC_RX=%d\n", sys_clk, data->bmc_clk_cnt, data->bmc_clk_rx_cnt);

    /* AFIO clock is usually enabled by the pinctrl driver (PRE_KERNEL_1) */

    /* 2. Pinctrl */
    ret = pinctrl_apply_state(cfg->pcfg, PINCTRL_STATE_DEFAULT);
    if (ret < 0) {
        LOG_ERR("Failed to configure Pinctrl");
        printk("Failed to configure Pinctrl: %d\n", ret);
        return ret;
    }

    /* 3. AFIO Configuration - Combined Fix from Learnings */
    /* Ensure AFIO Clock is enabled (Bit 0) */
    RCC->APB2PCENR |= (1 << 0);
    
    /* Enable PD PHY I/O: Set Bit 9 (USBPD_IN_HVT/PD_IO_EN) */
    AFIO->CR |= (1 << 9);
    printk("AFIO->CR: 0x%08x\n", AFIO->CR);
    
    /* Disable Pin Remap: Clear Bit 15 (PD01_RM) to keep PD on PB6/PB7 */
    AFIO->PCFR1 &= ~(1 << 15);
    
    /* 3b. GPIO Configuration - PB6/PB7 must be FLOATING INPUT for USBPD PHY (Reference: EXAM PD_Init) */
    /* CNF=01 (Floating), MODE=00 (Input) -> 0x4 */
    RCC->APB2PCENR |= (1 << 3);  /* Enable GPIOB clock */
    GPIOB->CFGLR &= ~(0xFF << 24); 
    GPIOB->CFGLR |= (0x44 << 24); /* Set PB6/PB7 to Floating Input */

    /* 4. USBPD Configuration - EXAM Style: Minimal Init */
    USBPD_TypeDef *pd = (USBPD_TypeDef *)cfg->base;
    
    /* Clear CONFIG to minimal state like EXAM's PD_Init() */
    pd->USBPD_CONFIG = PD_DMA_EN | PD_FILT_ED;
    
    /* Clear all status flags */
    pd->USBPD_STATUS = 0xFF;
    
    /* Initialize state */
    data->role = TC_ROLE_SINK;
    data->rev = PD_REV20;
    data->tx_pending_stack = false;

    IRQ_CONNECT(DT_INST_IRQN(0), 0, tcpc_wch_isr, DEVICE_DT_INST_GET(0), 0);
    irq_enable(DT_INST_IRQN(0));

    /* Initialize VBUS Polling */
    if (cfg->vbus_adc.dev) {
        printk("VBUS ADC device: %s\n", cfg->vbus_adc.dev->name);
        if (!adc_is_ready_dt(&cfg->vbus_adc)) {
             LOG_ERR("ADC device not ready");
             printk("ADC device not ready\n");
             return -ENODEV;
        }
        
        ret = adc_channel_setup_dt(&cfg->vbus_adc);
        if (ret < 0) {
            LOG_ERR("Could not setup VBUS ADC channel");
            printk("Could not setup VBUS ADC channel: %d\n", ret);
            return ret;
        }
    
        k_work_init_delayable(&data->vbus_check, tcpc_wch_vbus_check);
        k_work_schedule(&data->vbus_check, K_MSEC(100));
    }

    /* Initialize Alert Work (for ISR-safe callback dispatch) */
    k_work_init(&data->alert_work, tcpc_wch_alert_worker);

    return 0;
}

static enum tc_cc_voltage_state tcpc_wch_measure_cc_sink(USBPD_TypeDef *pd, int cc_idx)
{
    volatile uint16_t *port_reg = (cc_idx == 1) ? (uint16_t *)&pd->PORT_CC1 : (uint16_t *)&pd->PORT_CC2;
    uint16_t base = *port_reg;
    
    /* EXAM Logic from PD_Detect:
       USBPD->PORT_CC1 &= ~( CC_CE|PA_CC_AI );
       USBPD->PORT_CC1 |= CC_CMP_22;
       Delay_Us(2);
       if( USBPD->PORT_CC1 & PA_CC_AI ) ...
    */
    
    /* We need to define thresholds. 
       EXAM checks > 0.22V.
       Sink Default (vRd-USB) is 0.2V-2.6V? No.
       Rp=Default: 0.2V - 1.5V (SRC) -> Sink sees voltage across 5.1k.
       USB Default Rp (80uA/56k): ~0.4V at sink.
       1.5A Rp (180uA/22k): ~0.9V at sink.
       3.0A Rp (330uA/10k): ~1.7V at sink.
       
       EXAM Checks:
       1. CC_CMP_22 (0.22V). If > 0.22V, it's connected.
    */
    
    /* Check Connectivity (> 0.22V) */
    *port_reg = (base & ~CC_CMP_Mask) | CC_CMP_22;
    k_busy_wait(5);
    // printk("CC%d Measure 22: Reg=0x%04x\n", cc_idx, *port_reg);
    if (!(*port_reg & PA_CC_AI)) {
        return TC_CC_VOLT_OPEN;
    }
    
    /* Check > 0.66V (1.5A) */
    *port_reg = (base & ~CC_CMP_Mask) | CC_CMP_66;
    k_busy_wait(5);
    if (!(*port_reg & PA_CC_AI)) {
        return TC_CC_VOLT_RP_DEF; /* >0.22V but <0.66V */
    }
    
    /* Check > 1.23V (3.0A) */
    *port_reg = (base & ~CC_CMP_Mask) | CC_CMP_123;
    k_busy_wait(5);
    if (!(*port_reg & PA_CC_AI)) {
        return TC_CC_VOLT_RP_1A5; /* >0.66V but <1.23V */
    }
    
    return TC_CC_VOLT_RP_3A0; /* >1.23V */
}

static enum tc_cc_voltage_state tcpc_wch_measure_cc_source(USBPD_TypeDef *pd, int cc_idx)
{
    volatile uint16_t *port_reg = (cc_idx == 1) ? (uint16_t *)&pd->PORT_CC1 : (uint16_t *)&pd->PORT_CC2;
    uint16_t base = *port_reg;
    
    /* Clear CMP mask and AI bit */
    base &= ~(CC_CMP_Mask | PA_CC_AI);
    
    /* Check for Rd: Voltage should be < 1.6V (approx) but > 0.22V 
       Actually, if voltage < 0.22V, it might be Ra or Open? No, Ra is < 0.2V.
       If we have Rp enabled, and partner has Rd:
       Rp=80uA, Rd=5.1k -> V = 0.4V
       Rp=180uA, Rd=5.1k -> V = 0.9V
       Rp=330uA, Rd=5.1k -> V = 1.6V
    */
    
    /* Check if voltage > 0.22V */
    *port_reg = base | CC_CMP_22;
    k_busy_wait(5);
    if (!(*port_reg & PA_CC_AI)) {
        return TC_CC_VOLT_OPEN; /* Voltage < 0.22V -> Open or Ra */
    }
    
    /* Check if voltage < 1.6V? We don't have exactly 1.6V CMP.
       Max CMP is 1.23V (CC_CMP_123).
       If voltage > 1.23V and partner is Rd, it must be 3A0 Rp.
    */
    return TC_CC_VOLT_RD;
}

static int tcpc_wch_get_cc(const struct device *dev, enum tc_cc_voltage_state *cc1,
                                enum tc_cc_voltage_state *cc2)
{
    const struct tcpc_wch_config *cfg = dev->config;
    struct tcpc_wch_data *data = dev->data;
    USBPD_TypeDef *pd = (USBPD_TypeDef *)cfg->base;
    
    if (data->role == TC_ROLE_SINK) {
        *cc1 = tcpc_wch_measure_cc_sink(pd, 1);
        *cc2 = tcpc_wch_measure_cc_sink(pd, 2);
    } else {
        *cc1 = tcpc_wch_measure_cc_source(pd, 1);
        *cc2 = tcpc_wch_measure_cc_source(pd, 2);
    }

    return 0;
}

static int tcpc_wch_select_rp_value(const struct device *dev, enum tc_rp_value rp)
{
    struct tcpc_wch_data *data = dev->data;
    data->rp_val = rp;
    
    /* If we are currently in Rp mode, update hardware */
    /* This requires re-calling set_cc or similar. 
       For now just store it. */
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
        cc_cfg = CC_PD | CC_CMP_66; /* Set Rd and EXAM default CMP level (0.66V) */
        data->role = TC_ROLE_SINK;
        break;
    case TC_CC_RP:
        /* Map TC_RP_* to CC_PU_* */
        /* rp_val: 0=USB, 1=1A5, 2=3A0 */
        /* CC_PU: 3=80uA, 2=180uA, 1=330uA */
        cc_cfg = ((3 - data->rp_val) << 2) | CC_CMP_66; 
        data->role = TC_ROLE_SOURCE;
        break;
    case TC_CC_OPEN:
        cc_cfg = 0;
        data->role = TC_ROLE_SINK; /* Default to Sink role when open */
        break;
    default:
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
        
        /* DEBUG: Enable ALL IE bits (0xFF) to ensure RX_SUCCESS (Bit 1) is not masked */
        /* Note: IE bits are in the second byte (bits 8-15) */
        /* Also ensure PD_FILT_ED (Bit 1) is set for noise immunity */
        pd->USBPD_CONFIG |= (0xFF << 8) | PD_DMA_EN | PD_FILT_ED; 
        
        /* DEBUG: Explicitly mask to lower 16 bits for 16-bit DMA register */
        /* DMA requires full 32-bit address on CH32L103 (AHB) */
        pd->USBPD_DMA = (uint32_t)data->rx_buf;
        
        pd->USBPD_CONTROL &= ~PD_TX_EN;
        pd->USBPD_CONTROL &= ~PD_TX_EN;
        /* RX requires calculated timing (400kHz base) */
        pd->BMC_CLK_CNT = data->bmc_clk_rx_cnt;
        pd->USBPD_CONTROL |= BMC_START;
        
        /* Re-enable IRQ after PHY is fully configured (EXAM pattern) */
        NVIC_EnableIRQ(65);
    } else {
        pd->USBPD_CONTROL &= ~BMC_START;
    }
    return 0;
}


static int tcpc_wch_get_rx_pending_msg(const struct device *dev, struct pd_msg *msg)
{
    struct tcpc_wch_data *data = dev->data;

    if (!data->rx_pending) {
        return -ENODATA;
    }

    /* Bytes 0,1 are header in little endian */
    uint16_t header = data->rx_buf[0] | (data->rx_buf[1] << 8);
    
    msg->header.raw_value = header;
    msg->len = PD_HEADER_CNT(header) * 4;
    msg->type = PD_HEADER_TYPE(header);
    
    if (msg->len > 0) {
        memcpy(msg->data, &data->rx_buf[2], msg->len);
    }

    data->rx_pending = false;
    return 0;
}

static int tcpc_wch_set_roles(const struct device *dev, enum tc_power_role power_role,
				 enum tc_data_role data_role)
{
    struct tcpc_wch_data *data = dev->data;
    data->role = (power_role == TC_ROLE_SOURCE) ? TC_ROLE_SOURCE : TC_ROLE_SINK;
    /* Data role (UFP/DFP) is typically used for header field. 
       Zephyr stack usually builds the header itself into the msg->header. */
    return 0;
}

static int tcpc_wch_transmit_data(const struct device *dev, struct pd_msg *msg)
{
    const struct tcpc_wch_config *cfg = dev->config;
    struct tcpc_wch_data *data = dev->data;
    USBPD_TypeDef *pd = (USBPD_TypeDef *)cfg->base;

    /* Basic CCA: Check if hardware is currently busy with another BMC operation */
    /* Since we are always in RX mode, BMC_START is usually 1. 
       We check if PD_TX_EN is already set or if RX state machine is active. */
    if ((pd->USBPD_CONTROL & PD_TX_EN) || (pd->USBPD_CONTROL & (RX_STATE_0 | RX_STATE_1 | RX_STATE_2))) {
        return -EBUSY;
    }
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
    pd->BMC_TX_SZ = total_len;
    pd->BMC_CLK_CNT = data->bmc_clk_cnt;

    if ((pd->USBPD_CONFIG & CC_SEL) == CC_SEL) {
        pd->PORT_CC2 |= CC_LVE;
    } else {
        pd->PORT_CC1 |= CC_LVE;
    }

    data->tx_pending_stack = true;
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
    
    /* Hardware swap: flags are in Byte 0 (offset 0x08) */
    uint16_t status_word = pd->R16_STATUS;
    uint8_t status = (uint8_t)status_word; 
    
    if (status == 0) return;
    
    data->isr_count++;
    
    /* Clear flags immediately using 16-bit W1C (write back word to clear any byte) */
    pd->R16_STATUS = status_word;

    if (status & IF_RX_ACT) {
        /* Check SOP type in BMC_AUX (bits 0-1) */
        if ((status & 0x03) == 0x01) { /* BMC_AUX_SOP0 */
            uint8_t rx_len = pd->BMC_BYTE_CNT;
            if (rx_len >= 6) {
                 data->rx_pending = true;
                 /* Schedule alert callback from system work queue (not ISR context) */
                 data->alert_type = TCPC_ALERT_MSG_STATUS;
                 k_work_submit(&data->alert_work);
            }
        }
        /* Re-enter RX mode */
        pd->USBPD_CONFIG |= PD_ALL_CLR;
        pd->USBPD_CONFIG &= ~PD_ALL_CLR;
        pd->USBPD_DMA = (uint32_t)data->rx_buf;
        pd->USBPD_CONFIG &= ~PD_ALL_CLR;
        pd->USBPD_DMA = (uint32_t)data->rx_buf;
        pd->BMC_CLK_CNT = data->bmc_clk_rx_cnt;
        pd->USBPD_CONTROL |= BMC_START;
    }
    
    if (status & (IF_TX_END | IF_RX_RESET)) {
        pd->USBPD_CONFIG |= PD_ALL_CLR;
        pd->USBPD_CONFIG &= ~PD_ALL_CLR;
        pd->USBPD_DMA = (uint32_t)data->rx_buf;
        pd->USBPD_DMA = (uint32_t)data->rx_buf;
        pd->BMC_CLK_CNT = data->bmc_clk_rx_cnt;
        pd->USBPD_CONTROL |= BMC_START;
        
        if (status & IF_TX_END) {
             if (data->tx_pending_stack) {
                 data->tx_pending_stack = false;
                 /* Schedule alert callback from system work queue */
                 data->alert_type = TCPC_ALERT_TRANSMIT_MSG_SUCCESS;
                 k_work_submit(&data->alert_work);
             }
        }
    }
}

static void tcpc_wch_alert_worker(struct k_work *work)
{
    struct tcpc_wch_data *data = CONTAINER_OF(work, struct tcpc_wch_data, alert_work);
    const struct device *dev = data->dev;
    
    if (data->alert_cb) {
        data->alert_cb(dev, data->alert_data, data->alert_type);
    }
}

static void tcpc_wch_vbus_check(struct k_work *work)
{
    struct k_work_delayable *dwork = k_work_delayable_from_work(work);
    struct tcpc_wch_data *data = CONTAINER_OF(dwork, struct tcpc_wch_data, vbus_check);
    
    /* Device pointer is not stored in data? We need it to get config. 
       Usually we stick a device pointer in data. Or use CONTAINER_OF on device->data? 
       Wait, `data` is `dev->data`. We can't get `dev` from `data` easily unless we store it.
       Let's store `dev` in `data` during init.
    */
    /* Fix: Add dev to data struct or pass it differently? 
       Standard pattern: store dev in data. */
    const struct device *dev = data->dev;
    const struct tcpc_wch_config *cfg = dev->config;
    
    int ret;
    int16_t sample_buffer;
    struct adc_sequence sequence = {
        .buffer = &sample_buffer,
        .buffer_size = sizeof(sample_buffer),
        .resolution = 12,
        .channels = BIT(0), /* We only have 1 channel in the dt_spec, but index 0? 
                               adc_measure_dt uses seq->channels based on channel_id */
    };
    
    /* Correct way to use adc_dt_spec: initialization sets up channel ID. 
       measure_dt handles the sequence setup mostly?
       No, adc_sequence_init_dt exists. */
       
    adc_sequence_init_dt(&cfg->vbus_adc, &sequence);
    
    ret = adc_read_dt(&cfg->vbus_adc, &sequence);
    if (ret < 0) {
        LOG_ERR("VBUS ADC read failed: %d", ret);
    } else {
        int32_t mv = (int32_t)sample_buffer;
        adc_raw_to_millivolts_dt(&cfg->vbus_adc, &mv);
        
        /* Apply Divider */
        mv *= cfg->vbus_divider;
        
        uint32_t new_status = 0;
        /* TCPC Spec: 
           Bit 2: VBUS Present (VSafe5V) -> > 4.75V usually? 
           Let's use 4000mV as safe threshold for now.
        */
        if (mv > 4000) {
            new_status |= (1 << 2); /* VBUS Present */
        }
        
        /* Bit 3: VBUS Present Alert? No, that's in ALERT register. 
           TCPC Power Status Register (0x1fa):
           Bit 3: VBUS Detection Enabled
           Bit 2: VBUS Present
           Bit 1: VConn Present
           Bit 0: Sinking VBUS
        */
        
        if (data->role == TC_ROLE_SINK && (new_status & (1<<2))) {
             new_status |= (1 << 0); /* Sinking VBUS */
        }
        
        if (new_status != data->power_status) {
            data->power_status = new_status;
            /* Trigger Alert */
            if (data->alert_cb) {
                data->alert_cb(dev, data->alert_data, TCPC_ALERT_POWER_STATUS);
            }
        }
        
        data->vbus_mv_last = mv;
    }
    
    k_work_schedule(&data->vbus_check, K_MSEC(100));
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

static int tcpc_wch_get_status_register(const struct device *dev, enum tcpc_status_reg reg,
				   uint32_t *status)
{
    struct tcpc_wch_data *data = dev->data;
    
    switch(reg) {
        case TCPC_POWER_STATUS:
            *status = data->power_status;
            break;
        default:
             /* Other registers not implemented yet */
             *status = 0;
    }
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
    .set_roles = tcpc_wch_set_roles,
    .set_alert_handler_cb = tcpc_wch_set_alert_handler_cb,
    .get_status_register = tcpc_wch_get_status_register,
};

#define TCPC_WCH_INIT(n)                                                        \
    PINCTRL_DT_INST_DEFINE(n);                                                  \
    static const struct tcpc_wch_config tcpc_wch_cfg_##n = {                    \
        .base = DT_INST_REG_ADDR(n),                                            \
        .pcfg = PINCTRL_DT_INST_DEV_CONFIG_GET(n),                              \
        .clock_dev = DEVICE_DT_GET(DT_INST_CLOCKS_CTLR(n)),                     \
        .pclk_id = DT_INST_CLOCKS_CELL_BY_IDX(n, 0, id),                        \
        .vbus_adc = ADC_DT_SPEC_GET_BY_IDX(DT_DRV_INST(n), 0),                  \
        .vbus_divider = DT_INST_PROP_OR(n, vbus_divider_ratio, 1),              \
    };                                                                          \
    static struct tcpc_wch_data tcpc_wch_data_##n;                              \
    DEVICE_DT_INST_DEFINE(n, &tcpc_wch_init, NULL,                              \
                          &tcpc_wch_data_##n, &tcpc_wch_cfg_##n,                \
                          POST_KERNEL, CONFIG_USBC_TCPC_INIT_PRIORITY,          \
                          &tcpc_wch_driver_api);

DT_INST_FOREACH_STATUS_OKAY(TCPC_WCH_INIT)

uint32_t tcpc_wch_get_isr_count(const struct device *dev)
{
    struct tcpc_wch_data *data = dev->data;
    return data->isr_count;
}
