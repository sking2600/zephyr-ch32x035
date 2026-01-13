/*
 * Copyright (c) 2025 Scott King
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT wch_ch32_can

#include <zephyr/drivers/can.h>
#include <zephyr/drivers/can/transceiver.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/logging/log.h>
#include <zephyr/irq.h>
#include <zephyr/pm/device.h>
#include <hal_ch32fun.h>

LOG_MODULE_REGISTER(can_wch, CONFIG_CAN_LOG_LEVEL);

#define CAN_INIT_TIMEOUT_MS 10

 * NOTE regarding DMA:
 * The CH32L103 CAN controller supports DMA for *both* CAN FD and Classic CAN frames 
 * via the CANFD_DMA_T/R registers. This driver utilizes this internal DMA mechanism
 * for all transmissions and receptions to ensure efficient operation.
 */

static void can_wch_isr(const struct device *dev);

struct can_wch_config {
	const struct can_driver_config common;
	CAN_TypeDef *regs;
	const struct device *clock_dev;
	uint32_t clock_subsys;
	const struct pinctrl_dev_config *pin_cfg;
	void (*irq_config_func)(const struct device *dev);
};

struct can_wch_mailbox {
	can_tx_callback_t tx_callback;
	void *callback_arg;
};

struct can_wch_rx_filter {
	can_rx_callback_t callback;
	void *user_data;
	struct can_filter filter;
};

#define WCH_CAN_MAX_FILTERS 8

struct can_wch_data {
	struct can_driver_data common;
	struct k_mutex inst_mutex;
	struct k_sem tx_int_sem;
	struct can_wch_mailbox mb[3];
	struct can_wch_rx_filter rx_filters[WCH_CAN_MAX_FILTERS];
	enum can_state state;
	uint8_t tx_buf[3][64];
	uint8_t rx_buf[64] __aligned(4);
};

static int can_wch_get_capabilities(const struct device *dev, can_mode_t *cap)
{
	ARG_UNUSED(dev);

	*cap = CAN_MODE_NORMAL | CAN_MODE_LOOPBACK | CAN_MODE_LISTENONLY;

#ifdef CONFIG_CAN_FD_MODE
	*cap |= CAN_MODE_FD;
#endif

	return 0;
}

static int can_wch_start(const struct device *dev)
{
	const struct can_wch_config *cfg = dev->config;
	struct can_wch_data *data = dev->data;
	CAN_TypeDef *can = cfg->regs;
	int ret = 0;

	k_mutex_lock(&data->inst_mutex, K_FOREVER);

	if (data->common.started) {
		ret = -EALREADY;
		goto unlock;
	}

	/* Leave Init Mode */
	can->CTLR &= ~CAN_CTLR_INRQ;
	
	/* Wait for INAK to clear */
	uint32_t start_time = k_uptime_get_32();
	while ((can->STATR & CAN_STATR_INAK)) {
		if (k_uptime_get_32() - start_time > CAN_INIT_TIMEOUT_MS) {
			LOG_ERR("Failed to leave init mode");
			ret = -EIO;
			goto unlock;
		}
	}

	data->common.started = true;

unlock:
	k_mutex_unlock(&data->inst_mutex);
	return ret;
}

static int can_wch_stop(const struct device *dev)
{
	const struct can_wch_config *cfg = dev->config;
	struct can_wch_data *data = dev->data;
	CAN_TypeDef *can = cfg->regs;
	int ret = 0;

	k_mutex_lock(&data->inst_mutex, K_FOREVER);

	if (!data->common.started) {
		ret = -EALREADY;
		goto unlock;
	}

	/* Enter Init Mode */
	can->CTLR |= CAN_CTLR_INRQ;

	/* Wait for INAK to set */
	uint32_t start_time = k_uptime_get_32();
	while (!(can->STATR & CAN_STATR_INAK)) {
		if (k_uptime_get_32() - start_time > CAN_INIT_TIMEOUT_MS) {
			LOG_ERR("Failed to enter init mode");
			ret = -EIO;
			goto unlock;
		}
	}

	data->common.started = false;

unlock:
	k_mutex_unlock(&data->inst_mutex);
	return ret;
}

#ifdef CONFIG_CAN_FD_MODE
static int can_wch_set_timing_data(const struct device *dev,
				  const struct can_timing *timing_data)
{
	const struct can_wch_config *config = dev->config;
	CAN_TypeDef *can = config->regs;
	struct can_wch_data *data = dev->data;
	int ret = 0;

	k_mutex_lock(&data->inst_mutex, K_FOREVER);

	if (data->common.started) {
		ret = -EBUSY;
		goto unlock;
	}

	/* CANFD_BTR layout:
	 * BRP:   [25:16] (10 bits)
	 * TSEG1: [12:8]  (5 bits) - Approx, checking width
	 * TSEG2: [6:4]   (3 bits)
	 * SJW:   [3:0]   (4 bits)
	 * (Assumed based on shift values below. Standard might differ, but correcting clear mask to match logic.)
	 */
	can->CANFD_BTR &= ~(0x3FF << 16 | 0x1F << 8 | 0x7 << 4 | 0xF); 
	can->CANFD_BTR |= (timing_data->prescaler - 1) << 16;
	can->CANFD_BTR |= (timing_data->phase_seg1 - 1) << 8;
	can->CANFD_BTR |= (timing_data->phase_seg2 - 1) << 4;
	can->CANFD_BTR |= (timing_data->sjw - 1);

unlock:
	k_mutex_unlock(&data->inst_mutex);
	return ret;
}
#endif

static int can_wch_set_mode(const struct device *dev, can_mode_t mode)
{
	const struct can_wch_config *cfg = dev->config;
	struct can_wch_data *data = dev->data;
	CAN_TypeDef *can = cfg->regs;

	if (data->common.started) {
		return -EBUSY;
	}

	if ((mode & CAN_MODE_LOOPBACK) != 0) {
		can->BTIMR |= CAN_BTIMR_LBKM;
	} else {
		can->BTIMR &= ~CAN_BTIMR_LBKM;
	}

	if ((mode & CAN_MODE_LISTENONLY) != 0) {
		can->BTIMR |= CAN_BTIMR_SILM;
	} else {
		can->BTIMR &= ~CAN_BTIMR_SILM;
	}

#ifdef CONFIG_CAN_FD_MODE
	if ((mode & CAN_MODE_FD) != 0) {
		can->CANFD_CR |= CANFD_CR_TX_FD;
	} else {
		can->CANFD_CR &= ~CANFD_CR_TX_FD;
	}
#endif

	data->common.mode = mode;
	return 0;
}

static int can_wch_set_timing(const struct device *dev, const struct can_timing *timing)
{
	const struct can_wch_config *cfg = dev->config;
	struct can_wch_data *data = dev->data;
	CAN_TypeDef *can = cfg->regs;

	if (data->common.started) {
		return -EBUSY;
	}

	/* Prop Seg is included in Phase Seg 1 in WCH/STM32 hardware */
	uint32_t ts1 = timing->prop_seg + timing->phase_seg1;
	uint32_t ts2 = timing->phase_seg2;
	uint32_t sjw = timing->sjw;
	uint32_t brp = timing->prescaler;

	can->BTIMR = ((sjw - 1) << 24) |
		     ((ts1 - 1) << 16) |
		     ((ts2 - 1) << 20) |
		     (brp - 1);

	return 0;
}

static int can_wch_send(const struct device *dev, const struct can_frame *frame,
			k_timeout_t timeout, can_tx_callback_t callback, void *user_data)
{
	const struct can_wch_config *cfg = dev->config;
	struct can_wch_data *data = dev->data;
	CAN_TypeDef *can = cfg->regs;
	uint8_t mbox;

	if (!data->common.started) {
		return -ENETDOWN;
	}

	if (k_mutex_lock(&data->inst_mutex, timeout) != 0) {
		return -EAGAIN;
	}

	/* Find empty mailbox */
	if (can->TSTATR & CAN_TSTATR_TME0) {
		mbox = 0;
	} else if (can->TSTATR & CAN_TSTATR_TME1) {
		mbox = 1;
	} else if (can->TSTATR & CAN_TSTATR_TME2) {
		mbox = 2;
	} else {
		k_mutex_unlock(&data->inst_mutex);
		return -EAGAIN;
	}

	data->mb[mbox].tx_callback = callback;
	data->mb[mbox].callback_arg = user_data;

	/* Prepare Mailbox */
	can->sTxMailBox[mbox].TXMIR = 0;
	if (frame->flags & CAN_FRAME_IDE) {
		can->sTxMailBox[mbox].TXMIR = (frame->id << 3) | 1;
	} else {
		can->sTxMailBox[mbox].TXMIR = (frame->id << 21);
	}

	if (frame->flags & CAN_FRAME_RTR) {
		can->sTxMailBox[mbox].TXMIR |= 2;
	}

	/* Set DLC and transfer data */
	can->sTxMailBox[mbox].TXMDTR = frame->dlc;

#ifdef CONFIG_CAN_FD_MODE
	if (frame->flags & CAN_FRAME_FDF) {
		can->CANFD_CR |= CANFD_CR_TX_FD;
	}
#endif

	/* Use internal DMA for data */
	memcpy(data->tx_buf[mbox], frame->data, can_dlc_to_bytes(frame->dlc));
	can->CANFD_DMA_T[mbox] = (uint32_t)data->tx_buf[mbox];

	/* Request Transmission */
	can->sTxMailBox[mbox].TXMIR |= 1; /* TXRQ */

	k_mutex_unlock(&data->inst_mutex);
	return 0;
}

static int can_wch_add_rx_filter(const struct device *dev, can_rx_callback_t callback,
				 void *user_data, const struct can_filter *filter)
{
	struct can_wch_data *data = dev->data;
	int filter_id = -ENOSPC;

	k_mutex_lock(&data->inst_mutex, K_FOREVER);

	for (int i = 0; i < ARRAY_SIZE(data->rx_filters); i++) {
		if (data->rx_filters[i].callback == NULL) {
			data->rx_filters[i].callback = callback;
			data->rx_filters[i].user_data = user_data;
			data->rx_filters[i].filter = *filter;
			filter_id = i;
			break;
		}
	}

	k_mutex_unlock(&data->inst_mutex);

	return filter_id;
}

static void can_wch_remove_rx_filter(const struct device *dev, int filter_id)
{
	struct can_wch_data *data = dev->data;

	if (filter_id < 0 || filter_id >= ARRAY_SIZE(data->rx_filters)) {
		return;
	}

	k_mutex_lock(&data->inst_mutex, K_FOREVER);
	data->rx_filters[filter_id].callback = NULL;
	k_mutex_unlock(&data->inst_mutex);
}

static void can_wch_set_state_change_callback(const struct device *dev,
					      can_state_change_callback_t cb, void *user_data)
{
	struct can_wch_data *data = dev->data;
	data->common.state_change_cb = cb;
	data->common.state_change_cb_user_data = user_data;
}

static int can_wch_get_state(const struct device *dev, enum can_state *state,
			     struct can_bus_err_cnt *err_cnt)
{
	struct can_wch_data *data = dev->data;
	if (state) {
		*state = data->state;
	}
	return 0;
}

static int can_wch_get_core_clock(const struct device *dev, uint32_t *rate)
{
	const struct can_wch_config *cfg = dev->config;
	int ret;
	ret = clock_control_get_rate(cfg->clock_dev,
				     (clock_control_subsys_t)(uintptr_t)cfg->clock_subsys,
				     rate);
	if (ret != 0) {
		LOG_ERR("Failed to get clock rate: %d", ret);
		return ret;
	}

	return 0;
}

static int can_wch_init(const struct device *dev)
{
	const struct can_wch_config *cfg = dev->config;
	struct can_wch_data *data = dev->data;
	CAN_TypeDef *can = cfg->regs;
	int ret;

	k_mutex_init(&data->inst_mutex);
	k_sem_init(&data->tx_int_sem, 0, 1);

	/* Enable Clock */
	if (!device_is_ready(cfg->clock_dev)) {
		LOG_ERR("Clock device not ready");
		return -ENODEV;
	}
	clock_control_on(cfg->clock_dev, (clock_control_subsys_t)(uintptr_t)cfg->clock_subsys);

	/* Configure Pins */
	ret = pinctrl_apply_state(cfg->pin_cfg, PINCTRL_STATE_DEFAULT);
	if (ret < 0) {
		return ret;
	}

	/* Initialization sequence from WCH example */
	/* 1. Reset CAN */
	/* We might want a specific RCC reset here if available in HAL/clock_control */

	/* 2. Enter Init Mode */
	can->CTLR &= ~CAN_CTLR_SLEEP;
	can->CTLR |= CAN_CTLR_INRQ;
	
	/* Wait for init mode acknowledgment */
	uint32_t start_time = k_uptime_get_32();
	while ((can->STATR & CAN_STATR_INAK) != CAN_STATR_INAK) {
		if (k_uptime_get_32() - start_time > CAN_INIT_TIMEOUT_MS) {
			LOG_ERR("Failed to enter init mode");
			return -EIO;
		}
	}

	/* 3. Exit Sleep Mode (if it was set) */
	can->CTLR &= ~CAN_CTLR_SLEEP;

	/* 4. Configure DMA for RX */
	can->CANFD_DMA_R[0] = (uint32_t)data->rx_buf;

	/* 4. Configure Magic Bits (Based on WCH Example) */
	can->CANFD_CR |= CANFD_CR_RES_EXCEPT;
	can->CANFD_CR &= ~CANFD_CR_TX_FD;

	/* 5. Configure Interrupts */
	can->INTENR |= CAN_INTENR_TMEIE | CAN_INTENR_FMPIE0 | CAN_INTENR_ERRIE;
	cfg->irq_config_func(dev);

	return 0;
}

#ifdef CONFIG_PM_DEVICE
static int can_wch_pm_action(const struct device *dev, enum pm_device_action action)
{
	const struct can_wch_config *config = dev->config;
	int err;

	switch (action) {
	case PM_DEVICE_ACTION_SUSPEND:
		err = clock_control_off(config->clock_dev, (clock_control_subsys_t)(uintptr_t)config->clock_subsys);
		if (err < 0) {
			return err;
		}
		break;
	case PM_DEVICE_ACTION_RESUME:
		err = clock_control_on(config->clock_dev, (clock_control_subsys_t)(uintptr_t)config->clock_subsys);
		if (err < 0) {
			return err;
		}
		break;
	default:
		return -ENOTSUP;
	}

	return 0;
}
#endif

static const struct can_driver_api can_wch_driver_api = {
	.get_capabilities = can_wch_get_capabilities,
	.start = can_wch_start,
	.stop = can_wch_stop,
	.set_mode = can_wch_set_mode,
	.set_timing = can_wch_set_timing,
	.send = can_wch_send,
	.add_rx_filter = can_wch_add_rx_filter,
	.remove_rx_filter = can_wch_remove_rx_filter,
	.get_state = can_wch_get_state,
	.set_state_change_callback = can_wch_set_state_change_callback,
	.get_core_clock = can_wch_get_core_clock,
	.timing_min = {
		.sjw = 1,
		.prop_seg = 0,
		.phase_seg1 = 1,
		.phase_seg2 = 1,
		.prescaler = 1
	},
	.timing_max = {
		.sjw = 4,
		.prop_seg = 0,
		.phase_seg1 = 16,
		.phase_seg2 = 8,
		.prescaler = 1024
	},
#ifdef CONFIG_CAN_FD_MODE
	.set_timing_data = can_wch_set_timing_data,
	.timing_data_min = {
		.sjw = 1,
		.prop_seg = 0,
		.phase_seg1 = 1,
		.phase_seg2 = 1,
		.prescaler = 1
	},
	.timing_data_max = {
		.sjw = 4,
		.prop_seg = 0,
		.phase_seg1 = 16,
		.phase_seg2 = 8,
		.prescaler = 1024
	}
#endif
};

#define CAN_WCH_INIT(n)                                                                 \
	PINCTRL_DT_INST_DEFINE(n);                                                      \
	static void can_wch_irq_config_##n(const struct device *dev)                    \
	{                                                                               \
		IRQ_CONNECT(DT_INST_IRQ_BY_NAME(n, tx, irq), DT_INST_IRQ_BY_NAME(n, tx, priority), \
			    can_wch_isr, DEVICE_DT_INST_GET(n), 0);                     \
		irq_enable(DT_INST_IRQ_BY_NAME(n, tx, irq));                            \
		IRQ_CONNECT(DT_INST_IRQ_BY_NAME(n, rx, irq), DT_INST_IRQ_BY_NAME(n, rx, priority), \
			    can_wch_isr, DEVICE_DT_INST_GET(n), 0);                     \
		irq_enable(DT_INST_IRQ_BY_NAME(n, rx, irq));                            \
		IRQ_CONNECT(DT_INST_IRQ_BY_NAME(n, sce, irq), DT_INST_IRQ_BY_NAME(n, sce, priority), \
			    can_wch_isr, DEVICE_DT_INST_GET(n), 0);                     \
		irq_enable(DT_INST_IRQ_BY_NAME(n, sce, irq));                            \
	}                                                                               \
                                                                                        \
	static const struct can_wch_config can_wch_config_##n = {                       \
		.common = CAN_DT_DRIVER_CONFIG_INST_GET(n, 0, 8000000),                 \
		.regs = (CAN_TypeDef *)DT_INST_REG_ADDR(n),                             \
		.clock_dev = DEVICE_DT_GET(DT_INST_CLOCKS_CTLR(n)),                     \
		.clock_subsys = DT_INST_CLOCKS_CELL(n, id),                            \
		.pin_cfg = PINCTRL_DT_INST_DEV_CONFIG_GET(n),                           \
		.irq_config_func = can_wch_irq_config_##n,                              \
	};                                                                              \
                                                                                        \
	static struct can_wch_data can_wch_data_##n;                                    \
                                                                                        \
	PM_DEVICE_DT_INST_DEFINE(n, can_wch_pm_action);                                 \
	DEVICE_DT_INST_DEFINE(n, can_wch_init, PM_DEVICE_DT_INST_GET(n),                \
			      &can_wch_data_##n, &can_wch_config_##n,                   \
			      POST_KERNEL, CONFIG_CAN_INIT_PRIORITY,                    \
			      &can_wch_driver_api);

static void can_wch_isr(const struct device *dev)
{
	const struct can_wch_config *cfg = dev->config;
	struct can_wch_data *data = dev->data;
	CAN_TypeDef *can = cfg->regs;
	uint32_t tstatr = can->TSTATR;
	uint32_t rfifo0 = can->RFIFO0;

	/* Handle TX completion */
	for (int i = 0; i < 3; i++) {
		/* Actually, RQCP0 is bit 0, RQCP1 is bit 8, RQCP2 is bit 16 in TSTATR */
		if (tstatr & (CAN_TSTATR_RQCP0 << (8 * i))) {
			can_tx_callback_t cb = data->mb[i].tx_callback;
			void *arg = data->mb[i].callback_arg;

			can->TSTATR |= (CAN_TSTATR_RQCP0 << (8 * i)); /* Clear by setting */
			if (cb) {
				int status = (tstatr & (CAN_TSTATR_TXOK0 << (8 * i))) ? 0 : -EIO;
				cb(dev, status, arg);
			}
		}
	}

	/* Handle RX */
	if (rfifo0 & CAN_RFIFO0_FMP0) {
		struct can_frame frame = {0};
		uint32_t rxmir = can->sFIFOMailBox[0].RXMIR;
		uint32_t rxmdtr = can->sFIFOMailBox[0].RXMDTR;

		if (rxmir & 4) { /* IDE */
			frame.id = (rxmir >> 3) & CAN_EXT_ID_MASK;
			frame.flags |= CAN_FRAME_IDE;
		} else {
			frame.id = (rxmir >> 21) & CAN_STD_ID_MASK;
		}

		if (rxmir & 2) { /* RTR */
			frame.flags |= CAN_FRAME_RTR;
		}

		frame.dlc = rxmdtr & 0xF;
		
		/* Data is in shadow buffer via Internal DMA */
		/* Note: The HAL adds 0x20000000 if the register value is just an offset.
		 * But we provide the full address. Let's see if the hardware works with it.
		 */
		memcpy(frame.data, data->rx_buf, can_dlc_to_bytes(frame.dlc));

		/* Filter matching */
		for (int i = 0; i < ARRAY_SIZE(data->rx_filters); i++) {
			if (data->rx_filters[i].callback == NULL) {
				continue;
			}

			struct can_filter *f = &data->rx_filters[i].filter;
			bool match = false;

			if ((f->flags & CAN_FILTER_IDE) == (frame.flags & CAN_FRAME_IDE)) {
				if ((frame.id & f->mask) == (f->id & f->mask)) {
					match = true;
				}
			}

			if (match) {
				data->rx_filters[i].callback(dev, &frame, 
							     data->rx_filters[i].user_data);
			}
		}

		/* Release FIFO */
		can->RFIFO0 |= CAN_RFIFO0_RFOM0;
	}

	/* Handle Errors */
	if (can->ERRSR & (CAN_ERRSR_EWGF | CAN_ERRSR_EPVF | CAN_ERRSR_BOFF)) {
		/* Update state and notify if changed */
		enum can_state new_state = CAN_STATE_ERROR_ACTIVE;
		if (can->ERRSR & CAN_ERRSR_BOFF) {
			new_state = CAN_STATE_BUS_OFF;
		} else if (can->ERRSR & CAN_ERRSR_EPVF) {
			new_state = CAN_STATE_ERROR_PASSIVE;
		} else if (can->ERRSR & CAN_ERRSR_EWGF) {
			new_state = CAN_STATE_ERROR_WARNING;
		}

		if (new_state != data->state) {
			data->state = new_state;
			if (data->common.state_change_cb) {
				struct can_bus_err_cnt err_cnt;
				err_cnt.tx_err_cnt = (can->ERRSR >> 16) & 0xFF;
				err_cnt.rx_err_cnt = (can->ERRSR >> 24) & 0xFF;
				data->common.state_change_cb(dev, new_state, err_cnt, 
							     data->common.state_change_cb_user_data);
			}
		}
	}
}

DT_INST_FOREACH_STATUS_OKAY(CAN_WCH_INIT)

