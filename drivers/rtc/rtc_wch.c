/*
 * Copyright (c) 2024 WCH
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT wch_ch32_rtc

#include <zephyr/drivers/rtc.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/util.h>
#include <zephyr/sys/timeutil.h>
#include <zephyr/irq.h>
#include <soc.h>
#include <errno.h>

LOG_MODULE_REGISTER(rtc, CONFIG_RTC_LOG_LEVEL);

/* RTC bits */
/* Using definitions from HAL */


struct rtc_wch_config {
	RTC_TypeDef *rtc;
	const struct device *pfic_dev;
	void (*irq_config_func)(const struct device *dev);
};

struct rtc_wch_data {
	struct k_mutex lock;
#ifdef CONFIG_RTC_ALARM
	rtc_alarm_callback alarm_callback;
	void *alarm_user_data;
	bool alarm_pending;
#endif
};

static int rtc_wch_wait_rtoff(RTC_TypeDef *rtc)
{
	uint32_t timeout = 10000; /* 100ms at 10us steps */

	while (!(rtc->CTLRL & RTC_CTLRL_RTOFF) && timeout > 0) {
		k_busy_wait(10);
		timeout--;
	}

	return (timeout == 0) ? -ETIMEDOUT : 0;
}

static int rtc_wch_enter_config(RTC_TypeDef *rtc)
{
	int err = rtc_wch_wait_rtoff(rtc);

	if (err != 0) {
		return err;
	}
	rtc->CTLRL |= RTC_CTLRL_CNF;
	return 0;
}

static int rtc_wch_exit_config(RTC_TypeDef *rtc)
{
	int err = rtc_wch_wait_rtoff(rtc);

	if (err != 0) {
		return err;
	}
	rtc->CTLRL &= ~RTC_CTLRL_CNF;
	return rtc_wch_wait_rtoff(rtc);
}

static int rtc_wch_set_time(const struct device *dev, const struct rtc_time *timeptr)
{
	const struct rtc_wch_config *cfg = dev->config;
	struct rtc_wch_data *data = dev->data;
	time_t timestamp;
	uint32_t counter;

	timestamp = timeutil_timegm((struct tm *)timeptr);
	
	if (timestamp == -1) {
		return -EINVAL;
	}

	counter = (uint32_t)timestamp;

	k_mutex_lock(&data->lock, K_FOREVER);

	/* Write to CNT registers */
	if (rtc_wch_enter_config(cfg->rtc) != 0) {
		k_mutex_unlock(&data->lock);
		return -EIO;
	}
	cfg->rtc->CNTH = (counter >> 16) & 0xFFFF;
	cfg->rtc->CNTL = (counter & 0xFFFF);
	if (rtc_wch_exit_config(cfg->rtc) != 0) {
		k_mutex_unlock(&data->lock);
		return -EIO;
	}

	k_mutex_unlock(&data->lock);

	return 0;
}

static int rtc_wch_get_time(const struct device *dev, struct rtc_time *timeptr)
{
	const struct rtc_wch_config *cfg = dev->config;
	uint32_t h, l;
	uint32_t counter;
	time_t timestamp;

	/* Read counter safely */
	do {
		h = cfg->rtc->CNTH;
		l = cfg->rtc->CNTL;
	} while (h != cfg->rtc->CNTH);

	counter = (h << 16) | l;
	timestamp = (time_t)counter;

	/* Convert timestamp to rtc_time */
	gmtime_r(&timestamp, (struct tm *)timeptr);

	return 0;
}

#ifdef CONFIG_RTC_ALARM

static int rtc_wch_set_alarm(const struct device *dev, uint16_t id, const struct rtc_alarm_status *alarms)
{
	const struct rtc_wch_config *cfg = dev->config;
	struct rtc_wch_data *data = dev->data;
	time_t timestamp;
	uint32_t alarm_val;

	if (id != 0) {
		return -EINVAL;
	}

	if ((alarms->mask & RTC_ALARM_TIME_MASK_SECOND) == 0 ||
	    (alarms->mask & RTC_ALARM_TIME_MASK_MINUTE) == 0 ||
	    (alarms->mask & RTC_ALARM_TIME_MASK_HOUR) == 0 ||
	    (alarms->mask & RTC_ALARM_TIME_MASK_MONTHDAY) == 0 ||
	    (alarms->mask & RTC_ALARM_TIME_MASK_MONTH) == 0 ||
	    (alarms->mask & RTC_ALARM_TIME_MASK_YEAR) == 0) {
		return -ENOTSUP; /* Hardware only supports full timestamp alarm */
	}

	timestamp = timeutil_timegm((struct tm *)&alarms->time);
	if (timestamp == -1) {
		return -EINVAL;
	}

	alarm_val = (uint32_t)timestamp;

	k_mutex_lock(&data->lock, K_FOREVER);

	if (rtc_wch_enter_config(cfg->rtc) != 0) {
		k_mutex_unlock(&data->lock);
		return -EIO;
	}
	cfg->rtc->ALRMH = (alarm_val >> 16) & 0xFFFF;
	cfg->rtc->ALRML = (alarm_val & 0xFFFF);
	
	/* Enable Alarm Interrupt */
	cfg->rtc->CTLRH |= RTC_CTLRH_ALRIE;
	
	if (rtc_wch_exit_config(cfg->rtc) != 0) {
		k_mutex_unlock(&data->lock);
		return -EIO;
	}

	k_mutex_unlock(&data->lock);

	return 0;
}

static int rtc_wch_get_alarm(const struct device *dev, uint16_t id, struct rtc_alarm_status *alarms)
{
	const struct rtc_wch_config *cfg = dev->config;
	uint32_t h, l;
	uint32_t alarm_val;
	time_t timestamp;

	if (id != 0) {
		return -EINVAL;
	}

	h = cfg->rtc->ALRMH;
	l = cfg->rtc->ALRML;
	alarm_val = (h << 16) | l;
	timestamp = (time_t)alarm_val;

	gmtime_r(&timestamp, (struct tm *)&alarms->time);
	alarms->mask = RTC_ALARM_TIME_MASK_SECOND | RTC_ALARM_TIME_MASK_MINUTE | 
	               RTC_ALARM_TIME_MASK_HOUR | RTC_ALARM_TIME_MASK_MONTHDAY | 
	               RTC_ALARM_TIME_MASK_MONTH | RTC_ALARM_TIME_MASK_YEAR;
	
	return 0;
}

static int rtc_wch_set_alarm_callback(const struct device *dev, uint16_t id,
				      rtc_alarm_callback callback, void *user_data)
{
	struct rtc_wch_data *data = dev->data;

	if (id != 0) {
		return -EINVAL;
	}

	k_mutex_lock(&data->lock, K_FOREVER);
	data->alarm_callback = callback;
	data->alarm_user_data = user_data;
	k_mutex_unlock(&data->lock);

	return 0;
}

static int rtc_wch_cancel_alarm(const struct device *dev, uint16_t id)
{
	const struct rtc_wch_config *cfg = dev->config;
	struct rtc_wch_data *data = dev->data;

	if (id != 0) {
		return -EINVAL;
	}

	k_mutex_lock(&data->lock, K_FOREVER);

	if (rtc_wch_enter_config(cfg->rtc) != 0) {
		k_mutex_unlock(&data->lock);
		return -EIO;
	}
	cfg->rtc->CTLRH &= ~RTC_CTLRH_ALRIE;
	if (rtc_wch_exit_config(cfg->rtc) != 0) {
		k_mutex_unlock(&data->lock);
		return -EIO;
	}
	
	data->alarm_callback = NULL;
	data->alarm_user_data = NULL;

	k_mutex_unlock(&data->lock);

	return 0;
}

static uint32_t rtc_wch_alarm_is_pending(const struct device *dev, uint16_t id)
{
	struct rtc_wch_data *data = dev->data;

	if (id != 0) {
		return 0;
	}

	return data->alarm_pending ? 1 : 0;
}

static void rtc_wch_isr(const struct device *dev)
{
	const struct rtc_wch_config *cfg = dev->config;
	struct rtc_wch_data *data = dev->data;
	
	if (cfg->rtc->CTLRL & RTC_CTLRL_ALR) {
		if (rtc_wch_enter_config(cfg->rtc) == 0) {
			cfg->rtc->CTLRL &= ~RTC_CTLRL_ALR; /* Clear alarm flag */
			rtc_wch_exit_config(cfg->rtc);
		}

		if (data->alarm_callback) {
			data->alarm_callback(dev, 0, data->alarm_user_data);
		}
		data->alarm_pending = true;
	}
}
#endif /* CONFIG_RTC_ALARM */


static const struct rtc_driver_api rtc_wch_driver_api = {
	.set_time = rtc_wch_set_time,
	.get_time = rtc_wch_get_time,
#ifdef CONFIG_RTC_ALARM
	.set_alarm = rtc_wch_set_alarm,
	.get_alarm = rtc_wch_get_alarm,
	.set_alarm_callback = rtc_wch_set_alarm_callback,
	.cancel_alarm = rtc_wch_cancel_alarm,
	.alarm_is_pending = rtc_wch_alarm_is_pending,
#endif
};

static int rtc_wch_init(const struct device *dev)
{
	const struct rtc_wch_config *cfg = dev->config;
	struct rtc_wch_data *data = dev->data;


	k_mutex_init(&data->lock);

	/* Enable BKP and PWR clocks */
	/* PWR(28) and BKP(27) bits in APB1PCENR */
	RCC->APB1PCENR |= (1 << 28) | (1 << 27);

	/* Enable access to Backup Domain */
	PWR->CTLR |= PWR_CTLR_DBP;

	/* Wait for Synch */
	cfg->rtc->CTLRL &= ~RTC_CTLRL_RSF;
	uint32_t timeout = 10000; /* 100ms */
	while (!(cfg->rtc->CTLRL & RTC_CTLRL_RSF) && timeout > 0) {
		k_busy_wait(10);
		timeout--;
	}
	if (timeout == 0) {
		return -EIO;
	}
	
	/* Wait for last operation */
	return rtc_wch_wait_rtoff(cfg->rtc);

#ifdef CONFIG_RTC_ALARM
	cfg->irq_config_func(dev);
#endif

	return 0;
}

#ifdef CONFIG_RTC_ALARM
#define RTC_WCH_IRQ_INIT(inst)                                          \
	.irq_config_func = rtc_wch_irq_config_##inst,
#define RTC_WCH_IRQ_DEFINE(inst)                                        \
	static void rtc_wch_irq_config_##inst(const struct device *dev) \
	{                                                               \
		IRQ_CONNECT(DT_INST_IRQN(inst),                         \
			    DT_INST_IRQ(inst, priority),                \
			    rtc_wch_isr, DEVICE_DT_INST_GET(inst), 0);  \
		irq_enable(DT_INST_IRQN(inst));                         \
	}
#else
#define RTC_WCH_IRQ_INIT(inst)
#define RTC_WCH_IRQ_DEFINE(inst)
#endif


#define RTC_WCH_INIT(inst)                                              \
	RTC_WCH_IRQ_DEFINE(inst)                                        \
	static struct rtc_wch_data rtc_wch_data_##inst;                 \
	static const struct rtc_wch_config rtc_wch_config_##inst = {    \
		.rtc = (RTC_TypeDef *)DT_INST_REG_ADDR(inst),             \
		RTC_WCH_IRQ_INIT(inst)                                  \
	};                                                              \
	DEVICE_DT_INST_DEFINE(inst, rtc_wch_init, NULL,                 \
			      &rtc_wch_data_##inst, &rtc_wch_config_##inst,\
			      POST_KERNEL, CONFIG_RTC_INIT_PRIORITY,    \
			      &rtc_wch_driver_api);

DT_INST_FOREACH_STATUS_OKAY(RTC_WCH_INIT)
