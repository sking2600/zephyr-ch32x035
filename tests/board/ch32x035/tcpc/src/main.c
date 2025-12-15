/*
 * Copyright (c) 2024 Massdriver
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/ztest.h>
#include <zephyr/drivers/usb_c/usbc_tcpc.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(tcpc_test, LOG_LEVEL_INF);

const struct device *tcpc_dev = DEVICE_DT_GET(DT_NODELABEL(usbpd));

static void *tcpc_setup(void)
{
	k_object_access_grant(tcpc_dev, k_current_get());
	return NULL;
}

ZTEST_SUITE(tcpc_driver, NULL, tcpc_setup, NULL, NULL, NULL);

ZTEST(tcpc_driver, test_tcpc_initialization)
{
	zassert_true(device_is_ready(tcpc_dev), "TCPC device is not ready");
}

ZTEST(tcpc_driver, test_tcpc_get_cc)
{
	enum tc_cc_voltage_state cc1, cc2;
	int ret;

	ret = tcpc_get_cc(tcpc_dev, &cc1, &cc2);
	zassert_equal(ret, 0, "Failed to get CC state");
	
	LOG_INF("CC1 State: %d, CC2 State: %d", cc1, cc2);
}

ZTEST(tcpc_driver, test_tcpc_set_cc_rp)
{
	int ret;
	
    /* Set Role to Source (Rp) */
    ret = tcpc_select_rp_value(tcpc_dev, TC_RP_3A0);
    zassert_equal(ret, 0, "Failed to select RP value");

	ret = tcpc_set_cc(tcpc_dev, TC_CC_RP);
	zassert_equal(ret, 0, "Failed to set CC to RP");
    
    /* Verify implicit via get_cc isn't strictly possible without loopback, 
       but we ensure the driver returns success */
}

ZTEST(tcpc_driver, test_tcpc_set_cc_rd)
{
	int ret;
	ret = tcpc_set_cc(tcpc_dev, TC_CC_RD);
	zassert_equal(ret, 0, "Failed to set CC to RD");
}

ZTEST(tcpc_driver, test_tcpc_polarity)
{
	int ret;
	ret = tcpc_set_cc_polarity(tcpc_dev, TC_POLARITY_CC1);
	zassert_equal(ret, 0, "Failed to set polarity CC1");

	ret = tcpc_set_cc_polarity(tcpc_dev, TC_POLARITY_CC2);
	zassert_equal(ret, 0, "Failed to set polarity CC2");
}

ZTEST(tcpc_driver, test_dump_registers)
{
    tcpc_dump_std_reg(tcpc_dev);
}
