/*
 * SPDX-License-Identifier: Apache-2.0
 * 
 * Simple ICM20948 Self-Test Example
 * Minimal example showing how to trigger self-test
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/logging/log.h>
#include "icm20948.h"

LOG_MODULE_REGISTER(icm20948_self_test_simple, LOG_LEVEL_INF);

#define ICM20948_NODE DT_NODELABEL(icm20948)
static const struct device *icm20948_dev = DEVICE_DT_GET(ICM20948_NODE);

int main(void)
{
    struct sensor_value trigger_val = {.val1 = 1, .val2 = 0};
    int ret;
    
    LOG_INF("ICM20948 Simple Self-Test Example");
    
    if (!device_is_ready(icm20948_dev)) {
        LOG_ERR("ICM20948 device not ready");
        return -1;
    }
    
    LOG_INF("Triggering ICM20948 self-test...");
    
    /* Trigger self-test - any non-zero value works */
    ret = sensor_attr_set(icm20948_dev, 
                         SENSOR_CHAN_ALL,
                         SENSOR_ATTR_ICM20948_SELF_TEST, 
                         &trigger_val);
    
    if (ret == 0) {
        LOG_INF("✓ Self-test PASSED - sensor is functioning correctly");
    } else {
        LOG_ERR("✗ Self-test FAILED with error %d", ret);
    }
    
    return ret;
}
