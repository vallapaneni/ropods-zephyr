/**
 * @file main.c
 * @brief Sensor Manager Sample Application
 * 
 * This sample demonstrates the sensor manager functionality with real sensor devices.
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/printk.h>

#include "sensor_manager.h"

LOG_MODULE_REGISTER(sensor_manager_sample, LOG_LEVEL_INF);

/* Sample configuration */
#define MAX_CALLBACKS 10

/* Application state */
static K_SEM_DEFINE(app_exit_sem, 0, 1);
static uint32_t total_callback_count = 0;

/* Forward declarations */
static void process_device_data(const struct device *device, const char *device_name);

/**
 * @brief User callback for data ready events with sample counting
 */
static void sensor_data_ready_callback(const struct device *device,
                                      const struct sensor_trigger *trigger)
{
    total_callback_count++;
    
    LOG_INF("Sample trigger fired for device: %s (callback #%u/%u)", 
            device->name, total_callback_count, MAX_CALLBACKS);
    
    /* Process sensor data immediately in callback */
    process_device_data(device, device->name);
    
    /* Stop sensors and signal application exit after MAX_CALLBACKS */
    if (total_callback_count >= MAX_CALLBACKS) {
        LOG_INF("Reached maximum callbacks (%u), stopping sensors...", MAX_CALLBACKS);
        
        /* Stop acquisition for all devices */
        sensor_manager_stop_acquisition_all();
        
        /* Signal main thread to exit */
        k_sem_give(&app_exit_sem);
    }
}

/**
 * @brief Process sensor data from a device
 */
static void process_device_data(const struct device *device, const char *device_name)
{
    struct sensor_sample_block *data_buffer;
    size_t blocks_read;
    
    /* Get the actual sample block size for this device */
    size_t sample_block_size = sensor_manager_get_sample_block_size(device);
    if (sample_block_size == 0) {
        LOG_WRN("No sample block size available for %s (acquisition not active?)", device_name);
        return;
    }
    
    /* Allocate buffer for one sample block using the actual size */
    data_buffer = k_malloc(sample_block_size);
    if (!data_buffer) {
        LOG_ERR("Failed to allocate %zu bytes for sensor data", sample_block_size);
        return;
    }
    
    int ret = sensor_manager_read_data_buffer(device, data_buffer, 1, &blocks_read);
    
    if (ret == SENSOR_MANAGER_OK && blocks_read > 0) {
        LOG_INF("%s: Read %zu sensor sample blocks", device_name, blocks_read);
        
        /* Get the enabled channels to know the data format */
        enum sensor_channel enabled_channels[SENSOR_MANAGER_MAX_CHANNELS_PER_DEVICE];
        size_t num_enabled_channels;
        ret = sensor_manager_get_enabled_channels(device, enabled_channels, 
                                                 ARRAY_SIZE(enabled_channels), 
                                                 &num_enabled_channels);
        
        if (ret == SENSOR_MANAGER_OK && num_enabled_channels > 0) {
            /* Parse the variable-length data using the actual sample block size */
            for (size_t i = 0; i < blocks_read; i++) {
                struct sensor_sample_block *block = (struct sensor_sample_block *)
                    ((uint8_t *)data_buffer + i * sample_block_size);
                
                LOG_INF("  [%u ms] Block with %zu channels (block size: %zu bytes):", 
                       block->timestamp_ms, num_enabled_channels, sample_block_size);
                
                /* Parse channel data */
                uint8_t *data_ptr = block->data;
                for (size_t j = 0; j < MIN(num_enabled_channels, 3); j++) {
                    /* Get channel data size */
                    size_t channel_data_size = sensor_manager_get_channel_data_size(device, enabled_channels[j]);
                    struct sensor_value *values = (struct sensor_value *)data_ptr;
                    
                    LOG_INF("    Ch %d: %d.%06d",
                           enabled_channels[j],
                           values[0].val1,
                           values[0].val2);
                    
                    data_ptr += channel_data_size;
                }
                
                if (num_enabled_channels > 3) {
                    LOG_INF("    ... and %zu more channels", num_enabled_channels - 3);
                }
            }
            
            if (blocks_read > 1) {
                LOG_INF("  ... and %zu more blocks", blocks_read - 1);
            }
        } else {
            LOG_WRN("Could not get enabled channels for %s", device_name);
        }
    }
    
    k_free(data_buffer);
}

/**
 * @brief Report statistics for all managed devices
 */
static void report_statistics(void)
{
    LOG_INF("=== Sensor Manager Statistics ===");
    
    /* Check ICM20948 if available */
    const struct device *icm_dev = DEVICE_DT_GET_OR_NULL(DT_NODELABEL(icm20948));
    if (icm_dev && sensor_manager_is_device_managed(icm_dev)) {
        uint32_t data_ready_count, overflow_count;
        uint8_t buffer_usage;
        
        int ret = sensor_manager_get_stats(icm_dev, &data_ready_count,
                                          &overflow_count, &buffer_usage);
        
        if (ret == SENSOR_MANAGER_OK) {
            size_t sample_block_size = sensor_manager_get_sample_block_size(icm_dev);
            LOG_INF("ICM20948: %u events, %u overflows, %u%% buffer usage, %zu byte samples",
                   data_ready_count, overflow_count, buffer_usage, sample_block_size);
        }
    }
    
    LOG_INF("==============================");
}

/**
 * @brief Setup and configure sensor devices
 */
static int setup_sensors(void)
{
    int ret;
    
    LOG_INF("Setting up sensor devices...");
    
    /* Initialize sensor manager */
    ret = sensor_manager_init();
    if (ret != SENSOR_MANAGER_OK) {
        LOG_ERR("Failed to initialize sensor manager: %d", ret);
        return ret;
    }
    
    LOG_INF("Sensor manager initialized successfully");
    
    /* Setup ICM20948 if available */
    const struct device *icm_dev = DEVICE_DT_GET_OR_NULL(DT_NODELABEL(icm20948));
    if (icm_dev) {
        if (device_is_ready(icm_dev)) {
            LOG_INF("ICM20948 device found and ready");
            
            /* Add device to sensor manager */
            ret = sensor_manager_add_device(icm_dev, "ICM20948", false, 128);
            if (ret == SENSOR_MANAGER_OK) {
                LOG_INF("Added ICM20948 to sensor manager");
                
                /* Enable accelerometer channels */
                sensor_manager_enable_channel(icm_dev, SENSOR_CHAN_ACCEL_X);
                sensor_manager_enable_channel(icm_dev, SENSOR_CHAN_ACCEL_Y);
                sensor_manager_enable_channel(icm_dev, SENSOR_CHAN_ACCEL_Z);
                LOG_INF("Enabled accelerometer channels");
                
                /* Enable gyroscope channels */
                sensor_manager_enable_channel(icm_dev, SENSOR_CHAN_GYRO_X);
                sensor_manager_enable_channel(icm_dev, SENSOR_CHAN_GYRO_Y);
                sensor_manager_enable_channel(icm_dev, SENSOR_CHAN_GYRO_Z);
                LOG_INF("Enabled gyroscope channels");
                
                /* Set up data ready callback with sample counting trigger */
                ret = sensor_manager_set_trigger_callback(icm_dev,
                                                         SENSOR_TRIG_DATA_READY,
                                                         sensor_data_ready_callback,
                                                         10,  /* Trigger every 10 samples */
                                                         true); /* Repeat trigger */
                if (ret == SENSOR_MANAGER_OK) {
                    LOG_INF("Set up data ready callback (trigger every 10 samples)");
                } else {
                    LOG_WRN("Failed to set callback: %d", ret);
                }
            } else {
                LOG_ERR("Failed to add ICM20948 to sensor manager: %d", ret);
            }
        } else {
            LOG_WRN("ICM20948 device not ready");
        }
    } else {
        LOG_WRN("ICM20948 device not found in device tree");
    }
    
    /* Check for other potential sensor devices */
    LOG_INF("Checking for other sensor devices...");
    
    /* Try to find generic sensor devices */
    const struct device *temp_dev = device_get_binding("TEMP_0");
    if (temp_dev && device_is_ready(temp_dev)) {
        LOG_INF("Found temperature sensor");
        ret = sensor_manager_add_device(temp_dev, "Temperature", false, 32);
        if (ret == SENSOR_MANAGER_OK) {
            sensor_manager_enable_channel(temp_dev, SENSOR_CHAN_AMBIENT_TEMP);
            sensor_manager_set_trigger_callback(temp_dev,
                                               SENSOR_TRIG_DATA_READY,
                                               sensor_data_ready_callback,
                                               5,    /* Trigger every 5 samples */
                                               true); /* Repeat trigger */
            LOG_INF("Temperature sensor configured (trigger every 5 samples)");
        }
    }
    
    LOG_INF("Sensor setup completed. Acquisition will be started after all devices are configured.");
    return SENSOR_MANAGER_OK;
}

/**
 * @brief Cleanup sensor resources
 */
static void cleanup_sensors(void)
{
    LOG_INF("Cleaning up sensors...");
    
    /* Stop acquisition for all devices */
    sensor_manager_stop_acquisition_all();
    
    /* Deinitialize sensor manager */
    sensor_manager_deinit();
    LOG_INF("Sensor cleanup completed");
}

/**
 * @brief Main application function
 */
int main(void)
{
    int ret;
    
    printk("\n");
    printk("=====================================\n");
    printk("  Sensor Manager Sample Application  \n");
    printk("=====================================\n");
    printk("Build: %s %s\n", __DATE__, __TIME__);
    printk("\n");
    
    LOG_INF("Starting Sensor Manager Sample");
    
    /* Setup sensors */
    ret = setup_sensors();
    if (ret != SENSOR_MANAGER_OK) {
        LOG_ERR("Sensor setup failed: %d", ret);
        return -1;
    }
    
    /* Start data acquisition for all configured devices */
    LOG_INF("Starting data acquisition for all devices...");
    ret = sensor_manager_start_acquisition_all();
    if (ret != SENSOR_MANAGER_OK) {
        LOG_ERR("Failed to start acquisition: %d", ret);
        cleanup_sensors();
        return -1;
    }
    LOG_INF("Data acquisition started successfully");
    
    /* Application main - wait for callbacks to complete */
    LOG_INF("Waiting for sensor callbacks to complete...");
    printk("Application will exit after %u sensor callbacks\n\n", MAX_CALLBACKS);
    
    /* Wait for the callback to signal completion */
    k_sem_take(&app_exit_sem, K_FOREVER);
    
    LOG_INF("All callbacks completed, exiting application");
    
    /* Cleanup and exit */
    cleanup_sensors();
    
    printk("\n");
    printk("=====================================\n");
    printk("     Sample Application Finished     \n");
    printk("=====================================\n");
    
    LOG_INF("Application finished successfully");
    return 0;
}
