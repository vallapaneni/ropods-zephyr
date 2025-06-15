/**
 * @file main_new.c
 * @brief New Sensor Manager Sample Application
 * 
 * This sample demonstrates the new application-level buffer threshold callback system.
 * The sensor manager now uses a single shared ring buffer for all device data, and the
 * application sets up a callback that triggers when the buffer exceeds a threshold.
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/printk.h>
#include <zephyr/sys/util.h>

#include "sensor_manager.h"

LOG_MODULE_REGISTER(sensor_manager_sample, LOG_LEVEL_INF);

/* Sample configuration */
#define BUFFER_THRESHOLD_BYTES 512     /* Trigger callback when buffer has 512+ bytes */
#define MAX_READ_BYTES 256             /* Read up to 256 bytes per callback */
#define MAX_CALLBACKS 10               /* Exit after this many callbacks */

/* Application state */
static K_SEM_DEFINE(app_exit_sem, 0, 1);
static uint32_t buffer_callback_count = 0;

/* Forward declarations */
static void buffer_threshold_callback(size_t available_bytes, void *user_data);
static void process_buffer_data(uint8_t *buffer, size_t bytes_read);

/**
 * @brief Buffer threshold callback - called when shared buffer exceeds threshold
 * 
 * This is the new application-level trigger mechanism that replaces device-level callbacks.
 * It's called when the shared buffer has more data than the configured threshold.
 */
static void buffer_threshold_callback(size_t available_bytes, void *user_data)
{
    buffer_callback_count++;
    
    LOG_INF("=== Buffer Threshold Callback #%u ===", buffer_callback_count);
    LOG_INF("Available bytes in shared buffer: %zu", available_bytes);
    
    /* Read data from the shared buffer */
    uint8_t *read_buffer = k_malloc(MAX_READ_BYTES);
    if (!read_buffer) {
        LOG_ERR("Failed to allocate read buffer");
        return;
    }
    
    size_t bytes_read;
    int ret = sensor_manager_read_buffer_bytes(read_buffer, MAX_READ_BYTES, &bytes_read);
    
    if (ret == SENSOR_MANAGER_OK && bytes_read > 0) {
        LOG_INF("Successfully read %zu bytes from shared buffer", bytes_read);
        
        /* Process the raw buffer data */
        process_buffer_data(read_buffer, bytes_read);
        
        /* Show remaining buffer size */
        size_t remaining = sensor_manager_get_buffer_data_available();
        LOG_INF("Remaining bytes in buffer: %zu", remaining);
    } else {
        LOG_WRN("Failed to read from buffer: %d, bytes_read: %zu", ret, bytes_read);
    }
    
    k_free(read_buffer);
    
    /* Exit application after enough callbacks */
    if (buffer_callback_count >= MAX_CALLBACKS) {
        LOG_INF("Reached maximum callbacks (%u), signaling exit", MAX_CALLBACKS);
        k_sem_give(&app_exit_sem);
    }
}

/**
 * @brief Process raw buffer data and parse sensor samples
 * 
 * This function demonstrates how to parse the compact sample format from the shared buffer.
 * Each sample starts with: [device_id][device_location][time_delta][data...]
 */
static void process_buffer_data(uint8_t *buffer, size_t bytes_read)
{
    size_t offset = 0;
    uint32_t sample_count = 0;
    
    LOG_INF("--- Processing Buffer Data ---");
    
    while (offset < bytes_read) {
        /* Make sure we have at least the header */
        if (offset + sizeof(struct sensor_sample_block) > bytes_read) {
            LOG_WRN("Incomplete sample header at offset %zu, stopping", offset);
            break;
        }
        
        struct sensor_sample_block *sample = (struct sensor_sample_block *)(buffer + offset);
        
        LOG_INF("Sample %u: Device ID %u, Location %u, Time Delta %u ms", 
               ++sample_count, sample->device_id, sample->device_location, sample->time_delta);
        
        /* Move past the header */
        offset += sizeof(struct sensor_sample_block);
        
        /* For demonstration, assume each sample has 6 channels (3 accel + 3 gyro) */
        /* In a real application, you'd know the channel configuration */
        size_t data_size = 6 * sizeof(struct sensor_value);
        
        if (offset + data_size <= bytes_read) {
            struct sensor_value *values = (struct sensor_value *)(buffer + offset);
            LOG_INF("  Accel: X=%d.%06d, Y=%d.%06d, Z=%d.%06d", 
                   values[0].val1, values[0].val2,
                   values[1].val1, values[1].val2, 
                   values[2].val1, values[2].val2);
            LOG_INF("  Gyro:  X=%d.%06d, Y=%d.%06d, Z=%d.%06d", 
                   values[3].val1, values[3].val2,
                   values[4].val1, values[4].val2, 
                   values[5].val1, values[5].val2);
            
            offset += data_size;
        } else {
            LOG_WRN("Incomplete sample data at offset %zu, stopping", offset);
            break;
        }
        
        /* Show only first few samples to avoid log spam */
        if (sample_count >= 3) {
            if (offset < bytes_read) {
                LOG_INF("... and more samples (total buffer: %zu bytes)", bytes_read);
            }
            break;
        }
    }
    
    LOG_INF("Processed %u complete samples from %zu bytes", sample_count, offset);
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
            
            /* Add device to sensor manager with device ID 1 and location 0 */
            ret = sensor_manager_add_device(icm_dev, 1, 0, false);
            if (ret == SENSOR_MANAGER_OK) {
                LOG_INF("Added ICM20948 to sensor manager (ID: 1, Location: 0)");
                
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
                
                LOG_INF("ICM20948 setup complete - using shared buffer model");
            } else {
                LOG_ERR("Failed to add ICM20948 to sensor manager: %d", ret);
            }
        } else {
            LOG_WRN("ICM20948 device not ready");
        }
    } else {
        LOG_WRN("ICM20948 device not found");
    }
    
    /* Setup die temperature sensor if available */
    const struct device *temp_dev = DEVICE_DT_GET_OR_NULL(DT_NODELABEL(die_temp));
    if (temp_dev) {
        if (device_is_ready(temp_dev)) {
            LOG_INF("Die temperature sensor found and ready");
            
            /* Add device to sensor manager with device ID 2 and location 1 */
            ret = sensor_manager_add_device(temp_dev, 2, 1, false);
            if (ret == SENSOR_MANAGER_OK) {
                LOG_INF("Added die temperature sensor to sensor manager (ID: 2, Location: 1)");
                
                /* Enable temperature channel */
                sensor_manager_enable_channel(temp_dev, SENSOR_CHAN_DIE_TEMP);
                LOG_INF("Enabled die temperature channel");
                
                LOG_INF("Die temperature sensor setup complete");
            } else {
                LOG_ERR("Failed to add die temperature sensor to sensor manager: %d", ret);
            }
        } else {
            LOG_WRN("Die temperature sensor not ready");
        }
    } else {
        LOG_WRN("Die temperature sensor not found");
    }
    
    return SENSOR_MANAGER_OK;
}

/**
 * @brief Setup application-level buffer threshold callback
 */
static int setup_buffer_threshold_callback(void)
{
    int ret;
    
    LOG_INF("Setting up buffer threshold callback...");
    LOG_INF("Threshold: %u bytes, Max read: %u bytes", BUFFER_THRESHOLD_BYTES, MAX_READ_BYTES);
    
    /* Set up the application-level buffer threshold callback */
    ret = sensor_manager_set_buffer_threshold_callback(BUFFER_THRESHOLD_BYTES, 
                                                      buffer_threshold_callback, 
                                                      NULL);
    
    if (ret != SENSOR_MANAGER_OK) {
        LOG_ERR("Failed to set buffer threshold callback: %d", ret);
        return ret;
    }
    
    LOG_INF("Buffer threshold callback configured successfully");
    return SENSOR_MANAGER_OK;
}

/**
 * @brief Cleanup sensors and manager
 */
static void cleanup_sensors(void)
{
    LOG_INF("Cleaning up sensors...");
    
    /* Stop data acquisition */
    sensor_manager_stop_acquisition_all();
    
    /* Deinitialize sensor manager */
    sensor_manager_deinit();
    
    LOG_INF("Sensor cleanup complete");
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
    printk("   New Buffer Threshold Callback API \n");
    printk("=====================================\n");
    printk("Build: %s %s\n", __DATE__, __TIME__);
    printk("\n");
    
    LOG_INF("Starting Sensor Manager Sample with Buffer Threshold Callbacks");
    
    /* Setup sensors */
    ret = setup_sensors();
    if (ret != SENSOR_MANAGER_OK) {
        LOG_ERR("Sensor setup failed: %d", ret);
        return -1;
    }
    
    /* Setup buffer threshold callback */
    ret = setup_buffer_threshold_callback();
    if (ret != SENSOR_MANAGER_OK) {
        LOG_ERR("Buffer threshold callback setup failed: %d", ret);
        cleanup_sensors();
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
    
    /* Application main loop - wait for buffer threshold callbacks */
    LOG_INF("Waiting for buffer threshold callbacks...");
    printk("Application will exit after %u buffer threshold callbacks\n", MAX_CALLBACKS);
    printk("Buffer threshold: %u bytes, Read size: %u bytes\n\n", 
           BUFFER_THRESHOLD_BYTES, MAX_READ_BYTES);
    
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
