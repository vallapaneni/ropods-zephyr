/**
 * @file main.c
 * @brief Enhanced Sensor Manager Sample with BLE Quaternion Streaming
 * 
 * This sample demonstrates:
 * - Application-level buffer threshold callback system
 * - ICM20948 quaternion (9-axis) data acquisition 
 * - BLE streaming of quaternion + IMU data in 10-sample packets
 * - Real-time sensor data processing and transmission
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/printk.h>
#include <zephyr/sys/util.h>

#include "sensor_manager.h"
#include "ble_sensor_service.h"
#include "icm20948.h"

LOG_MODULE_REGISTER(sensor_manager_sample, LOG_LEVEL_INF);

/* Sample configuration */
#define BUFFER_THRESHOLD_BYTES 512     /* Trigger callback when buffer has 512+ bytes */
#define MAX_READ_BYTES 512             /* Read up to 512 bytes per callback (more for quat data) */
#define MAX_CALLBACKS 100              /* Run longer for BLE streaming demo */
#define SAMPLES_PER_BLE_PACKET 10      /* Pack 10 samples per BLE transmission */

/* Application state */
static K_SEM_DEFINE(app_exit_sem, 0, 1);
static uint32_t buffer_callback_count = 0;
static bool ble_streaming_active = false;

/* BLE packet management */
static uint8_t ble_packet_buffer[sizeof(struct sensor_data_packet) + (SAMPLES_PER_BLE_PACKET * 11 * sizeof(float))];
static struct sensor_data_packet *current_ble_packet = (struct sensor_data_packet *)ble_packet_buffer;
static uint8_t samples_in_packet = 0;
static uint32_t total_samples_sent = 0;
static uint32_t total_packets_sent = 0;

/* Forward declarations */
static void buffer_threshold_callback(size_t available_bytes, void *user_data);
static void process_buffer_data(uint8_t *buffer, size_t bytes_read);
static int control_command_handler(const struct control_command *cmd, struct control_response *resp);
static int send_ble_packet_if_ready(void);
static void convert_sensor_value_to_float(const struct sensor_value *val, float *result);

/**
 * @brief Convert sensor_value to float for BLE transmission
 */
static void convert_sensor_value_to_float(const struct sensor_value *val, float *result)
{
	*result = (float)val->val1 + (float)val->val2 / 1000000.0f;
}

/**
 * @brief Send BLE packet when we have enough samples
 */
static int send_ble_packet_if_ready(void)
{
	if (samples_in_packet >= SAMPLES_PER_BLE_PACKET && ble_sensor_is_connected()) {
		current_ble_packet->sample_count = samples_in_packet;
		
		/* Calculate packet size based on actual data */
		size_t packet_size = sizeof(struct sensor_data_packet) + 
				     samples_in_packet * 11 * sizeof(float); /* timestamp + quat + accel + gyro */
		
		int ret = ble_sensor_send_data_packet(current_ble_packet, packet_size);
		if (ret == 0) {
			total_packets_sent++;
			total_samples_sent += samples_in_packet;
			
			LOG_INF("Sent BLE packet %u with %d samples (total: %u samples)",
				total_packets_sent, samples_in_packet, total_samples_sent);
		} else {
			LOG_WRN("Failed to send BLE packet: %d", ret);
		}
		
		/* Reset packet for next batch */
		samples_in_packet = 0;
		current_ble_packet->packet_id++;
		
		return ret;
	}
	
	return 0;
}

/**
 * @brief Generic control command handler - supports attribute set/get and channel control
 */
static int control_command_handler(const struct control_command *cmd, struct control_response *resp)
{
	LOG_INF("Control command: 0x%02x, device=%d, channel/attr=0x%04x, value=%u", 
		cmd->command, cmd->device_id, cmd->channel_or_attr, cmd->value);
	
	/* Get the ICM20948 device for sensor operations */
	const struct device *icm_dev = DEVICE_DT_GET_ONE(invensense_icm20948);
	if (!device_is_ready(icm_dev)) {
		LOG_ERR("ICM20948 device not ready");
		return -ENODEV;
	}
	
	switch (cmd->command) {
	case CTRL_CMD_CHANNEL_ENABLE:
		LOG_INF("Enable channel %d on device %d", cmd->channel_or_attr, cmd->device_id);
		if (cmd->device_id == 0 || cmd->device_id == 1) {
			int ret = sensor_manager_enable_channel(icm_dev, (enum sensor_channel)cmd->channel_or_attr);
			if (ret != SENSOR_MANAGER_OK) {
				LOG_ERR("Failed to enable channel: %d", ret);
				return -EIO;
			}
		}
		break;
		
	case CTRL_CMD_CHANNEL_DISABLE:
		LOG_INF("Disable channel %d on device %d", cmd->channel_or_attr, cmd->device_id);
		if (cmd->device_id == 0 || cmd->device_id == 1) {
			int ret = sensor_manager_disable_channel(icm_dev, (enum sensor_channel)cmd->channel_or_attr);
			if (ret != SENSOR_MANAGER_OK) {
				LOG_ERR("Failed to disable channel: %d", ret);
				return -EIO;
			}
		}
		break;
		
	case CTRL_CMD_SET_ATTRIBUTE:
		LOG_INF("Set attribute 0x%04x to %u on device %d", cmd->channel_or_attr, cmd->value, cmd->device_id);
		
		switch (cmd->channel_or_attr) {
		case ATTR_SAMPLE_RATE:
			LOG_INF("Setting sample rate to %u Hz", cmd->value);
			/* Set sample rate using sensor API */
			{
				struct sensor_value odr_val = {.val1 = cmd->value, .val2 = 0};
				int ret = sensor_attr_set(icm_dev, SENSOR_CHAN_ACCEL_XYZ, SENSOR_ATTR_SAMPLING_FREQUENCY, &odr_val);
				if (ret != 0) {
					LOG_ERR("Failed to set accel sample rate: %d", ret);
					return -EIO;
				}
				ret = sensor_attr_set(icm_dev, SENSOR_CHAN_GYRO_XYZ, SENSOR_ATTR_SAMPLING_FREQUENCY, &odr_val);
				if (ret != 0) {
					LOG_ERR("Failed to set gyro sample rate: %d", ret);
					return -EIO;
				}
			}
			break;
			
		case ATTR_FULL_SCALE_RANGE:
			LOG_INF("Setting full scale range to %u", cmd->value);
			/* Set full scale range using sensor API */
			{
				struct sensor_value fs_val = {.val1 = cmd->value, .val2 = 0};
				int ret = sensor_attr_set(icm_dev, SENSOR_CHAN_ACCEL_XYZ, SENSOR_ATTR_FULL_SCALE, &fs_val);
				if (ret != 0) {
					LOG_ERR("Failed to set accel full scale: %d", ret);
					return -EIO;
				}
				ret = sensor_attr_set(icm_dev, SENSOR_CHAN_GYRO_XYZ, SENSOR_ATTR_FULL_SCALE, &fs_val);
				if (ret != 0) {
					LOG_ERR("Failed to set gyro full scale: %d", ret);
					return -EIO;
				}
			}
			break;
			
		case ATTR_POWER_MODE:
			LOG_INF("Setting power mode to %u (not implemented)", cmd->value);
			/* Power mode setting would depend on sensor driver */
			break;
			
		default:
			LOG_WRN("Unknown attribute ID: 0x%04x", cmd->channel_or_attr);
			return -ENOTSUP;
		}
		break;
		
	case CTRL_CMD_GET_ATTRIBUTE:
		LOG_INF("Get attribute 0x%04x from device %d", cmd->channel_or_attr, cmd->device_id);
		
		switch (cmd->channel_or_attr) {
		case ATTR_SAMPLE_RATE:
		{
			struct sensor_value odr_val;
			int ret = sensor_attr_get(icm_dev, SENSOR_CHAN_ACCEL_XYZ, SENSOR_ATTR_SAMPLING_FREQUENCY, &odr_val);
			if (ret == 0) {
				resp->value = (uint32_t)odr_val.val1;
				LOG_INF("Sample rate: %u Hz", resp->value);
			} else {
				LOG_ERR("Failed to get sample rate: %d", ret);
				return -EIO;
			}
		}
		break;
		
		case ATTR_FULL_SCALE_RANGE:
		{
			struct sensor_value fs_val;
			int ret = sensor_attr_get(icm_dev, SENSOR_CHAN_ACCEL_XYZ, SENSOR_ATTR_FULL_SCALE, &fs_val);
			if (ret == 0) {
				resp->value = (uint32_t)fs_val.val1;
				LOG_INF("Full scale range: %u", resp->value);
			} else {
				LOG_ERR("Failed to get full scale: %d", ret);
				return -EIO;
			}
		}
		break;
		
		default:
			LOG_WRN("Unknown attribute ID: 0x%04x", cmd->channel_or_attr);
			return -ENOTSUP;
		}
		break;
		
	case CTRL_CMD_START_STREAMING:
		ble_streaming_active = true;
		LOG_INF("BLE streaming started by client");
		break;
		
	case CTRL_CMD_STOP_STREAMING:
		ble_streaming_active = false;
		LOG_INF("BLE streaming stopped by client");
		break;
		
	case CTRL_CMD_RESET_COUNTERS:
		total_samples_sent = 0;
		total_packets_sent = 0;
		current_ble_packet->packet_id = 0;
		LOG_INF("BLE counters reset by client");
		break;
		
	default:
		LOG_WRN("Unknown control command: 0x%02x", cmd->command);
		return -ENOTSUP;
	}
	
	return 0;
}

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
 * @brief Process raw buffer data and extract quaternion + IMU data for BLE streaming
 * 
 * This function demonstrates parsing the compact sample format and converting
 * to the generic sensor data packet format for BLE transmission.
 */
static void process_buffer_data(uint8_t *buffer, size_t bytes_read)
{
	size_t offset = 0;
	uint32_t sample_count = 0;
	
	LOG_DBG("--- Processing Buffer Data for BLE Streaming ---");
	
	while (offset < bytes_read) {
		/* Make sure we have at least the header */
		if (offset + sizeof(struct sensor_sample_block) > bytes_read) {
			LOG_WRN("Incomplete sample header at offset %zu, stopping", offset);
			break;
		}
		
		struct sensor_sample_block *sample = (struct sensor_sample_block *)(buffer + offset);
		
		/* Only process ICM20948 data (device ID 1) */
		if (sample->device_id != 1) {
			LOG_DBG("Skipping non-ICM sample from device %d", sample->device_id);
			offset += sizeof(struct sensor_sample_block);
			/* Skip the data portion - we need to know the size for this device */
			continue;
		}
		
		LOG_DBG("Processing sample %u: Device ID %u, Location %u, Time Delta %u ms", 
		       ++sample_count, sample->device_id, sample->device_location, sample->time_delta);
		
		/* Initialize BLE packet if this is the first sample */
		if (samples_in_packet == 0) {
			current_ble_packet->device_id = sample->device_id;
			current_ble_packet->device_location = sample->device_location;
		}
		
		/* Move past the header */
		offset += sizeof(struct sensor_sample_block);
		
		/* For ICM20948, we expect 10 channels of quaternion + IMU data:
		 * - Game rotation vector quaternion (w, x, y, z) - 4 values
		 * - Accelerometer (x, y, z) - 3 values  
		 * - Gyroscope (x, y, z) - 3 values
		 * Total: 10 sensor_value structures = 40 floats in the BLE packet
		 */
		size_t expected_data_size = 10 * sizeof(struct sensor_value);
		
		if (offset + expected_data_size <= bytes_read && samples_in_packet < SAMPLES_PER_BLE_PACKET) {
			struct sensor_value *values = (struct sensor_value *)(buffer + offset);
			
			/* Calculate position in the BLE packet data array */
			/* Each sample contains 11 floats (timestamp + quat + accel + gyro) */
			size_t sample_data_offset = samples_in_packet * 11 * sizeof(float);
			float *sample_data = (float *)(current_ble_packet->data + sample_data_offset);
			
			/* Add timestamp as first value (converted to float) */
			sample_data[0] = (float)sample->time_delta;
			
			/* Extract quaternion data (next 4 values) */
			convert_sensor_value_to_float(&values[0], &sample_data[1]); /* quat_w */
			convert_sensor_value_to_float(&values[1], &sample_data[2]); /* quat_x */
			convert_sensor_value_to_float(&values[2], &sample_data[3]); /* quat_y */
			convert_sensor_value_to_float(&values[3], &sample_data[4]); /* quat_z */
			
			/* Extract accelerometer data (next 3 values) */
			convert_sensor_value_to_float(&values[4], &sample_data[5]); /* accel_x */
			convert_sensor_value_to_float(&values[5], &sample_data[6]); /* accel_y */
			convert_sensor_value_to_float(&values[6], &sample_data[7]); /* accel_z */
			
			/* Extract gyroscope data (last 3 values) */
			convert_sensor_value_to_float(&values[7], &sample_data[8]); /* gyro_x */
			convert_sensor_value_to_float(&values[8], &sample_data[9]); /* gyro_y */
			convert_sensor_value_to_float(&values[9], &sample_data[10]); /* gyro_z */
			
			samples_in_packet++;
			
			LOG_DBG("Added sample %d to BLE packet (quat: %.3f,%.3f,%.3f,%.3f)", 
			       samples_in_packet, sample_data[1], sample_data[2], sample_data[3], sample_data[4]);
		}
		
		/* Move to next sample */
		offset += expected_data_size;
		
		/* Send BLE packet if we have enough samples */
		if (ble_streaming_active) {
			send_ble_packet_if_ready();
		}
	}
	
	LOG_DBG("Processed %u samples, %d samples in current BLE packet", sample_count, samples_in_packet);
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
                
                /* Enable quaternion channels (game rotation vector: w, x, y, z) */
                sensor_manager_enable_channel(icm_dev, SENSOR_CHAN_ICM20948_GAME_ROTATION_VECTOR);
                LOG_INF("Enabled game rotation vector (quaternion) channel");
                
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
                
                LOG_INF("ICM20948 setup complete - quaternion + 9-axis IMU data enabled");
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
 * @brief Setup BLE service
 */
static int setup_ble_service(void)
{
    int ret;
    
    LOG_INF("Setting up BLE sensor service...");
    
    /* Initialize BLE service */
    ret = ble_sensor_service_init();
    if (ret != 0) {
        LOG_ERR("Failed to initialize BLE service: %d", ret);
        return ret;
    }
    
    /* Register control command callback */
    ble_sensor_register_control_callback(control_command_handler);
    
    /* Setup device information */
    struct device_info devices[] = {
        {
            .device_id = 1,
            .device_location = 0,
            .device_type = 1, /* IMU */
            .num_channels = 10, /* quaternion (4) + accel (3) + gyro (3) */
            .device_name = "ICM20948"
        },
        {
            .device_id = 2,
            .device_location = 1,
            .device_type = 2, /* Temperature */
            .num_channels = 1,
            .device_name = "Die_Temp"
        }
    };
    
    ret = ble_sensor_update_device_info(devices, ARRAY_SIZE(devices));
    if (ret != 0) {
        LOG_ERR("Failed to update device info: %d", ret);
        return ret;
    }
    
    /* Start advertising */
    ret = ble_sensor_start_advertising();
    if (ret != 0) {
        LOG_ERR("Failed to start BLE advertising: %d", ret);
        return ret;
    }
    
    LOG_INF("BLE sensor service setup complete - advertising started");
    return 0;
}
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
    
    /* Setup BLE service */
    ret = setup_ble_service();
    if (ret != 0) {
        LOG_ERR("BLE service setup failed: %d", ret);
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
