/**
 * @file sensor_manager.h
 * @brief Sensor Manager for handling multiple sensor devices with enabled channels
 * 
 * This module provides a comprehensive sensor management system that can:
 * - Handle multiple sensor devices simultaneously
 * - Manage enabled channels per device
 * - Set up data ready trigger callbacks
 * - Acquire and buffer timestamped sensor data
 * - Provide thread-safe access to sensor data
 */

#ifndef SENSOR_MANAGER_H
#define SENSOR_MANAGER_H

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/sys/ring_buffer.h>

#ifdef __cplusplus
extern "C" {
#endif

/** Maximum number of sensor devices that can be managed */
#define SENSOR_MANAGER_MAX_DEVICES 8

/** Maximum number of channels per sensor device */
#define SENSOR_MANAGER_MAX_CHANNELS_PER_DEVICE 16

/** Default buffer size for each sensor device (number of data entries) */
#define SENSOR_MANAGER_DEFAULT_BUFFER_SIZE 64

/** Maximum length for sensor device name */
#define SENSOR_MANAGER_MAX_NAME_LEN 32

/**
 * @brief Simplified sensor sample block structure with variable-length data
 * 
 * Contains timestamp followed by variable-length sensor data in the order channels were enabled.
 * The application layer knows which channels are enabled and their order.
 * Each channel may have different data lengths (1 or more sensor_value structures).
 * Layout in buffer: [timestamp_ms][channel_data][channel_data]... (variable lengths, fixed order)
 */
struct __packed sensor_sample_block {
    uint32_t timestamp_ms;          /**< Timestamp in milliseconds */
    uint8_t data[];                 /**< Variable-length sensor data in enabled channel order */
};

/**
 * @brief Structure to manage enabled channels for a sensor device
 */
struct sensor_channel_config {
    enum sensor_channel channel;    /**< Channel identifier */
    bool enabled;                   /**< Channel enable status */
};

/**
 * @brief Structure to hold sensor device configuration and data
 */
struct sensor_device_info {
    const struct device *device;                                           /**< Pointer to sensor device */
    char name[SENSOR_MANAGER_MAX_NAME_LEN];                               /**< Device name */
    bool active;                                                          /**< Device active status */
    bool samples_prefetched;                                              /**< Some device drivers prefetch the data */
    bool acquisition_active;                                              /**< Data acquisition in progress */
    
    /* Channel configuration - fixed once acquisition starts */
    struct sensor_channel_config channels[SENSOR_MANAGER_MAX_CHANNELS_PER_DEVICE];
    uint8_t num_enabled_channels;                                         /**< Number of enabled channels */
    size_t sample_block_size;                                             /**< Pre-computed fixed sample size */
    
    /* Trigger configuration */
    struct sensor_trigger trigger;                                        /**< Data ready trigger */
    sensor_trigger_handler_t callback;                                    /**< User callback function */
    
    /* Data buffer */
    struct ring_buf data_buffer;                                          /**< Ring buffer for sensor data */
    uint8_t *buffer_memory;                                               /**< Memory for ring buffer */
    size_t buffer_size;                                                   /**< Buffer size in bytes */
    struct k_mutex buffer_mutex;                                          /**< Mutex for buffer access */
    
    /* Statistics */
    uint32_t data_ready_count;                                            /**< Count of data ready events */
    uint32_t buffer_overflow_count;                                       /**< Count of buffer overflows */
};

/**
 * @brief Sensor manager context structure
 */
struct sensor_manager {
    struct sensor_device_info devices[SENSOR_MANAGER_MAX_DEVICES];        /**< Array of managed devices */
    uint8_t num_devices;                                                  /**< Number of registered devices */
    struct k_mutex manager_mutex;                                         /**< Mutex for manager operations */
    bool initialized;                                                     /**< Initialization status */
};

/**
 * @brief Sensor manager error codes
 */
enum sensor_manager_error {
    SENSOR_MANAGER_OK = 0,                    /**< Success */
    SENSOR_MANAGER_ERROR_INVALID_PARAM,       /**< Invalid parameter */
    SENSOR_MANAGER_ERROR_DEVICE_NOT_FOUND,    /**< Device not found */
    SENSOR_MANAGER_ERROR_DEVICE_NOT_READY,    /**< Device not ready */
    SENSOR_MANAGER_ERROR_MAX_DEVICES,         /**< Maximum devices reached */
    SENSOR_MANAGER_ERROR_MAX_CHANNELS,        /**< Maximum channels reached */
    SENSOR_MANAGER_ERROR_BUFFER_FULL,         /**< Buffer is full */
    SENSOR_MANAGER_ERROR_NOT_INITIALIZED,     /**< Manager not initialized */
    SENSOR_MANAGER_ERROR_MEMORY_ALLOC,        /**< Memory allocation failed */
    SENSOR_MANAGER_ERROR_TRIGGER_SETUP,       /**< Trigger setup failed */
    SENSOR_MANAGER_ERROR_ACQUISITION_ACTIVE,  /**< Cannot change channels during acquisition */
};

/**
 * @brief Initialize the sensor manager
 * 
 * @return SENSOR_MANAGER_OK on success, error code otherwise
 */
int sensor_manager_init(void);

/**
 * @brief Deinitialize the sensor manager
 * 
 * @return SENSOR_MANAGER_OK on success, error code otherwise
 */
int sensor_manager_deinit(void);

/**
 * @brief Add a sensor device to the manager
 * 
 * @param device Pointer to the sensor device
 * @param name Human-readable name for the device
 * @param samples_prefetched Whether this device pre-fetches samples
 * @param buffer_size Size of the data buffer (number of sample blocks, 0 for default)
 * @return SENSOR_MANAGER_OK on success, error code otherwise
 */
int sensor_manager_add_device(const struct device *device, const char *name, bool samples_prefetched, size_t buffer_size);

/**
 * @brief Remove a sensor device from the manager
 * 
 * @param device Pointer to the sensor device to remove
 * @return SENSOR_MANAGER_OK on success, error code otherwise
 */
int sensor_manager_remove_device(const struct device *device);

/**
 * @brief Enable a channel for a specific sensor device
 * 
 * @param device Pointer to the sensor device
 * @param channel Channel to enable
 * @return SENSOR_MANAGER_OK on success, error code otherwise
 */
int sensor_manager_enable_channel(const struct device *device, enum sensor_channel channel);

/**
 * @brief Disable a channel for a specific sensor device
 * 
 * @param device Pointer to the sensor device
 * @param channel Channel to disable
 * @return SENSOR_MANAGER_OK on success, error code otherwise
 */
int sensor_manager_disable_channel(const struct device *device, enum sensor_channel channel);

/**
 * @brief Set data ready trigger callback for a sensor device
 * 
 * @param device Pointer to the sensor device
 * @param trigger_type Type of trigger (e.g., SENSOR_TRIG_DATA_READY)
 * @param callback Callback function to be called on trigger
 * @return SENSOR_MANAGER_OK on success, error code otherwise
 */
int sensor_manager_set_trigger_callback(const struct device *device, 
                                       enum sensor_trigger_type trigger_type,
                                       sensor_trigger_handler_t callback);

/**
 * @brief Start data acquisition for a sensor device
 * 
 * @param device Pointer to the sensor device
 * @return SENSOR_MANAGER_OK on success, error code otherwise
 */
int sensor_manager_start_acquisition(const struct device *device);

/**
 * @brief Stop data acquisition for a sensor device
 * 
 * @param device Pointer to the sensor device
 * @return SENSOR_MANAGER_OK on success, error code otherwise
 */
int sensor_manager_stop_acquisition(const struct device *device);

/**
 * @brief Get the latest sensor data for a specific channel
 * 
 * @param device Pointer to the sensor device
 * @param channel Channel to read from
 * @param timestamp_ms Pointer to store the timestamp in milliseconds
 * @param value Pointer to store the sensor value
 * @return SENSOR_MANAGER_OK on success, error code otherwise
 */
int sensor_manager_get_latest_data(const struct device *device, 
                                  enum sensor_channel channel,
                                  uint32_t *timestamp_ms,
                                  struct sensor_value *value);

/**
 * @brief Read multiple sensor sample blocks from buffer
 * 
 * @param device Pointer to the sensor device
 * @param data Array to store sensor sample blocks
 * @param max_blocks Maximum number of sample blocks to read
 * @param blocks_read Pointer to store actual number of blocks read
 * @return SENSOR_MANAGER_OK on success, error code otherwise
 */
int sensor_manager_read_data_buffer(const struct device *device,
                                   struct sensor_sample_block *data,
                                   size_t max_blocks,
                                   size_t *blocks_read);

/**
 * @brief Clear the data buffer for a sensor device
 * 
 * @param device Pointer to the sensor device
 * @return SENSOR_MANAGER_OK on success, error code otherwise
 */
int sensor_manager_clear_buffer(const struct device *device);

/**
 * @brief Get statistics for a sensor device
 * 
 * @param device Pointer to the sensor device
 * @param data_ready_count Pointer to store data ready event count
 * @param buffer_overflow_count Pointer to store buffer overflow count
 * @param buffer_usage Pointer to store current buffer usage (0-100%)
 * @return SENSOR_MANAGER_OK on success, error code otherwise
 */
int sensor_manager_get_stats(const struct device *device,
                            uint32_t *data_ready_count,
                            uint32_t *buffer_overflow_count,
                            uint8_t *buffer_usage);

/**
 * @brief Get list of enabled channels for a sensor device
 * 
 * @param device Pointer to the sensor device
 * @param channels Array to store enabled channels
 * @param max_channels Maximum number of channels to store
 * @param num_channels Pointer to store actual number of enabled channels
 * @return SENSOR_MANAGER_OK on success, error code otherwise
 */
int sensor_manager_get_enabled_channels(const struct device *device,
                                       enum sensor_channel *channels,
                                       size_t max_channels,
                                       size_t *num_channels);

/**
 * @brief Get the data size for a specific sensor channel
 * 
 * This function queries the sensor device to determine how many bytes
 * of data a specific channel provides. This is useful for applications
 * that need to handle variable-sized sensor data.
 * 
 * @param device Pointer to the sensor device
 * @param channel The sensor channel to query
 * @return Size in bytes of the channel data, or sizeof(struct sensor_value) as fallback
 */
size_t sensor_manager_get_channel_data_size(const struct device *device, enum sensor_channel channel);

/**
 * @brief Check if a sensor device is managed by the sensor manager
 * 
 * @param device Pointer to the sensor device
 * @return true if device is managed, false otherwise
 */
bool sensor_manager_is_device_managed(const struct device *device);

/**
 * @brief Get the sensor manager instance (for internal use)
 * 
 * @return Pointer to the sensor manager instance
 */
struct sensor_manager *sensor_manager_get_instance(void);

#ifdef __cplusplus
}
#endif

#endif /* SENSOR_MANAGER_H */
