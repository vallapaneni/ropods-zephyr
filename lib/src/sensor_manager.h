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

/* Static memory pool configuration */
/** Size of static memory pool for all ring buffers (in bytes) */
#define SENSOR_MANAGER_MEMORY_POOL_SIZE (32 * 1024)  /* 32KB total pool */

/** Maximum buffer size per device (in bytes) */
#define SENSOR_MANAGER_MAX_BUFFER_SIZE_PER_DEVICE (8 * 1024)  /* 8KB per device max */

/** Memory pool block alignment (must be power of 2) */
#define SENSOR_MANAGER_MEMORY_ALIGN 4

/**
 * @brief Simplified sensor sample block structure with compact addressing
 * 
 * Contains device ID, location, time delta since last sample, followed by variable-length sensor data.
 * The application layer knows which channels are enabled and their order.
 * Layout in buffer: [device_id][device_location][time_delta][channel_data][channel_data]...
 */
struct __packed sensor_sample_block {
    uint8_t device_id;              /**< Device identifier (1 byte) */
    uint8_t device_location;        /**< Device location identifier (1 byte) */
    uint16_t time_delta;            /**< Time elapsed since last sample in milliseconds (2 bytes) */
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
    uint8_t device_id;                                                    /**< Device identifier (set by application) */
    uint8_t device_location;                                              /**< Device location identifier (set by application) */
    bool active;                                                          /**< Device active status */
    bool samples_prefetched;                                              /**< Some device drivers prefetch the data */
    bool acquisition_active;                                              /**< Data acquisition in progress */
    
    /* Channel configuration - fixed once acquisition starts */
    struct sensor_channel_config channels[SENSOR_MANAGER_MAX_CHANNELS_PER_DEVICE];
    uint8_t num_enabled_channels;                                         /**< Number of enabled channels */
    size_t sample_block_size;                                             /**< Pre-computed fixed sample size */
    
    /* Trigger configuration */
    struct sensor_trigger trigger;                                        /**< Data ready trigger */

    /* Statistics */
    uint32_t data_ready_count;                                            /**< Count of data ready events */
    
    /* Device-specific timestamp for time delta calculation */
    uint32_t last_sample_timestamp_ms;                                    /**< Last sample timestamp for this device */
};

/**
 * @brief Application-level trigger callback function type
 * 
 * Called when shared buffer data size exceeds the configured threshold.
 * Application should read data from the shared buffer in response.
 * 
 * @param available_bytes Number of bytes available in shared buffer
 * @param user_data User data pointer passed during callback setup
 */
typedef void (*sensor_manager_buffer_callback_t)(size_t available_bytes, void *user_data);

/**
 * @brief Sensor manager context structure
 */
struct sensor_manager {
    struct sensor_device_info devices[SENSOR_MANAGER_MAX_DEVICES];        /**< Array of managed devices */
    uint8_t num_devices;                                                  /**< Number of registered devices */
    bool initialized;                                                     /**< Initialization status */
    bool acquisition_active;                                              /**< Global acquisition state */
    
    /* Shared ring buffer for all device data */
    struct ring_buf shared_data_buffer;                                   /**< Shared ring buffer for all devices */
    struct k_mutex shared_buffer_mutex;                                   /**< Mutex for shared buffer access */
    
    /* Static memory pool for ring buffers */
    uint8_t memory_pool[SENSOR_MANAGER_MEMORY_POOL_SIZE];                 /**< Static memory pool */
    uint8_t *allocated_memory;                                            /**< Pointer to allocated block (or NULL) */
    size_t allocated_size;                                                /**< Size of allocated block */
    
    /* Application-level buffer threshold triggering */
    size_t buffer_threshold_bytes;                                        /**< Buffer threshold in bytes */
    sensor_manager_buffer_callback_t buffer_callback;                     /**< Buffer threshold callback */
    void *buffer_callback_user_data;                                     /**< User data for buffer callback */
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
    SENSOR_MANAGER_ERROR_MEMORY_POOL_FULL,    /**< Static memory pool exhausted */
    SENSOR_MANAGER_ERROR_TRIGGER_SETUP,       /**< Trigger setup failed */
    SENSOR_MANAGER_ERROR_ACQUISITION_ACTIVE,  /**< Cannot change channels during acquisition */
    SENSOR_MANAGER_ERROR_NOT_SUPPORTED,       /**< Function not supported in current design */
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
 * @param sensor_id Sensor identifier (1-255, 0 is reserved)
 * @param sensor_location Sensor location identifier
 * @param samples_prefetched Whether this device pre-fetches samples
 * @return SENSOR_MANAGER_OK on success, error code otherwise
 */
int sensor_manager_add_device(const struct device *device, uint8_t device_id, uint8_t device_location, bool samples_prefetched);

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
 * @brief Start data acquisition for all managed sensor devices
 * 
 * Allocates memory from the static pool for all devices and starts
 * data acquisition. This must be called after all devices have been
 * added and their channels configured.
 * 
 * @return SENSOR_MANAGER_OK on success, error code otherwise
 */
int sensor_manager_start_acquisition_all(size_t pool_size);

/**
 * @brief Stop data acquisition for all managed sensor devices
 * 
 * Stops data acquisition for all devices and frees all allocated
 * memory back to the static pool.
 * 
 * @return SENSOR_MANAGER_OK on success, error code otherwise
 */
int sensor_manager_stop_acquisition_all(void);

/**
 * @brief Clear the shared data buffer (all devices)
 * 
 * @return SENSOR_MANAGER_OK on success, error code otherwise
 */
int sensor_manager_clear_shared_buffer(void);

/**
 * @brief Get the sensor ID for a managed sensor device
 * 
 * @param device Pointer to the sensor device
 * @return Sensor ID, or 0 if device is not managed
 */
uint8_t sensor_manager_get_sensor_id(const struct device *device);

/**
 * @brief Get the data size in bytes for a specific sensor channel
 *
 * Tries to get the actual data length using SENSOR_ATTR_DATA_LENGTH, or falls back to sizeof(struct sensor_value).
 *
 * @param device Pointer to the sensor device
 * @param channel The sensor channel to query
 * @return Size in bytes of the channel data
 */
size_t sensor_manager_get_channel_data_size(const struct device *device, enum sensor_channel channel);

/**
 * @brief Get the sensor manager instance (for internal use)
 * 
 * @return Pointer to the sensor manager instance
 */
struct sensor_manager *sensor_manager_get_instance(void);

/**
 * @brief Set up application-level trigger based on shared buffer data availability
 * 
 * Sets up a callback that will be triggered when the amount of data in the shared
 * buffer exceeds the specified threshold. This replaces device-level triggers
 * with application-level buffer management.
 * 
 * @param threshold_bytes Minimum number of bytes in buffer to trigger callback
 * @param callback Callback function to call when threshold is exceeded
 * @param user_data User data pointer to pass to callback
 * @return SENSOR_MANAGER_OK on success, error code otherwise
 */
int sensor_manager_set_buffer_threshold_callback(size_t threshold_bytes,
                                                sensor_manager_buffer_callback_t callback,
                                                void *user_data);

/**
 * @brief Read data from shared buffer up to specified byte count
 * 
 * Reads raw bytes from the shared buffer for application-level parsing.
 * Applications can use this to read exactly the amount of data they want
 * to process based on the buffer threshold callback.
 * 
 * @param data_ptr Pointer to store the address of the data in the ring buffer
 * @param max_bytes Maximum number of bytes to read
 * @param bytes_read Pointer to store actual number of bytes read
 * @return SENSOR_MANAGER_OK on success, error code otherwise
 */
int sensor_manager_read_buffer_bytes(uint8_t **data_ptr, size_t max_bytes, size_t *bytes_read);

/**
 * @brief Get current number of bytes available in shared buffer
 * 
 * @return Number of bytes currently available to read
 */
size_t sensor_manager_get_buffer_data_available(void);

#ifdef __cplusplus
}
#endif

#endif /* SENSOR_MANAGER_H */
