/**
 * @file sensor_manager.c
 * @brief Implementation of the Sensor Manager
 */

#include "sensor_manager.h"
#include <zephyr/logging/log.h>
#include <zephyr/sys/printk.h>
#include <string.h>
#include <stdlib.h>

/* Define SENSOR_ATTR_DATA_LENGTH if not available in Zephyr */
#ifndef SENSOR_ATTR_DATA_LENGTH
#define SENSOR_ATTR_DATA_LENGTH 118
#endif

LOG_MODULE_REGISTER(sensor_manager, CONFIG_SENSOR_MANAGER_LOG_LEVEL);

/* Forward declarations */
static void check_buffer_threshold(void);

/* Global sensor manager instance */
static struct sensor_manager g_sensor_manager = {0};

/**
 * @brief Internal function to find a device info structure
 */
static struct sensor_device_info *find_device_info(const struct device *device)
{
    if (!device) {
        return NULL;
    }

    for (int i = 0; i < g_sensor_manager.num_devices; i++) {
        if (g_sensor_manager.devices[i].device == device && 
            g_sensor_manager.devices[i].active) {
            return &g_sensor_manager.devices[i];
        }
    }
    return NULL;
}

/**
 * @brief Internal function to get the data size for a specific sensor channel
 * 
 * @param device Pointer to the sensor device
 * @param channel The sensor channel to query
 * @return Size in bytes of the channel data, or sizeof(struct sensor_value) as fallback
 */
static size_t get_sensor_channel_data_size(const struct device *device, enum sensor_channel channel)
{
    if (!device) {
        return sizeof(struct sensor_value);
    }

    /* Try to get actual data size from sensor attribute */
    struct sensor_value data_length;
    int ret = sensor_attr_get(device, channel, SENSOR_ATTR_DATA_LENGTH, &data_length);
    
    if (ret == 0 && data_length.val1 > 0) {
        /* Use actual data length from sensor (number of sensor_value elements) */
        return data_length.val1 * sizeof(struct sensor_value);
    }
    
    /* Fallback: assume single sensor_value for unknown channels */
    LOG_DBG("Could not get data length for channel %d, assuming 1 sensor_value", channel);
    return sizeof(struct sensor_value);
}

/**
 * @brief Allocate memory from the static memory pool for all devices
 * 
 * @param size Total size in bytes to allocate
 * @return Pointer to allocated memory, or NULL if allocation failed
 */
static uint8_t *sensor_manager_pool_alloc_all(size_t size)
{
    if (size == 0 || size > SENSOR_MANAGER_MEMORY_POOL_SIZE) {
        LOG_ERR("Invalid allocation size: %zu (max: %d)", size, SENSOR_MANAGER_MEMORY_POOL_SIZE);
        return NULL;
    }

    /* Check if memory is already allocated */
    if (g_sensor_manager.allocated_memory != NULL) {
        LOG_ERR("Memory already allocated, call stop_acquisition_all first");
        return NULL;
    }

    /* Align size to memory alignment boundary */
    size_t aligned_size = (size + SENSOR_MANAGER_MEMORY_ALIGN - 1) & ~(SENSOR_MANAGER_MEMORY_ALIGN - 1);
    
    /* Allocate from beginning of pool */
    g_sensor_manager.allocated_memory = g_sensor_manager.memory_pool;
    g_sensor_manager.allocated_size = aligned_size;

    LOG_DBG("Allocated %zu bytes (aligned from %zu) from memory pool", aligned_size, size);
    return g_sensor_manager.allocated_memory;
}

/**
 * @brief Free all memory back to the static memory pool
 */
static void sensor_manager_pool_free_all(void)
{
    if (g_sensor_manager.allocated_memory != NULL) {
        LOG_DBG("Freed %zu bytes back to memory pool", g_sensor_manager.allocated_size);
        g_sensor_manager.allocated_memory = NULL;
        g_sensor_manager.allocated_size = 0;
    }
}

/**
 * @brief Internal data ready callback handler with shared buffer and compact sample format
 */
static void internal_data_ready_handler(const struct device *device,
                                       const struct sensor_trigger *trigger)
{
    struct sensor_device_info *dev_info = find_device_info(device);
    if (!dev_info) {
        LOG_ERR("Device not found in data ready handler");
        return;
    }

    uint32_t current_timestamp_ms = (uint32_t)(k_uptime_get() / 1000); /* Convert μs to ms */
    uint16_t time_delta;
    int ret;

    /* Calculate time delta since last sample for this device (16-bit, wraps at ~65 seconds) */
    if (dev_info->last_sample_timestamp_ms == 0) {
        time_delta = 0; /* First sample for this device */
    } else {
        uint32_t delta = current_timestamp_ms - dev_info->last_sample_timestamp_ms;
        time_delta = (uint16_t)delta; /* Provide least 16 bits; any wraps can be detected by app */
    }
    dev_info->last_sample_timestamp_ms = current_timestamp_ms;

    /* Increment data ready counter */
    dev_info->data_ready_count++;

    /* Fetch sensor sample if not pre-fetched */
    if (!dev_info->samples_prefetched) {
        ret = sensor_sample_fetch(device);
        if (ret != 0) {
            LOG_ERR("Failed to fetch sensor sample: %d", ret);
            return;
        }
    }

    /* Lock shared buffer for data storage */
    k_mutex_lock(&g_sensor_manager.shared_buffer_mutex, K_FOREVER);

    /* Use pre-computed fixed sample block size */
    size_t sample_size = dev_info->sample_block_size;

    /* Ensure we have enough space - if not, reset buffer (application should read faster)
     * Philosophy: Keep it simple - if the application can't keep up with data rate,
     * reset the buffer and start fresh rather than complex partial sample management */
    if (ring_buf_space_get(&g_sensor_manager.shared_data_buffer) < sample_size) {
        /* Application is not reading fast enough - reset buffer and start fresh */
        ring_buf_reset(&g_sensor_manager.shared_data_buffer);
        LOG_WRN("Shared buffer overflow - reset buffer for device ID %d location %d", 
                dev_info->device_id, dev_info->device_location);
    }

    /* Reserve space in shared ring buffer and get direct write pointers */
    uint8_t *write_ptr;
    uint32_t available_space;
    
    /* Get contiguous space for writing */
    available_space = ring_buf_put_claim(&g_sensor_manager.shared_data_buffer, &write_ptr, sample_size);
    if (available_space < sample_size) {
        /* This should not happen after making room, but handle gracefully */
        k_mutex_unlock(&g_sensor_manager.shared_buffer_mutex);
        LOG_ERR("Failed to get space in shared buffer for device ID %d location %d after cleanup", 
                dev_info->device_id, dev_info->device_location);
        return;
    }

    /* Zero-copy path: Write directly to shared ring buffer memory using compact format */
    uint8_t *current_ptr = write_ptr;
    
    /* Write device ID (1 byte) */
    *current_ptr = dev_info->device_id;
    current_ptr += 1;
    
    /* Write device location (1 byte) */
    *current_ptr = dev_info->device_location;
    current_ptr += 1;
    
    /* Write time delta (2 bytes, little-endian) */
    memcpy(current_ptr, &time_delta, sizeof(time_delta));
    current_ptr += sizeof(time_delta);

    /* Read enabled channels directly into shared ring buffer memory with variable sizes */
    uint8_t actual_channels = 0;
    for (int i = 0; i < SENSOR_MANAGER_MAX_CHANNELS_PER_DEVICE; i++) {
        if (!dev_info->channels[i].enabled) {
            continue;
        }

        enum sensor_channel channel = dev_info->channels[i].channel;
        size_t channel_data_size = get_sensor_channel_data_size(device, channel);

        /* sensor_channel_get writes directly to shared ring buffer memory! */
        struct sensor_value *value_ptr = (struct sensor_value *)current_ptr;
        ret = sensor_channel_get(device, channel, value_ptr);
        if (ret != 0) {
            LOG_WRN("Failed to get channel %d data: %d", channel, ret);
            /* Zero out failed channel to maintain structure */
            memset(value_ptr, 0, channel_data_size);
        }

        current_ptr += channel_data_size;
        actual_channels++;
    }

    /* Commit the data to shared ring buffer (always full sample_size) */
    ring_buf_put_finish(&g_sensor_manager.shared_data_buffer, sample_size);
    LOG_DBG("Stored %d channels, %zu bytes for device ID %d location %d in shared buffer", 
            actual_channels, sample_size, dev_info->device_id, dev_info->device_location);

    /* Check if buffer threshold has been reached (call callback while holding mutex) */
    check_buffer_threshold();

    k_mutex_unlock(&g_sensor_manager.shared_buffer_mutex);

    /* Note: No device-level callbacks - applications should monitor shared buffer directly */
}

int sensor_manager_init(void)
{
    if (g_sensor_manager.initialized) {
        return SENSOR_MANAGER_OK;
    }

    /* Initialize device structures */
    memset(g_sensor_manager.devices, 0, sizeof(g_sensor_manager.devices));
    g_sensor_manager.num_devices = 0;
    g_sensor_manager.acquisition_active = false;

    /* Initialize shared buffer mutex */
    k_mutex_init(&g_sensor_manager.shared_buffer_mutex);

    /* Initialize memory pool */
    memset(g_sensor_manager.memory_pool, 0, sizeof(g_sensor_manager.memory_pool));
    g_sensor_manager.allocated_memory = NULL;
    g_sensor_manager.allocated_size = 0;

    /* Initialize buffer threshold callback fields */
    g_sensor_manager.buffer_threshold_bytes = 0;
    g_sensor_manager.buffer_callback = NULL;
    g_sensor_manager.buffer_callback_user_data = NULL;

    g_sensor_manager.initialized = true;

    LOG_INF("Sensor manager initialized with %dKB static memory pool", 
            SENSOR_MANAGER_MEMORY_POOL_SIZE / 1024);
    return SENSOR_MANAGER_OK;
}

int sensor_manager_deinit(void)
{
    if (!g_sensor_manager.initialized) {
        return SENSOR_MANAGER_ERROR_NOT_INITIALIZED;
    }

    /* Stop and cleanup all devices */
    sensor_manager_stop_acquisition_all();
    
    memset(g_sensor_manager.devices, 0, sizeof(g_sensor_manager.devices));
    g_sensor_manager.num_devices = 0;
    g_sensor_manager.initialized = false;

    LOG_INF("Sensor manager deinitialized");
    return SENSOR_MANAGER_OK;
}

int sensor_manager_add_device(const struct device *device, uint8_t device_id, uint8_t device_location, bool samples_prefetched)
{
    if (!g_sensor_manager.initialized) {
        return SENSOR_MANAGER_ERROR_NOT_INITIALIZED;
    }

    if (!device) {
        return SENSOR_MANAGER_ERROR_INVALID_PARAM;
    }

    if (!device_is_ready(device)) {
        return SENSOR_MANAGER_ERROR_DEVICE_NOT_READY;
    }

    /* Check if device already exists */
    if (find_device_info(device)) {
        return SENSOR_MANAGER_ERROR_INVALID_PARAM;
    }

    /* Check if we have space for more devices */
    if (g_sensor_manager.num_devices >= SENSOR_MANAGER_MAX_DEVICES) {
        return SENSOR_MANAGER_ERROR_MAX_DEVICES;
    }

    /* Find an available slot */
    struct sensor_device_info *dev_info = NULL;
    for (int i = 0; i < SENSOR_MANAGER_MAX_DEVICES; i++) {
        if (!g_sensor_manager.devices[i].active) {
            dev_info = &g_sensor_manager.devices[i];
            break;
        }
    }

    if (!dev_info) {
        return SENSOR_MANAGER_ERROR_MAX_DEVICES;
    }

    /* Initialize device info */
    memset(dev_info, 0, sizeof(*dev_info));
    dev_info->device = device;
    dev_info->device_id = device_id;
    dev_info->device_location = device_location;
    dev_info->samples_prefetched = samples_prefetched;
    
    /* Initialize fields */
    dev_info->acquisition_active = false;
    dev_info->sample_block_size = 0; /* Will be computed when acquisition starts */
    dev_info->last_sample_timestamp_ms = 0; /* Initialize device-specific timestamp */

    /* Initialize channels as disabled */
    for (int i = 0; i < SENSOR_MANAGER_MAX_CHANNELS_PER_DEVICE; i++) {
        dev_info->channels[i].enabled = false;
    }
    dev_info->num_enabled_channels = 0;

    /* Mark device as active */
    dev_info->active = true;
    g_sensor_manager.num_devices++;

    LOG_INF("Added sensor device: ID %d, location %d", device_id, device_location);
    return SENSOR_MANAGER_OK;
}

int sensor_manager_remove_device(const struct device *device)
{
    if (!g_sensor_manager.initialized) {
        return SENSOR_MANAGER_ERROR_NOT_INITIALIZED;
    }

    if (!device) {
        return SENSOR_MANAGER_ERROR_INVALID_PARAM;
    }

    struct sensor_device_info *dev_info = find_device_info(device);
    if (!dev_info) {
        return SENSOR_MANAGER_ERROR_DEVICE_NOT_FOUND;
    }

    /* Device cannot be removed during active acquisition */
    if (g_sensor_manager.acquisition_active) {
        LOG_ERR("Cannot remove device during active acquisition. Stop acquisition first.");
        return SENSOR_MANAGER_ERROR_ACQUISITION_ACTIVE;
    }

    /* Clear device info */
    memset(dev_info, 0, sizeof(*dev_info));
    g_sensor_manager.num_devices--;

    LOG_INF("Removed sensor device");
    return SENSOR_MANAGER_OK;
}

int sensor_manager_enable_channel(const struct device *device, enum sensor_channel channel)
{
    if (!g_sensor_manager.initialized) {
        return SENSOR_MANAGER_ERROR_NOT_INITIALIZED;
    }

    if (!device) {
        return SENSOR_MANAGER_ERROR_INVALID_PARAM;
    }

    struct sensor_device_info *dev_info = find_device_info(device);
    if (!dev_info) {
        return SENSOR_MANAGER_ERROR_DEVICE_NOT_FOUND;
    }

    /* Check if acquisition is active - prevent channel changes during acquisition */
    if (g_sensor_manager.acquisition_active) {
        return SENSOR_MANAGER_ERROR_ACQUISITION_ACTIVE;
    }

    /* Check if channel is already enabled */
    for (int i = 0; i < SENSOR_MANAGER_MAX_CHANNELS_PER_DEVICE; i++) {
        if (dev_info->channels[i].enabled && dev_info->channels[i].channel == channel) {
            return SENSOR_MANAGER_OK; /* Already enabled */
        }
    }

    /* Find available slot for new channel */
    for (int i = 0; i < SENSOR_MANAGER_MAX_CHANNELS_PER_DEVICE; i++) {
        if (!dev_info->channels[i].enabled) {
            dev_info->channels[i].channel = channel;
            dev_info->channels[i].enabled = true;
            dev_info->num_enabled_channels++;
            break;
        }
    }

    if (dev_info->num_enabled_channels > SENSOR_MANAGER_MAX_CHANNELS_PER_DEVICE) {
        return SENSOR_MANAGER_ERROR_MAX_CHANNELS;
    }

    LOG_DBG("Enabled channel %d for device ID %d location %d", 
            channel, dev_info->device_id, dev_info->device_location);
    return SENSOR_MANAGER_OK;
}

int sensor_manager_disable_channel(const struct device *device, enum sensor_channel channel)
{
    if (!g_sensor_manager.initialized) {
        return SENSOR_MANAGER_ERROR_NOT_INITIALIZED;
    }

    if (!device) {
        return SENSOR_MANAGER_ERROR_INVALID_PARAM;
    }

    struct sensor_device_info *dev_info = find_device_info(device);
    if (!dev_info) {
        return SENSOR_MANAGER_ERROR_DEVICE_NOT_FOUND;
    }

    /* Check if acquisition is active - prevent channel changes during acquisition */
    if (g_sensor_manager.acquisition_active) {
        return SENSOR_MANAGER_ERROR_ACQUISITION_ACTIVE;
    }

    /* Find and disable the channel */
    for (int i = 0; i < SENSOR_MANAGER_MAX_CHANNELS_PER_DEVICE; i++) {
        if (dev_info->channels[i].enabled && dev_info->channels[i].channel == channel) {
            dev_info->channels[i].enabled = false;
            dev_info->num_enabled_channels--;
            break;
        }
    }

    LOG_DBG("Disabled channel %d for device ID %d location %d", 
            channel, dev_info->device_id, dev_info->device_location);
    return SENSOR_MANAGER_OK;
}

int sensor_manager_start_acquisition_all(void)
{
    if (!g_sensor_manager.initialized) {
        return SENSOR_MANAGER_ERROR_NOT_INITIALIZED;
    }

    /* Check if acquisition is already active */
    if (g_sensor_manager.acquisition_active) {
        return SENSOR_MANAGER_ERROR_ACQUISITION_ACTIVE;
    }

    /* Check if we have any devices */
    if (g_sensor_manager.num_devices == 0) {
        return SENSOR_MANAGER_ERROR_INVALID_PARAM;
    }

    /* Calculate total memory needed for shared buffer and prepare device sample sizes */
    size_t total_shared_buffer_size = 0;
    size_t max_sample_size = 0;
    
    for (int i = 0; i < g_sensor_manager.num_devices; i++) {
        struct sensor_device_info *dev_info = &g_sensor_manager.devices[i];
        if (!dev_info->active) {
            continue;
        }

        /* Check if device has enabled channels */
        if (dev_info->num_enabled_channels == 0) {
            LOG_ERR("Device ID %d location %d has no enabled channels", 
                    dev_info->device_id, dev_info->device_location);
            return SENSOR_MANAGER_ERROR_INVALID_PARAM;
        }

        /* Pre-compute fixed sample block size based on enabled channels and compact format */
        size_t sample_size = 1 + 1 + 2; /* device_id (1) + device_location (1) + time_delta (2) */
        for (int j = 0; j < SENSOR_MANAGER_MAX_CHANNELS_PER_DEVICE; j++) {
            if (!dev_info->channels[j].enabled) {
                continue;
            }
            size_t channel_data_size = get_sensor_channel_data_size(dev_info->device, dev_info->channels[j].channel);
            sample_size += channel_data_size;
        }
        dev_info->sample_block_size = sample_size;

        /* Use a default buffer size calculation for shared buffer sizing
         * Since we don't have requested_buffer_samples anymore, use a reasonable default */
        size_t device_buffer_contribution = SENSOR_MANAGER_DEFAULT_BUFFER_SIZE * dev_info->sample_block_size;
        total_shared_buffer_size += device_buffer_contribution;
        
        if (dev_info->sample_block_size > max_sample_size) {
            max_sample_size = dev_info->sample_block_size;
        }
    }

    /* Add some extra space to shared buffer for interleaving efficiency */
    total_shared_buffer_size += max_sample_size * 4; /* 4 extra samples worth of space */

    LOG_DBG("Total shared buffer size needed: %zu bytes (max sample: %zu)", total_shared_buffer_size, max_sample_size);

    /* Allocate memory from static pool for shared buffer */
    uint8_t *memory_base = sensor_manager_pool_alloc_all(total_shared_buffer_size);
    if (!memory_base) {
        LOG_ERR("Failed to allocate %zu bytes from memory pool for shared buffer", total_shared_buffer_size);
        return SENSOR_MANAGER_ERROR_MEMORY_POOL_FULL;
    }

    /* Initialize shared ring buffer */
    ring_buf_init(&g_sensor_manager.shared_data_buffer, total_shared_buffer_size, memory_base);
    
    /* Set up triggers and activate devices */
    for (int i = 0; i < g_sensor_manager.num_devices; i++) {
        struct sensor_device_info *dev_info = &g_sensor_manager.devices[i];
        if (!dev_info->active) {
            continue;
        }
        
        /* Set up trigger with internal handler */
        int ret = sensor_trigger_set(dev_info->device, &dev_info->trigger, internal_data_ready_handler);
        if (ret != 0) {
            LOG_ERR("Failed to set trigger for device ID %d location %d: %d", 
                    dev_info->device_id, dev_info->device_location, ret);
            /* Continue with other devices, don't fail completely */
        }

        /* Mark device acquisition as active */
        dev_info->acquisition_active = true;
        
        LOG_DBG("Device ID %d location %d ready for shared buffer: sample size %zu bytes", 
                dev_info->device_id, dev_info->device_location, dev_info->sample_block_size);
    }

    /* Mark global acquisition as active */
    g_sensor_manager.acquisition_active = true;

    LOG_INF("Started acquisition for all %d sensor devices with shared buffer (%zu bytes total)", 
            g_sensor_manager.num_devices, total_shared_buffer_size);
    return SENSOR_MANAGER_OK;
}

int sensor_manager_stop_acquisition_all(void)
{
    if (!g_sensor_manager.initialized) {
        return SENSOR_MANAGER_ERROR_NOT_INITIALIZED;
    }

    /* Check if acquisition is active */
    if (!g_sensor_manager.acquisition_active) {
        return SENSOR_MANAGER_OK; /* Already stopped */
    }

    /* Stop acquisition for all devices */
    for (int i = 0; i < g_sensor_manager.num_devices; i++) {
        struct sensor_device_info *dev_info = &g_sensor_manager.devices[i];
        if (!dev_info->active || !dev_info->acquisition_active) {
            continue;
        }

        /* Disable trigger */
        sensor_trigger_set(dev_info->device, &dev_info->trigger, NULL);

        /* Mark device acquisition as inactive */
        dev_info->acquisition_active = false;
        
        LOG_DBG("Stopped acquisition for device ID %d location %d", 
                dev_info->device_id, dev_info->device_location);
    }

    /* Free all memory back to the pool */
    sensor_manager_pool_free_all();

    /* Mark global acquisition as inactive */
    g_sensor_manager.acquisition_active = false;

    LOG_INF("Stopped acquisition for all sensor devices and freed shared buffer memory");
    return SENSOR_MANAGER_OK;
}

int sensor_manager_clear_shared_buffer(void)
{
    if (!g_sensor_manager.initialized) {
        return SENSOR_MANAGER_ERROR_NOT_INITIALIZED;
    }

    k_mutex_lock(&g_sensor_manager.shared_buffer_mutex, K_FOREVER);
    ring_buf_reset(&g_sensor_manager.shared_data_buffer);
    k_mutex_unlock(&g_sensor_manager.shared_buffer_mutex);

    LOG_DBG("Cleared shared buffer");
    return SENSOR_MANAGER_OK;
}

struct sensor_manager *sensor_manager_get_instance(void)
{
    return &g_sensor_manager;
}

/**
 * @brief Check and trigger buffer threshold callback if needed
 * 
 * Internal function called after adding data to shared buffer to check
 * if the threshold has been exceeded and trigger the callback.
 */
static void check_buffer_threshold(void)
{
    if (g_sensor_manager.buffer_callback && g_sensor_manager.buffer_threshold_bytes > 0) {
        size_t available_bytes = ring_buf_size_get(&g_sensor_manager.shared_data_buffer);
        
        if (available_bytes >= g_sensor_manager.buffer_threshold_bytes) {
            /* Trigger the application callback */
            g_sensor_manager.buffer_callback(available_bytes, g_sensor_manager.buffer_callback_user_data);
        }
    }
}

int sensor_manager_set_buffer_threshold_callback(size_t threshold_bytes,
                                                sensor_manager_buffer_callback_t callback,
                                                void *user_data)
{
    if (!g_sensor_manager.initialized) {
        return SENSOR_MANAGER_ERROR_NOT_INITIALIZED;
    }

    if (threshold_bytes == 0 && callback != NULL) {
        return SENSOR_MANAGER_ERROR_INVALID_PARAM;
    }

    k_mutex_lock(&g_sensor_manager.shared_buffer_mutex, K_FOREVER);
    
    g_sensor_manager.buffer_threshold_bytes = threshold_bytes;
    g_sensor_manager.buffer_callback = callback;
    g_sensor_manager.buffer_callback_user_data = user_data;
    
    k_mutex_unlock(&g_sensor_manager.shared_buffer_mutex);

    if (callback) {
        LOG_INF("Set buffer threshold callback: %zu bytes", threshold_bytes);
    } else {
        LOG_INF("Cleared buffer threshold callback");
    }

    return SENSOR_MANAGER_OK;
}

int sensor_manager_read_buffer_bytes(uint8_t *buffer, size_t max_bytes, size_t *bytes_read)
{
    if (!g_sensor_manager.initialized) {
        return SENSOR_MANAGER_ERROR_NOT_INITIALIZED;
    }

    if (!buffer || !bytes_read || max_bytes == 0) {
        return SENSOR_MANAGER_ERROR_INVALID_PARAM;
    }

    k_mutex_lock(&g_sensor_manager.shared_buffer_mutex, K_FOREVER);

    uint32_t available_data = ring_buf_size_get(&g_sensor_manager.shared_data_buffer);
    size_t to_read = (max_bytes < available_data) ? max_bytes : available_data;
    
    uint32_t actual_read = ring_buf_get(&g_sensor_manager.shared_data_buffer, buffer, to_read);
    *bytes_read = actual_read;

    k_mutex_unlock(&g_sensor_manager.shared_buffer_mutex);

    LOG_DBG("Read %zu bytes from shared buffer (%zu requested)", actual_read, max_bytes);
    return SENSOR_MANAGER_OK;
}

size_t sensor_manager_get_buffer_data_available(void)
{
    if (!g_sensor_manager.initialized) {
        return 0;
    }

    k_mutex_lock(&g_sensor_manager.shared_buffer_mutex, K_FOREVER);
    size_t available = ring_buf_size_get(&g_sensor_manager.shared_data_buffer);
    k_mutex_unlock(&g_sensor_manager.shared_buffer_mutex);

    return available;
}
