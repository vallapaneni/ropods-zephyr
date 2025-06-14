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
 * @brief Internal data ready callback handler with optimized zero-copy and fixed sample size
 */
static void internal_data_ready_handler(const struct device *device,
                                       const struct sensor_trigger *trigger)
{
    struct sensor_device_info *dev_info = find_device_info(device);
    if (!dev_info) {
        LOG_ERR("Device not found in data ready handler");
        return;
    }

    uint32_t timestamp_ms = (uint32_t)(k_uptime_get() / 1000); /* Convert μs to ms */
    int ret;

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

    /* Lock buffer for data storage */
    k_mutex_lock(&dev_info->buffer_mutex, K_FOREVER);

    /* Use pre-computed fixed sample block size */
    size_t sample_size = dev_info->sample_block_size;

    /* Ensure we have enough space by removing old samples if needed (always keep latest data) */
    while (ring_buf_space_get(&dev_info->data_buffer) < sample_size && 
           !ring_buf_is_empty(&dev_info->data_buffer)) {
        
        /* Efficiently remove one complete sample by advancing read pointer */
        uint8_t *read_ptr;
        uint32_t available_bytes = ring_buf_get_claim(&dev_info->data_buffer, &read_ptr, sample_size);
        
        if (available_bytes >= sample_size) {
            /* Successfully claim and discard one complete sample */
            ring_buf_get_finish(&dev_info->data_buffer, sample_size);
            dev_info->buffer_overflow_count++;
        } else {
            /* Not enough contiguous data or corrupted buffer, reset */
            ring_buf_reset(&dev_info->data_buffer);
            break;
        }
    }

    /* Reserve space in ring buffer and get direct write pointers */
    uint8_t *write_ptr;
    uint32_t available_space;
    
    /* Get contiguous space for writing - guaranteed to succeed since buffer size is multiple of sample size */
    available_space = ring_buf_put_claim(&dev_info->data_buffer, &write_ptr, sample_size);
    if (available_space < sample_size) {
        /* This should never happen with optimized buffer size, but handle gracefully */
        LOG_ERR("Unexpected: insufficient contiguous space in optimized buffer");
        k_mutex_unlock(&dev_info->buffer_mutex);
        return;
    }

    /* Zero-copy path: Write directly to ring buffer memory */
    uint8_t *current_ptr = write_ptr;
    
    /* Write timestamp directly to ring buffer */
    memcpy(current_ptr, &timestamp_ms, sizeof(timestamp_ms));
    current_ptr += sizeof(timestamp_ms);

    /* Read enabled channels directly into ring buffer memory with variable sizes */
    uint8_t actual_channels = 0;
    for (int i = 0; i < SENSOR_MANAGER_MAX_CHANNELS_PER_DEVICE; i++) {
        if (!dev_info->channels[i].enabled) {
            continue;
        }

        enum sensor_channel channel = dev_info->channels[i].channel;
        size_t channel_data_size = get_sensor_channel_data_size(device, channel);

        /* sensor_channel_get writes directly to ring buffer memory! */
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

    /* Commit the data to ring buffer (always full sample_size) */
    ring_buf_put_finish(&dev_info->data_buffer, sample_size);
    LOG_DBG("Zero-copy stored %d channels, %zu bytes for device %s", 
            actual_channels, sample_size, dev_info->name);

    k_mutex_unlock(&dev_info->buffer_mutex);

    /* Call user callback if set */
    if (dev_info->callback) {
        dev_info->callback(device, trigger);
    }
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

    /* Initialize memory pool */
    memset(g_sensor_manager.memory_pool, 0, sizeof(g_sensor_manager.memory_pool));
    g_sensor_manager.allocated_memory = NULL;
    g_sensor_manager.allocated_size = 0;

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

int sensor_manager_add_device(const struct device *device, const char *name, bool samples_prefetched, size_t buffer_size)
{
    if (!g_sensor_manager.initialized) {
        return SENSOR_MANAGER_ERROR_NOT_INITIALIZED;
    }

    if (!device || !name) {
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
    strncpy(dev_info->name, name, SENSOR_MANAGER_MAX_NAME_LEN - 1);
    dev_info->name[SENSOR_MANAGER_MAX_NAME_LEN - 1] = '\0';
    dev_info->samples_prefetched = samples_prefetched;
    
    /* Initialize new optimization fields */
    dev_info->acquisition_active = false;
    dev_info->sample_block_size = 0; /* Will be computed when acquisition starts */
    
    /* Store requested buffer size (number of samples) - buffer will be allocated in start_acquisition */
    if (buffer_size == 0) {
        buffer_size = SENSOR_MANAGER_DEFAULT_BUFFER_SIZE;
    }
    dev_info->requested_buffer_samples = buffer_size;
    dev_info->buffer_size = 0; /* Will be computed when acquisition starts */
    dev_info->buffer_memory = NULL; /* Will be allocated when acquisition starts */

    /* Initialize buffer mutex */
    k_mutex_init(&dev_info->buffer_mutex);

    /* Initialize channels as disabled */
    for (int i = 0; i < SENSOR_MANAGER_MAX_CHANNELS_PER_DEVICE; i++) {
        dev_info->channels[i].enabled = false;
    }
    dev_info->num_enabled_channels = 0;

    /* Mark device as active */
    dev_info->active = true;
    g_sensor_manager.num_devices++;

    LOG_INF("Added sensor device: %s", name);
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

    LOG_DBG("Enabled channel %d for device %s", channel, dev_info->name);
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

    LOG_DBG("Disabled channel %d for device %s", channel, dev_info->name);
    return SENSOR_MANAGER_OK;
}

int sensor_manager_set_trigger_callback(const struct device *device, 
                                       enum sensor_trigger_type trigger_type,
                                       sensor_trigger_handler_t callback)
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

    /* Set up trigger configuration */
    dev_info->trigger.type = trigger_type;
    dev_info->trigger.chan = SENSOR_CHAN_ALL;
    dev_info->callback = callback;

    LOG_DBG("Set trigger callback for device %s", dev_info->name);
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

    /* Calculate total memory needed for all devices */
    size_t total_memory_needed = 0;
    for (int i = 0; i < g_sensor_manager.num_devices; i++) {
        struct sensor_device_info *dev_info = &g_sensor_manager.devices[i];
        if (!dev_info->active) {
            continue;
        }

        /* Check if device has enabled channels */
        if (dev_info->num_enabled_channels == 0) {
            LOG_ERR("Device %s has no enabled channels", dev_info->name);
            return SENSOR_MANAGER_ERROR_INVALID_PARAM;
        }

        /* Pre-compute fixed sample block size based on enabled channels */
        size_t sample_size = sizeof(uint32_t); /* timestamp_ms */
        for (int j = 0; j < SENSOR_MANAGER_MAX_CHANNELS_PER_DEVICE; j++) {
            if (!dev_info->channels[j].enabled) {
                continue;
            }
            size_t channel_data_size = get_sensor_channel_data_size(dev_info->device, dev_info->channels[j].channel);
            sample_size += channel_data_size;
        }
        dev_info->sample_block_size = sample_size;

        /* Calculate buffer size for this device */
        dev_info->buffer_size = dev_info->requested_buffer_samples * dev_info->sample_block_size;
        total_memory_needed += dev_info->buffer_size;
    }

    LOG_DBG("Total memory needed for all devices: %zu bytes", total_memory_needed);

    /* Allocate memory from static pool for all devices */
    uint8_t *memory_base = sensor_manager_pool_alloc_all(total_memory_needed);
    if (!memory_base) {
        LOG_ERR("Failed to allocate %zu bytes from memory pool for all devices", total_memory_needed);
        return SENSOR_MANAGER_ERROR_MEMORY_POOL_FULL;
    }

    /* Assign memory regions to each device and initialize ring buffers */
    uint8_t *current_memory = memory_base;
    for (int i = 0; i < g_sensor_manager.num_devices; i++) {
        struct sensor_device_info *dev_info = &g_sensor_manager.devices[i];
        if (!dev_info->active) {
            continue;
        }

        /* Assign memory region to this device */
        dev_info->buffer_memory = current_memory;
        current_memory += dev_info->buffer_size;

        /* Initialize ring buffer */
        ring_buf_init(&dev_info->data_buffer, dev_info->buffer_size, dev_info->buffer_memory);
        
        /* Set up trigger with internal handler */
        int ret = sensor_trigger_set(dev_info->device, &dev_info->trigger, internal_data_ready_handler);
        if (ret != 0) {
            LOG_ERR("Failed to set trigger for device %s: %d", dev_info->name, ret);
            /* Continue with other devices, don't fail completely */
        }

        /* Mark device acquisition as active */
        dev_info->acquisition_active = true;
        
        LOG_DBG("Initialized buffer for device %s: %zu bytes (%zu samples)", 
                dev_info->name, dev_info->buffer_size, dev_info->requested_buffer_samples);
    }

    /* Mark global acquisition as active */
    g_sensor_manager.acquisition_active = true;

    LOG_INF("Started acquisition for all %d sensor devices (%zu bytes total)", 
            g_sensor_manager.num_devices, total_memory_needed);
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
        
        /* Clear buffer memory pointer (will be freed globally) */
        dev_info->buffer_memory = NULL;
        dev_info->buffer_size = 0;
        
        LOG_DBG("Stopped acquisition for device %s", dev_info->name);
    }

    /* Free all memory back to the pool */
    sensor_manager_pool_free_all();

    /* Mark global acquisition as inactive */
    g_sensor_manager.acquisition_active = false;

    LOG_INF("Stopped acquisition for all sensor devices and freed memory pool");
    return SENSOR_MANAGER_OK;
}

int sensor_manager_get_latest_data(const struct device *device, 
                                  enum sensor_channel channel,
                                  uint32_t *timestamp_ms,
                                  struct sensor_value *value)
{
    if (!g_sensor_manager.initialized) {
        return SENSOR_MANAGER_ERROR_NOT_INITIALIZED;
    }

    if (!device || !timestamp_ms || !value) {
        return SENSOR_MANAGER_ERROR_INVALID_PARAM;
    }

    struct sensor_device_info *dev_info = find_device_info(device);
    if (!dev_info) {
        return SENSOR_MANAGER_ERROR_DEVICE_NOT_FOUND;
    }

    k_mutex_lock(&dev_info->buffer_mutex, K_FOREVER);

    /* Check if we have any data and acquisition is active */
    if (!dev_info->acquisition_active || dev_info->sample_block_size == 0) {
        k_mutex_unlock(&dev_info->buffer_mutex);
        return SENSOR_MANAGER_ERROR_DEVICE_NOT_FOUND;
    }

    /* Search ring buffer for latest data of specified channel */
    bool found = false;
    uint32_t latest_timestamp = 0;
    struct sensor_value latest_value = {0};

    /* Work backwards through ring buffer to find latest sample containing our channel */
    uint32_t available_data = ring_buf_size_get(&dev_info->data_buffer);
    
    if (available_data < dev_info->sample_block_size) {
        k_mutex_unlock(&dev_info->buffer_mutex);
        return SENSOR_MANAGER_ERROR_DEVICE_NOT_FOUND; /* No complete samples */
    }

    /* Calculate number of complete samples */
    uint32_t num_samples = available_data / dev_info->sample_block_size;
    
    /* Use temporary buffer to read all samples and find the latest one with our channel */
    uint8_t temp_buffer[available_data];
    uint32_t peeked_bytes = ring_buf_peek(&dev_info->data_buffer, temp_buffer, available_data);
    
    if (peeked_bytes != available_data) {
        k_mutex_unlock(&dev_info->buffer_mutex);
        return SENSOR_MANAGER_ERROR_DEVICE_NOT_FOUND; /* Buffer error */
    }
    
    /* Parse samples starting from the most recent */
    for (int sample_idx = num_samples - 1; sample_idx >= 0; sample_idx--) {
        uint8_t *sample_ptr = temp_buffer + (sample_idx * dev_info->sample_block_size);
        
        /* Parse variable-length sample format */
        uint8_t *ptr = sample_ptr;
        uint32_t block_timestamp;
        memcpy(&block_timestamp, ptr, sizeof(uint32_t));
        ptr += sizeof(uint32_t);
        
        /* Find the requested channel by its position in enabled channels array */
        size_t channel_offset = 0;
        for (int i = 0; i < dev_info->num_enabled_channels; i++) {
            if (dev_info->channels[i].enabled && dev_info->channels[i].channel == channel) {
                /* Found the channel, get its value from the current offset */
                struct sensor_value val;
                memcpy(&val, ptr + channel_offset, sizeof(struct sensor_value));
                
                /* This is the latest sample with our channel */
                latest_timestamp = block_timestamp;
                latest_value = val;
                found = true;
                break;
            }
            
            /* Move to next channel position */
            if (dev_info->channels[i].enabled) {
                channel_offset += get_sensor_channel_data_size(device, dev_info->channels[i].channel);
            }
        }
        
        if (found) {
            break; /* Found latest data */
        }
    }

    k_mutex_unlock(&dev_info->buffer_mutex);

    if (!found) {
        return SENSOR_MANAGER_ERROR_DEVICE_NOT_FOUND; /* Channel not found in any sample */
    }

    *timestamp_ms = latest_timestamp;
    *value = latest_value;
    return SENSOR_MANAGER_OK;
}

int sensor_manager_read_data_buffer(const struct device *device,
                                   struct sensor_sample_block *data,
                                   size_t max_blocks,
                                   size_t *blocks_read)
{
    if (!g_sensor_manager.initialized) {
        return SENSOR_MANAGER_ERROR_NOT_INITIALIZED;
    }

    if (!device || !data || !blocks_read) {
        return SENSOR_MANAGER_ERROR_INVALID_PARAM;
    }

    struct sensor_device_info *dev_info = find_device_info(device);
    if (!dev_info) {
        return SENSOR_MANAGER_ERROR_DEVICE_NOT_FOUND;
    }

    k_mutex_lock(&dev_info->buffer_mutex, K_FOREVER);

    /* Check if we have acquisition active and fixed sample size */
    if (!dev_info->acquisition_active || dev_info->sample_block_size == 0) {
        *blocks_read = 0;
        k_mutex_unlock(&dev_info->buffer_mutex);
        return SENSOR_MANAGER_OK;
    }

    uint32_t available_data = ring_buf_size_get(&dev_info->data_buffer);
    
    if (available_data < dev_info->sample_block_size) {
        *blocks_read = 0;
        k_mutex_unlock(&dev_info->buffer_mutex);
        return SENSOR_MANAGER_OK; /* No complete samples */
    }

    /* Calculate number of complete samples available */
    uint32_t available_samples = available_data / dev_info->sample_block_size;
    size_t samples_to_read = MIN(available_samples, max_blocks);

    /* Read samples directly from ring buffer using variable-length parsing */
    for (size_t i = 0; i < samples_to_read; i++) {
        uint8_t sample_buffer[dev_info->sample_block_size];
        
        /* Get one complete sample */
        if (ring_buf_get(&dev_info->data_buffer, sample_buffer, dev_info->sample_block_size) 
            != dev_info->sample_block_size) {
            break; /* Error reading sample */
        }
        
        /* Parse variable-length sample format */
        uint8_t *ptr = sample_buffer;
        
        /* Read timestamp */
        memcpy(&data[i].timestamp_ms, ptr, sizeof(uint32_t));
        ptr += sizeof(uint32_t);
        
        /* Copy variable-length sensor data directly from buffer */
        size_t data_size = dev_info->sample_block_size - sizeof(uint32_t);
        memcpy(data[i].data, ptr, data_size);
    }

    *blocks_read = samples_to_read;
    k_mutex_unlock(&dev_info->buffer_mutex);

    return SENSOR_MANAGER_OK;
}

int sensor_manager_clear_buffer(const struct device *device)
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

    k_mutex_lock(&dev_info->buffer_mutex, K_FOREVER);
    ring_buf_reset(&dev_info->data_buffer);
    k_mutex_unlock(&dev_info->buffer_mutex);

    LOG_DBG("Cleared buffer for device %s", dev_info->name);
    return SENSOR_MANAGER_OK;
}

int sensor_manager_get_stats(const struct device *device,
                            uint32_t *data_ready_count,
                            uint32_t *buffer_overflow_count,
                            uint8_t *buffer_usage)
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

    k_mutex_lock(&dev_info->buffer_mutex, K_FOREVER);

    if (data_ready_count) {
        *data_ready_count = dev_info->data_ready_count;
    }

    if (buffer_overflow_count) {
        *buffer_overflow_count = dev_info->buffer_overflow_count;
    }

    if (buffer_usage) {
        uint32_t used = ring_buf_size_get(&dev_info->data_buffer);
        uint32_t total = dev_info->buffer_size;
        *buffer_usage = (uint8_t)((used * 100) / total);
    }

    k_mutex_unlock(&dev_info->buffer_mutex);

    return SENSOR_MANAGER_OK;
}

int sensor_manager_get_enabled_channels(const struct device *device,
                                       enum sensor_channel *channels,
                                       size_t max_channels,
                                       size_t *num_channels)
{
    if (!g_sensor_manager.initialized) {
        return SENSOR_MANAGER_ERROR_NOT_INITIALIZED;
    }

    if (!device || !channels || !num_channels) {
        return SENSOR_MANAGER_ERROR_INVALID_PARAM;
    }

    struct sensor_device_info *dev_info = find_device_info(device);
    if (!dev_info) {
        return SENSOR_MANAGER_ERROR_DEVICE_NOT_FOUND;
    }

    size_t count = 0;
    for (int i = 0; i < SENSOR_MANAGER_MAX_CHANNELS_PER_DEVICE && count < max_channels; i++) {
        if (dev_info->channels[i].enabled) {
            channels[count] = dev_info->channels[i].channel;
            count++;
        }
    }

    *num_channels = count;

    return SENSOR_MANAGER_OK;
}

bool sensor_manager_is_device_managed(const struct device *device)
{
    if (!g_sensor_manager.initialized || !device) {
        return false;
    }

    return (find_device_info(device) != NULL);
}

struct sensor_manager *sensor_manager_get_instance(void)
{
    return &g_sensor_manager;
}

size_t sensor_manager_get_channel_data_size(const struct device *device, enum sensor_channel channel)
{
    return get_sensor_channel_data_size(device, channel);
}

size_t sensor_manager_get_sample_block_size(const struct device *device)
{
    if (!g_sensor_manager.initialized || !device) {
        return 0;
    }

    struct sensor_device_info *dev_info = find_device_info(device);
    if (!dev_info) {
        return 0;
    }

    /* Return the pre-computed sample block size (only valid during active acquisition) */
    return dev_info->sample_block_size;
}
