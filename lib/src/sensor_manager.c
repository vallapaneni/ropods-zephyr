/**
 * @file sensor_manager.c
 * @brief Implementation of the Sensor Manager
 */

#include "sensor_manager.h"
#include <zephyr/logging/log.h>
#include <zephyr/sys/printk.h>
#include <string.h>
#include <stdlib.h>

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
 * @brief Internal data ready callback handler
 */
static void internal_data_ready_handler(const struct device *device,
                                       const struct sensor_trigger *trigger)
{
    struct sensor_device_info *dev_info = find_device_info(device);
    if (!dev_info) {
        LOG_ERR("Device not found in data ready handler");
        return;
    }

    int64_t timestamp = k_uptime_get();
    int ret;

    /* Increment data ready counter */
    dev_info->data_ready_count++;

    /* Fetch sensor sample */
    ret = sensor_sample_fetch(device);
    if (ret != 0) {
        LOG_ERR("Failed to fetch sensor sample: %d", ret);
        return;
    }

    /* Lock buffer for data storage */
    k_mutex_lock(&dev_info->buffer_mutex, K_FOREVER);

    /* Read all enabled channels and store in buffer */
    for (int i = 0; i < SENSOR_MANAGER_MAX_CHANNELS_PER_DEVICE; i++) {
        if (!dev_info->channels[i].enabled) {
            continue;
        }

        struct sensor_data_entry entry;
        entry.timestamp_us = timestamp;
        entry.channel = dev_info->channels[i].channel;

        ret = sensor_channel_get(device, entry.channel, &entry.value);
        if (ret != 0) {
            LOG_WRN("Failed to get channel %d data: %d", entry.channel, ret);
            continue;
        }

        /* Try to put data in ring buffer */
        size_t written = ring_buf_put(&dev_info->data_buffer, 
                                     (uint8_t *)&entry, 
                                     sizeof(entry));
        
        if (written != sizeof(entry)) {
            dev_info->buffer_overflow_count++;
            LOG_WRN("Buffer overflow for device %s", dev_info->name);
        }
    }

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

    /* Initialize manager mutex */
    k_mutex_init(&g_sensor_manager.manager_mutex);

    /* Initialize device structures */
    memset(g_sensor_manager.devices, 0, sizeof(g_sensor_manager.devices));
    g_sensor_manager.num_devices = 0;
    g_sensor_manager.initialized = true;

    LOG_INF("Sensor manager initialized");
    return SENSOR_MANAGER_OK;
}

int sensor_manager_deinit(void)
{
    if (!g_sensor_manager.initialized) {
        return SENSOR_MANAGER_ERROR_NOT_INITIALIZED;
    }

    k_mutex_lock(&g_sensor_manager.manager_mutex, K_FOREVER);

    /* Stop and cleanup all devices */
    for (int i = 0; i < g_sensor_manager.num_devices; i++) {
        struct sensor_device_info *dev_info = &g_sensor_manager.devices[i];
        if (dev_info->active) {
            sensor_manager_stop_acquisition(dev_info->device);
            if (dev_info->buffer_memory) {
                k_free(dev_info->buffer_memory);
            }
        }
    }

    memset(g_sensor_manager.devices, 0, sizeof(g_sensor_manager.devices));
    g_sensor_manager.num_devices = 0;
    g_sensor_manager.initialized = false;

    k_mutex_unlock(&g_sensor_manager.manager_mutex);

    LOG_INF("Sensor manager deinitialized");
    return SENSOR_MANAGER_OK;
}

int sensor_manager_add_device(const struct device *device, const char *name, size_t buffer_size)
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

    k_mutex_lock(&g_sensor_manager.manager_mutex, K_FOREVER);

    /* Check if device already exists */
    if (find_device_info(device)) {
        k_mutex_unlock(&g_sensor_manager.manager_mutex);
        return SENSOR_MANAGER_ERROR_INVALID_PARAM;
    }

    /* Check if we have space for more devices */
    if (g_sensor_manager.num_devices >= SENSOR_MANAGER_MAX_DEVICES) {
        k_mutex_unlock(&g_sensor_manager.manager_mutex);
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
        k_mutex_unlock(&g_sensor_manager.manager_mutex);
        return SENSOR_MANAGER_ERROR_MAX_DEVICES;
    }

    /* Initialize device info */
    memset(dev_info, 0, sizeof(*dev_info));
    dev_info->device = device;
    strncpy(dev_info->name, name, SENSOR_MANAGER_MAX_NAME_LEN - 1);
    dev_info->name[SENSOR_MANAGER_MAX_NAME_LEN - 1] = '\0';
    
    /* Set buffer size */
    if (buffer_size == 0) {
        buffer_size = SENSOR_MANAGER_DEFAULT_BUFFER_SIZE;
    }
    dev_info->buffer_size = buffer_size * sizeof(struct sensor_data_entry);

    /* Allocate buffer memory */
    dev_info->buffer_memory = k_malloc(dev_info->buffer_size);
    if (!dev_info->buffer_memory) {
        k_mutex_unlock(&g_sensor_manager.manager_mutex);
        return SENSOR_MANAGER_ERROR_MEMORY_ALLOC;
    }

    /* Initialize ring buffer */
    ring_buf_init(&dev_info->data_buffer, dev_info->buffer_size, dev_info->buffer_memory);

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

    k_mutex_unlock(&g_sensor_manager.manager_mutex);

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

    k_mutex_lock(&g_sensor_manager.manager_mutex, K_FOREVER);

    struct sensor_device_info *dev_info = find_device_info(device);
    if (!dev_info) {
        k_mutex_unlock(&g_sensor_manager.manager_mutex);
        return SENSOR_MANAGER_ERROR_DEVICE_NOT_FOUND;
    }

    /* Stop acquisition if active */
    sensor_manager_stop_acquisition(device);

    /* Free buffer memory */
    if (dev_info->buffer_memory) {
        k_free(dev_info->buffer_memory);
    }

    /* Clear device info */
    memset(dev_info, 0, sizeof(*dev_info));
    g_sensor_manager.num_devices--;

    k_mutex_unlock(&g_sensor_manager.manager_mutex);

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

    k_mutex_lock(&g_sensor_manager.manager_mutex, K_FOREVER);

    struct sensor_device_info *dev_info = find_device_info(device);
    if (!dev_info) {
        k_mutex_unlock(&g_sensor_manager.manager_mutex);
        return SENSOR_MANAGER_ERROR_DEVICE_NOT_FOUND;
    }

    /* Check if channel is already enabled */
    for (int i = 0; i < SENSOR_MANAGER_MAX_CHANNELS_PER_DEVICE; i++) {
        if (dev_info->channels[i].enabled && dev_info->channels[i].channel == channel) {
            k_mutex_unlock(&g_sensor_manager.manager_mutex);
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
        k_mutex_unlock(&g_sensor_manager.manager_mutex);
        return SENSOR_MANAGER_ERROR_MAX_CHANNELS;
    }

    k_mutex_unlock(&g_sensor_manager.manager_mutex);

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

    k_mutex_lock(&g_sensor_manager.manager_mutex, K_FOREVER);

    struct sensor_device_info *dev_info = find_device_info(device);
    if (!dev_info) {
        k_mutex_unlock(&g_sensor_manager.manager_mutex);
        return SENSOR_MANAGER_ERROR_DEVICE_NOT_FOUND;
    }

    /* Find and disable the channel */
    for (int i = 0; i < SENSOR_MANAGER_MAX_CHANNELS_PER_DEVICE; i++) {
        if (dev_info->channels[i].enabled && dev_info->channels[i].channel == channel) {
            dev_info->channels[i].enabled = false;
            dev_info->num_enabled_channels--;
            break;
        }
    }

    k_mutex_unlock(&g_sensor_manager.manager_mutex);

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

    k_mutex_lock(&g_sensor_manager.manager_mutex, K_FOREVER);

    struct sensor_device_info *dev_info = find_device_info(device);
    if (!dev_info) {
        k_mutex_unlock(&g_sensor_manager.manager_mutex);
        return SENSOR_MANAGER_ERROR_DEVICE_NOT_FOUND;
    }

    /* Set up trigger configuration */
    dev_info->trigger.type = trigger_type;
    dev_info->trigger.chan = SENSOR_CHAN_ALL;
    dev_info->callback = callback;

    k_mutex_unlock(&g_sensor_manager.manager_mutex);

    LOG_DBG("Set trigger callback for device %s", dev_info->name);
    return SENSOR_MANAGER_OK;
}

int sensor_manager_start_acquisition(const struct device *device)
{
    if (!g_sensor_manager.initialized) {
        return SENSOR_MANAGER_ERROR_NOT_INITIALIZED;
    }

    if (!device) {
        return SENSOR_MANAGER_ERROR_INVALID_PARAM;
    }

    k_mutex_lock(&g_sensor_manager.manager_mutex, K_FOREVER);

    struct sensor_device_info *dev_info = find_device_info(device);
    if (!dev_info) {
        k_mutex_unlock(&g_sensor_manager.manager_mutex);
        return SENSOR_MANAGER_ERROR_DEVICE_NOT_FOUND;
    }

    /* Check if we have enabled channels */
    if (dev_info->num_enabled_channels == 0) {
        k_mutex_unlock(&g_sensor_manager.manager_mutex);
        return SENSOR_MANAGER_ERROR_INVALID_PARAM;
    }

    /* Set up trigger with internal handler */
    int ret = sensor_trigger_set(device, &dev_info->trigger, internal_data_ready_handler);
    if (ret != 0) {
        k_mutex_unlock(&g_sensor_manager.manager_mutex);
        LOG_ERR("Failed to set trigger for device %s: %d", dev_info->name, ret);
        return SENSOR_MANAGER_ERROR_TRIGGER_SETUP;
    }

    k_mutex_unlock(&g_sensor_manager.manager_mutex);

    LOG_INF("Started acquisition for device %s", dev_info->name);
    return SENSOR_MANAGER_OK;
}

int sensor_manager_stop_acquisition(const struct device *device)
{
    if (!g_sensor_manager.initialized) {
        return SENSOR_MANAGER_ERROR_NOT_INITIALIZED;
    }

    if (!device) {
        return SENSOR_MANAGER_ERROR_INVALID_PARAM;
    }

    k_mutex_lock(&g_sensor_manager.manager_mutex, K_FOREVER);

    struct sensor_device_info *dev_info = find_device_info(device);
    if (!dev_info) {
        k_mutex_unlock(&g_sensor_manager.manager_mutex);
        return SENSOR_MANAGER_ERROR_DEVICE_NOT_FOUND;
    }

    /* Disable trigger */
    sensor_trigger_set(device, &dev_info->trigger, NULL);

    k_mutex_unlock(&g_sensor_manager.manager_mutex);

    LOG_INF("Stopped acquisition for device %s", dev_info->name);
    return SENSOR_MANAGER_OK;
}

int sensor_manager_get_latest_data(const struct device *device, 
                                  enum sensor_channel channel,
                                  struct sensor_data_entry *data)
{
    if (!g_sensor_manager.initialized) {
        return SENSOR_MANAGER_ERROR_NOT_INITIALIZED;
    }

    if (!device || !data) {
        return SENSOR_MANAGER_ERROR_INVALID_PARAM;
    }

    struct sensor_device_info *dev_info = find_device_info(device);
    if (!dev_info) {
        return SENSOR_MANAGER_ERROR_DEVICE_NOT_FOUND;
    }

    k_mutex_lock(&dev_info->buffer_mutex, K_FOREVER);

    /* Search ring buffer for latest data of specified channel */
    uint8_t buffer_data[sizeof(struct sensor_data_entry)];
    size_t data_size = sizeof(struct sensor_data_entry);
    bool found = false;
    struct sensor_data_entry latest_entry = {0};

    /* Read from buffer (this is a simplified approach - in practice you might want to optimize) */
    while (ring_buf_get(&dev_info->data_buffer, buffer_data, data_size) == data_size) {
        struct sensor_data_entry *entry = (struct sensor_data_entry *)buffer_data;
        if (entry->channel == channel) {
            if (!found || entry->timestamp_us > latest_entry.timestamp_us) {
                latest_entry = *entry;
                found = true;
            }
        }
    }

    k_mutex_unlock(&dev_info->buffer_mutex);

    if (!found) {
        return SENSOR_MANAGER_ERROR_DEVICE_NOT_FOUND; /* No data found */
    }

    *data = latest_entry;
    return SENSOR_MANAGER_OK;
}

int sensor_manager_read_data_buffer(const struct device *device,
                                   struct sensor_data_entry *data,
                                   size_t max_entries,
                                   size_t *entries_read)
{
    if (!g_sensor_manager.initialized) {
        return SENSOR_MANAGER_ERROR_NOT_INITIALIZED;
    }

    if (!device || !data || !entries_read) {
        return SENSOR_MANAGER_ERROR_INVALID_PARAM;
    }

    struct sensor_device_info *dev_info = find_device_info(device);
    if (!dev_info) {
        return SENSOR_MANAGER_ERROR_DEVICE_NOT_FOUND;
    }

    k_mutex_lock(&dev_info->buffer_mutex, K_FOREVER);

    size_t count = 0;
    size_t data_size = sizeof(struct sensor_data_entry);

    while (count < max_entries) {
        size_t read = ring_buf_get(&dev_info->data_buffer, 
                                  (uint8_t *)&data[count], 
                                  data_size);
        if (read != data_size) {
            break; /* No more data */
        }
        count++;
    }

    *entries_read = count;
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

    k_mutex_lock(&g_sensor_manager.manager_mutex, K_FOREVER);

    size_t count = 0;
    for (int i = 0; i < SENSOR_MANAGER_MAX_CHANNELS_PER_DEVICE && count < max_channels; i++) {
        if (dev_info->channels[i].enabled) {
            channels[count] = dev_info->channels[i].channel;
            count++;
        }
    }

    *num_channels = count;
    k_mutex_unlock(&g_sensor_manager.manager_mutex);

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
