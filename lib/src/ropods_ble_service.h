/**
 * @file ropods_ble_service.h
 * @brief Generic ROPODS BLE Service for streaming sensor data and device control
 * 
 * This service provides a generic BLE interface for ROPODS devices with:
 * - Data streaming via notification characteristic
 * - Generic control commands for device/channel management  
 * - Device information and capability discovery
 * - Extensible attribute system for any sensor type
 * 
 * The service is completely sensor-agnostic and can work with any data format.
 */

#ifndef ROPODS_BLE_SERVICE_H
#define ROPODS_BLE_SERVICE_H

#include <zephyr/kernel.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/hci.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/device.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ROPODS BLE Service UUID - Base: 6E400001-B5A3-F393-E0A9-E50E24DCCA9E */
#define BT_UUID_ROPODS_SERVICE_VAL \
	BT_UUID_128_ENCODE(0x6E400001, 0xB5A3, 0xF393, 0xE0A9, 0xE50E24DCCA9E)

/* ROPODS BLE Characteristic UUIDs */
#define BT_UUID_ROPODS_DATA_STREAM_VAL \
	BT_UUID_128_ENCODE(0x6E400002, 0xB5A3, 0xF393, 0xE0A9, 0xE50E24DCCA9E)

#define BT_UUID_ROPODS_CONTROL_VAL \
	BT_UUID_128_ENCODE(0x6E400003, 0xB5A3, 0xF393, 0xE0A9, 0xE50E24DCCA9E)

#define BT_UUID_ROPODS_DEVICE_INFO_VAL \
	BT_UUID_128_ENCODE(0x6E400004, 0xB5A3, 0xF393, 0xE0A9, 0xE50E24DCCA9E)

#define BT_UUID_ROPODS_SERVICE           BT_UUID_DECLARE_128(BT_UUID_ROPODS_SERVICE_VAL)
#define BT_UUID_ROPODS_DATA_STREAM       BT_UUID_DECLARE_128(BT_UUID_ROPODS_DATA_STREAM_VAL)
#define BT_UUID_ROPODS_CONTROL           BT_UUID_DECLARE_128(BT_UUID_ROPODS_CONTROL_VAL)
#define BT_UUID_ROPODS_DEVICE_INFO       BT_UUID_DECLARE_128(BT_UUID_ROPODS_DEVICE_INFO_VAL)

/* Maximum data packet size (MTU - headers) */
#define ROPODS_BLE_MAX_DATA_SIZE        (247 - 3)
#define ROPODS_BLE_MAX_DEVICES 4

/* Generic data packet structure for streaming any type of data */
struct ropods_data_packet {
	uint32_t packet_id;          /**< Packet sequence number */
	uint8_t sample_count;        /**< Number of samples in this packet */
	uint8_t device_id;           /**< Source device ID */
	uint8_t device_location;     /**< Device location */
	uint8_t data_format;         /**< Data format identifier */
	uint64_t timestamp;          /**< Timestamp (microseconds) */
	uint8_t data[];              /**< Variable-length data payload */
} __packed;

/* Device information structure for BLE characteristic */
struct ropods_device_info_entry {
    uint8_t sensor_id;       /* Sensor identifier */
    uint8_t sensor_location; /* Sensor location */
} __packed;

/* Device info payload structure (for packing/unpacking) */
struct ropods_device_info_payload {
    uint8_t num_sensors;
    struct ropods_device_info_entry sensors[ROPODS_BLE_MAX_DEVICES];
    uint8_t hw_revision;
    uint8_t fw_revision;
} __packed;

/* Generic control command types */
enum ropods_control_command_type {
	ROPODS_CMD_CHANNEL_ENABLE = 0x01,    /**< Enable sensor channel */
	ROPODS_CMD_CHANNEL_DISABLE = 0x02,   /**< Disable sensor channel */
	ROPODS_CMD_SET_ATTRIBUTE = 0x10,     /**< Set device/channel attribute */
	ROPODS_CMD_GET_ATTRIBUTE = 0x11,     /**< Get device/channel attribute */
	ROPODS_CMD_START_STREAMING = 0x20,   /**< Start data streaming */
	ROPODS_CMD_STOP_STREAMING = 0x21,    /**< Stop data streaming */
	ROPODS_CMD_RESET_COUNTERS = 0x22,    /**< Reset packet/sample counters */
	ROPODS_CMD_GET_STATUS = 0x30,        /**< Get device status */
	ROPODS_CMD_SYSTEM_RESET = 0xFF,      /**< System reset command */
};

/* Generic attribute IDs (can be extended per device type) */
enum ropods_attribute_id {
	ROPODS_ATTR_SAMPLE_RATE = 0x01,      /**< Sample rate in Hz */
	ROPODS_ATTR_FULL_SCALE_RANGE = 0x02, /**< Full scale range */
	ROPODS_ATTR_POWER_MODE = 0x03,       /**< Power mode */
	ROPODS_ATTR_FILTER_CONFIG = 0x04,    /**< Filter configuration */
	ROPODS_ATTR_CALIBRATION_OFFSET = 0x05, /**< Calibration offset */
	ROPODS_ATTR_CALIBRATION_SCALE = 0x06,  /**< Calibration scale */
	ROPODS_ATTR_TRIGGER_MODE = 0x07,     /**< Trigger mode */
	ROPODS_ATTR_BUFFER_SIZE = 0x08,      /**< Buffer size */
	ROPODS_ATTR_DATA_FORMAT = 0x09,      /**< Data format */
	ROPODS_ATTR_TIMESTAMP_MODE = 0x0A,   /**< Timestamp mode */
	/* Custom attributes start at 0x80 */
	ROPODS_ATTR_CUSTOM_BASE = 0x80,
};

/* Control command structure */
struct ropods_control_command {
	uint8_t command;             /**< Command type */
	uint8_t device_id;           /**< Target device ID (0 = all devices) */
	uint16_t channel_or_attr;    /**< Channel ID or attribute ID */
	uint32_t value;              /**< Command value/parameter */
} __packed;

/* Control response structure */
struct ropods_control_response {
	uint8_t command;             /**< Original command type */
	uint8_t status;              /**< Response status (0=success, non-zero=error) */
	uint8_t device_id;           /**< Device ID */
	uint8_t reserved;            /**< Reserved for alignment */
	uint32_t value;              /**< Response value (for GET commands) */
} __packed;

/* Response status codes */
enum ropods_response_status {
	ROPODS_STATUS_SUCCESS = 0x00,        /**< Command successful */
	ROPODS_STATUS_INVALID_CMD = 0x01,    /**< Invalid command */
	ROPODS_STATUS_INVALID_DEVICE = 0x02, /**< Invalid device ID */
	ROPODS_STATUS_INVALID_CHANNEL = 0x03, /**< Invalid channel */
	ROPODS_STATUS_INVALID_ATTR = 0x04,   /**< Invalid attribute */
	ROPODS_STATUS_INVALID_VALUE = 0x05,  /**< Invalid value */
	ROPODS_STATUS_DEVICE_BUSY = 0x06,    /**< Device busy */
	ROPODS_STATUS_NOT_SUPPORTED = 0x07,  /**< Operation not supported */
	ROPODS_STATUS_HARDWARE_ERROR = 0x08, /**< Hardware error */
	ROPODS_STATUS_TIMEOUT = 0x09,        /**< Operation timeout */
	ROPODS_STATUS_UNKNOWN_ERROR = 0xFF,  /**< Unknown error */
};

/* Callback function types */
typedef int (*ropods_control_callback_t)(const struct ropods_control_command *cmd, 
                                         struct ropods_control_response *resp);

/**
 * @brief Initialize the ROPODS BLE service
 * 
 * @return 0 on success, negative error code on failure
 */
int ropods_ble_service_init(void);

/**
 * @brief Start BLE advertising with ROPODS service
 * 
 * @param device_name Device name to advertise (max 29 chars)
 * @return 0 on success, negative error code on failure
 */
int ropods_ble_start_advertising(const char *device_name);

/**
 * @brief Stop BLE advertising
 * 
 * @return 0 on success, negative error code on failure
 */
int ropods_ble_stop_advertising(void);

/**
 * @brief Send data packet via BLE notification
 * 
 * @param packet Pointer to the data packet to send
 * @param packet_size Size of the packet in bytes
 * @return 0 on success, negative error code on failure
 */
int ropods_ble_send_data_packet(const struct ropods_data_packet *packet, size_t packet_size);

/**
 * @brief Send raw data via BLE notification (for custom formats)
 * 
 * @param data Pointer to raw data
 * @param data_size Size of data in bytes
 * @return 0 on success, negative error code on failure
 */
int ropods_ble_send_raw_data(const uint8_t *data, size_t data_size);

/**
 * @brief Check if a client is connected and notifications are enabled
 * 
 * @return true if connected and ready to receive data, false otherwise
 */
bool ropods_ble_is_connected(void);

/**
 * @brief Check if data streaming is active
 * 
 * @return true if streaming is active, false otherwise
 */
bool ropods_ble_is_streaming(void);

/**
 * @brief Register callback for control commands
 * 
 * @param callback Function to call when control commands are received
 */
void ropods_ble_register_control_callback(ropods_control_callback_t callback);

/**
 * @brief Update device information descriptor
 * 
 * @param sensors Array of device information structures
 * @param num_sensors Number of sensors
 * @param hw_rev Hardware revision
 * @param fw_rev Firmware revision
 * @return 0 on success, negative error code on failure
 */
int ropods_ble_update_device_info(const struct ropods_device_info_entry *sensors, uint8_t num_sensors, uint8_t hw_rev, uint8_t fw_rev);

/**
 * @brief Get connection statistics
 * 
 * @param packets_sent Number of data packets sent (output)
 * @param bytes_sent Total bytes sent (output)
 * @param connection_time Connection time in seconds (output)
 */
void ropods_ble_get_stats(uint32_t *packets_sent, uint32_t *bytes_sent, uint32_t *connection_time);

/**
 * @brief Reset connection statistics
 */
void ropods_ble_reset_stats(void);

/**
 * Register a device pointer with the BLE service.
 * @param dev Zephyr device pointer
 * @param device_id Application-defined device ID
 * @return 0 on success, negative error code on failure
 */
int ropods_ble_register_device(const struct device *dev, uint8_t device_id);

#ifdef __cplusplus
}
#endif

#endif /* ROPODS_BLE_SERVICE_H */
