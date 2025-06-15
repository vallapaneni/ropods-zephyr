/**
 * @file ble_sensor_service.h
 * @brief BLE Service for streaming ICM20948 quaternion and sensor data
 * 
 * This service provides BLE characteristics for streaming:
 * - Quaternion data (w, x, y, z) from game rotation vector
 * - Raw accelerometer data (x, y, z)
 * - Raw gyroscope data (x, y, z)
 * - Combined 9-axis data packets with 10 samples per transmission
 */

#ifndef BLE_SENSOR_SERVICE_H
#define BLE_SENSOR_SERVICE_H

#include <zephyr/kernel.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/hci.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/drivers/sensor.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Custom UUID for PPOD Sensor Service */
#define BT_UUID_PPOD_SENSOR_SERVICE_VAL \
	BT_UUID_128_ENCODE(0x12345678, 0x1234, 0x5678, 0x1234, 0x56789abcdef0)

/* Custom UUIDs for characteristics */
#define BT_UUID_PPOD_DATA_STREAM_VAL \
	BT_UUID_128_ENCODE(0x12345678, 0x1234, 0x5678, 0x1234, 0x56789abcdef1)

#define BT_UUID_PPOD_CONTROL_VAL \
	BT_UUID_128_ENCODE(0x12345678, 0x1234, 0x5678, 0x1234, 0x56789abcdef2)

#define BT_UUID_PPOD_DEVICE_INFO_VAL \
	BT_UUID_128_ENCODE(0x12345678, 0x1234, 0x5678, 0x1234, 0x56789abcdef3)

#define BT_UUID_PPOD_SENSOR_SERVICE      BT_UUID_DECLARE_128(BT_UUID_PPOD_SENSOR_SERVICE_VAL)
#define BT_UUID_PPOD_DATA_STREAM         BT_UUID_DECLARE_128(BT_UUID_PPOD_DATA_STREAM_VAL)
#define BT_UUID_PPOD_CONTROL             BT_UUID_DECLARE_128(BT_UUID_PPOD_CONTROL_VAL)
#define BT_UUID_PPOD_DEVICE_INFO         BT_UUID_DECLARE_128(BT_UUID_PPOD_DEVICE_INFO_VAL)

/* Generic sensor data packet structure for streaming */
struct sensor_data_packet {
	uint32_t packet_id;          /**< Packet sequence number */
	uint8_t sample_count;        /**< Number of samples in this packet */
	uint8_t device_id;           /**< Source device ID */
	uint8_t device_location;     /**< Device location */
	uint8_t reserved;            /**< Reserved for alignment */
	uint8_t data[];              /**< Variable-length sensor data */
} __packed;

/* Device information structure */
struct device_info {
	uint8_t device_id;           /**< Device identifier */
	uint8_t device_location;     /**< Device location */
	uint8_t device_type;         /**< Device type (IMU, magnetometer, etc.) */
	uint8_t num_channels;        /**< Number of available channels */
	char device_name[16];        /**< Device name string */
} __packed;

/* Generic control command structure */
enum control_command_type {
	CTRL_CMD_CHANNEL_ENABLE = 0x01,      /**< Enable/disable sensor channel */
	CTRL_CMD_CHANNEL_DISABLE = 0x02,     /**< Disable sensor channel */
	CTRL_CMD_SET_ATTRIBUTE = 0x10,       /**< Set sensor attribute by ID */
	CTRL_CMD_GET_ATTRIBUTE = 0x11,       /**< Get sensor attribute by ID */
	CTRL_CMD_START_STREAMING = 0x20,     /**< Start data streaming */
	CTRL_CMD_STOP_STREAMING = 0x21,      /**< Stop data streaming */
	CTRL_CMD_RESET_COUNTERS = 0x22,      /**< Reset packet/sample counters */
};

/* Sensor attribute IDs */
enum sensor_attribute_id {
	ATTR_SAMPLE_RATE = 0x01,             /**< Sample rate in Hz (uint16_t) */
	ATTR_FULL_SCALE_RANGE = 0x02,        /**< Full scale range (uint16_t) */
	ATTR_POWER_MODE = 0x03,              /**< Power mode (uint8_t) */
	ATTR_FILTER_CONFIG = 0x04,           /**< Filter configuration (uint8_t) */
	ATTR_CALIBRATION_OFFSET = 0x05,      /**< Calibration offset (float) */
	ATTR_CALIBRATION_SCALE = 0x06,       /**< Calibration scale (float) */
	ATTR_TRIGGER_MODE = 0x07,            /**< Trigger mode (uint8_t) */
	ATTR_BUFFER_SIZE = 0x08,             /**< Buffer size (uint16_t) */
};

/* Control command structure */
struct control_command {
	uint8_t command;             /**< Command type from control_command_type */
	uint8_t device_id;           /**< Target device ID (0 = all devices) */
	uint16_t channel_or_attr;    /**< Channel ID or attribute ID */
	uint32_t value;              /**< Command value/parameter */
} __packed;

/* Control response structure */
struct control_response {
	uint8_t command;             /**< Original command type */
	uint8_t status;              /**< Response status (0=success, non-zero=error) */
	uint8_t device_id;           /**< Device ID */
	uint8_t reserved;            /**< Reserved for alignment */
	uint32_t value;              /**< Response value (for GET commands) */
} __packed;

/**
 * @brief Initialize the BLE sensor service
 * 
 * @return 0 on success, negative error code on failure
 */
int ble_sensor_service_init(void);

/**
 * @brief Start BLE advertising
 * 
 * @return 0 on success, negative error code on failure
 */
int ble_sensor_start_advertising(void);

/**
 * @brief Send sensor data packet via BLE notification
 * 
 * @param packet Pointer to the data packet to send
 * @param packet_size Size of the packet in bytes
 * @return 0 on success, negative error code on failure
 */
int ble_sensor_send_data_packet(const struct sensor_data_packet *packet, size_t packet_size);

/**
 * @brief Check if a client is connected and notifications are enabled
 * 
 * @return true if connected and ready to receive data, false otherwise
 */
bool ble_sensor_is_connected(void);

/**
 * @brief Register callback for control commands
 * 
 * @param callback Function to call when control commands are received
 */
void ble_sensor_register_control_callback(int (*callback)(const struct control_command *cmd, struct control_response *resp));

/**
 * @brief Update device information descriptor
 * 
 * @param devices Array of device information structures
 * @param count Number of devices
 * @return 0 on success, negative error code on failure
 */
int ble_sensor_update_device_info(const struct device_info *devices, uint8_t count);

#ifdef __cplusplus
}
#endif

#endif /* BLE_SENSOR_SERVICE_H */
