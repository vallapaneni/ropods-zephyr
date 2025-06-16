/**
 * @file ropods_ble_service.c
 * @brief Generic ROPODS BLE Service implementation
 */

#include "ropods_ble_service.h"
#include <zephyr/logging/log.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/hci.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/bluetooth/services/dis.h>
#include <zephyr/sys/atomic.h>

LOG_MODULE_REGISTER(ropods_ble_service, CONFIG_ROPODS_BLE_LOG_LEVEL);

/* Service state */
static struct bt_conn *current_conn = NULL;
static atomic_t notification_enabled = ATOMIC_INIT(0);
static atomic_t streaming_active = ATOMIC_INIT(0);

/* Statistics */
static uint32_t packets_sent = 0;
static uint32_t bytes_sent = 0;
static int64_t connection_start_time = 0;

/* Control callback */
static ropods_control_callback_t control_callback = NULL;

/* Characteristic value buffers */
static struct ropods_control_command control_cmd = {0};
static struct ropods_control_response control_resp = {0};
static uint8_t device_info_buffer[256] = {0};
static size_t device_info_size = 0;

/* Forward declarations */
static void connected(struct bt_conn *conn, uint8_t err);
static void disconnected(struct bt_conn *conn, uint8_t reason);
static void le_param_updated(struct bt_conn *conn, uint16_t interval, 
                           uint16_t latency, uint16_t timeout);

/* Connection callbacks */
BT_CONN_CB_DEFINE(conn_callbacks) = {
	.connected = connected,
	.disconnected = disconnected,
	.le_param_updated = le_param_updated,
};

/* CCC changed callback for data stream */
static void data_stream_ccc_changed(const struct bt_gatt_attr *attr, uint16_t value)
{
	bool notifications_enabled = (value == BT_GATT_CCC_NOTIFY);
	
	atomic_set(&notification_enabled, notifications_enabled ? 1 : 0);
	
	LOG_INF("Data stream notifications %s", notifications_enabled ? "enabled" : "disabled");
}

/* Control characteristic read callback */
static ssize_t control_read(struct bt_conn *conn, const struct bt_gatt_attr *attr,
                           void *buf, uint16_t len, uint16_t offset)
{
	return bt_gatt_attr_read(conn, attr, buf, len, offset, &control_resp,
				 sizeof(control_resp));
}

/* Control characteristic write callback */
static ssize_t control_write(struct bt_conn *conn, const struct bt_gatt_attr *attr,
                            const void *buf, uint16_t len, uint16_t offset, uint8_t flags)
{
	if (offset != 0) {
		return BT_GATT_ERR(BT_ATT_ERR_INVALID_OFFSET);
	}

	if (len != sizeof(struct ropods_control_command)) {
		return BT_GATT_ERR(BT_ATT_ERR_INVALID_ATTRIBUTE_LEN);
	}

	memcpy(&control_cmd, buf, len);
	
	LOG_DBG("Control command received: cmd=0x%02x, device=%d, attr/ch=%d, value=%d",
	        control_cmd.command, control_cmd.device_id, 
	        control_cmd.channel_or_attr, control_cmd.value);

	/* Initialize response */
	control_resp.command = control_cmd.command;
	control_resp.device_id = control_cmd.device_id;
	control_resp.status = ROPODS_STATUS_SUCCESS;
	control_resp.value = 0;

	/* Handle streaming control commands directly */
	switch (control_cmd.command) {
	case ROPODS_CMD_START_STREAMING:
		atomic_set(&streaming_active, 1);
		LOG_INF("Data streaming started");
		break;
		
	case ROPODS_CMD_STOP_STREAMING:
		atomic_set(&streaming_active, 0);
		LOG_INF("Data streaming stopped");
		break;
		
	case ROPODS_CMD_RESET_COUNTERS:
		ropods_ble_reset_stats();
		LOG_INF("Statistics reset");
		break;
		
	default:
		/* Forward to application callback */
		if (control_callback) {
			int ret = control_callback(&control_cmd, &control_resp);
			if (ret != 0) {
				control_resp.status = ROPODS_STATUS_UNKNOWN_ERROR;
				LOG_ERR("Control callback failed: %d", ret);
			}
		} else {
			control_resp.status = ROPODS_STATUS_NOT_SUPPORTED;
			LOG_WRN("No control callback registered for command 0x%02x", 
			        control_cmd.command);
		}
		break;
	}

	return len;
}

/* Device info characteristic read callback */
static ssize_t device_info_read(struct bt_conn *conn, const struct bt_gatt_attr *attr,
                               void *buf, uint16_t len, uint16_t offset)
{
	return bt_gatt_attr_read(conn, attr, buf, len, offset, 
	                        device_info_buffer, device_info_size);
}

/* GATT service definition */
BT_GATT_SERVICE_DEFINE(ropods_service,
	/* Primary Service */
	BT_GATT_PRIMARY_SERVICE(BT_UUID_ROPODS_SERVICE),
	
	/* Data Stream Characteristic */
	BT_GATT_CHARACTERISTIC(BT_UUID_ROPODS_DATA_STREAM,
			       BT_GATT_CHRC_READ | BT_GATT_CHRC_NOTIFY,
			       BT_GATT_PERM_READ, NULL, NULL, NULL),
	BT_GATT_CCC(data_stream_ccc_changed, BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),
	
	/* Control Characteristic */
	BT_GATT_CHARACTERISTIC(BT_UUID_ROPODS_CONTROL,
			       BT_GATT_CHRC_READ | BT_GATT_CHRC_WRITE,
			       BT_GATT_PERM_READ | BT_GATT_PERM_WRITE,
			       control_read, control_write, NULL),
	
	/* Device Info Characteristic */
	BT_GATT_CHARACTERISTIC(BT_UUID_ROPODS_DEVICE_INFO,
			       BT_GATT_CHRC_READ,
			       BT_GATT_PERM_READ,
			       device_info_read, NULL, NULL),
);

/* Connection event handlers */
static void connected(struct bt_conn *conn, uint8_t err)
{
	char addr[BT_ADDR_LE_STR_LEN];

	if (err) {
		LOG_ERR("Connection failed (err 0x%02x)", err);
		return;
	}

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));
	LOG_INF("Connected to %s", addr);

	current_conn = bt_conn_ref(conn);
	connection_start_time = k_uptime_get();
	
	/* Reset statistics on new connection */
	ropods_ble_reset_stats();
}

static void disconnected(struct bt_conn *conn, uint8_t reason)
{
	char addr[BT_ADDR_LE_STR_LEN];

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));
	LOG_INF("Disconnected from %s (reason 0x%02x)", addr, reason);

	if (current_conn) {
		bt_conn_unref(current_conn);
		current_conn = NULL;
	}
	
	atomic_set(&notification_enabled, 0);
	atomic_set(&streaming_active, 0);
	connection_start_time = 0;
}

static void le_param_updated(struct bt_conn *conn, uint16_t interval,
			    uint16_t latency, uint16_t timeout)
{
	LOG_DBG("Connection parameters updated: interval=%d, latency=%d, timeout=%d",
	        interval, latency, timeout);
}

/* Public API Implementation */

int ropods_ble_service_init(void)
{
	int err;

	LOG_INF("Initializing ROPODS BLE service");

	err = bt_enable(NULL);
	if (err) {
		LOG_ERR("Bluetooth init failed (err %d)", err);
		return err;
	}

	LOG_INF("ROPODS BLE service initialized successfully");
	return 0;
}

int ropods_ble_start_advertising(const char *device_name)
{
	int err;
	
	/* Set device name if provided */
	if (device_name) {
		err = bt_set_name(device_name);
		if (err) {
			LOG_ERR("Failed to set device name (err %d)", err);
			return err;
		}
	}

	/* Advertising data - define all data statically to avoid compiler issues */
	static const uint8_t flags_data[] = { BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR };
	static const uint8_t ropods_service_uuid[] = {
		0x9e, 0xca, 0xdc, 0x24, 0x0e, 0xe5, 0xa9, 0xe0,
		0x93, 0xf3, 0xa3, 0xb5, 0x01, 0x00, 0x40, 0x6e
	};
	
	static const struct bt_data ad[] = {
		BT_DATA(BT_DATA_FLAGS, flags_data, sizeof(flags_data)),
		BT_DATA(BT_DATA_UUID128_ALL, ropods_service_uuid, sizeof(ropods_service_uuid)),
	};

	/* Scan response data */
	static const struct bt_data sd[] = {
		BT_DATA(BT_DATA_NAME_COMPLETE, NULL, 0), /* Name will be set automatically */
	};

	/* Advertising parameters */
	static const struct bt_le_adv_param adv_param = {
		.id = BT_ID_DEFAULT,
		.sid = 0,
		.secondary_max_skip = 0,
		.options = BT_LE_ADV_OPT_USE_IDENTITY,
		.interval_min = BT_GAP_ADV_FAST_INT_MIN_2,
		.interval_max = BT_GAP_ADV_FAST_INT_MAX_2,
		.peer = NULL,
	};

	err = bt_le_adv_start(&adv_param, ad, ARRAY_SIZE(ad), sd, ARRAY_SIZE(sd));
	if (err) {
		LOG_ERR("Advertising failed to start (err %d)", err);
		return err;
	}

	LOG_INF("ROPODS BLE advertising started");
	return 0;
}

int ropods_ble_stop_advertising(void)
{
	int err;

	err = bt_le_adv_stop();
	if (err) {
		LOG_ERR("Advertising failed to stop (err %d)", err);
		return err;
	}

	LOG_INF("ROPODS BLE advertising stopped");
	return 0;
}

int ropods_ble_send_data_packet(const struct ropods_data_packet *packet, size_t packet_size)
{
	if (!packet || packet_size == 0) {
		return -EINVAL;
	}

	return ropods_ble_send_raw_data((const uint8_t *)packet, packet_size);
}

int ropods_ble_send_raw_data(const uint8_t *data, size_t data_size)
{
	int err;

	if (!data || data_size == 0) {
		return -EINVAL;
	}

	if (data_size > ROPODS_BLE_MAX_DATA_SIZE) {
		LOG_WRN("Data size %zu exceeds maximum %d", data_size, ROPODS_BLE_MAX_DATA_SIZE);
		return -EMSGSIZE;
	}

	if (!current_conn || !atomic_get(&notification_enabled)) {
		return -ENOTCONN;
	}

	err = bt_gatt_notify(current_conn, &ropods_service.attrs[1], data, data_size);
	if (err) {
		LOG_ERR("Failed to send notification (err %d)", err);
		return err;
	}

	/* Update statistics */
	packets_sent++;
	bytes_sent += data_size;

	LOG_DBG("Sent data packet: size=%zu, total_packets=%u, total_bytes=%u",
	        data_size, packets_sent, bytes_sent);

	return 0;
}

bool ropods_ble_is_connected(void)
{
	return (current_conn != NULL) && atomic_get(&notification_enabled);
}

bool ropods_ble_is_streaming(void)
{
	return ropods_ble_is_connected() && atomic_get(&streaming_active);
}

void ropods_ble_register_control_callback(ropods_control_callback_t callback)
{
	control_callback = callback;
	LOG_INF("Control callback %sregistered", callback ? "" : "un");
}

int ropods_ble_update_device_info(const struct ropods_device_info *devices, uint8_t count)
{
	size_t total_size = 0;
	
	if (!devices || count == 0) {
		device_info_size = 0;
		return 0;
	}

	/* Calculate total size needed */
	total_size = sizeof(uint8_t) + (count * sizeof(struct ropods_device_info));
	
	if (total_size > sizeof(device_info_buffer)) {
		LOG_ERR("Device info too large: %zu > %zu", total_size, sizeof(device_info_buffer));
		return -ENOMEM;
	}

	/* Pack device info */
	device_info_buffer[0] = count;
	memcpy(&device_info_buffer[1], devices, count * sizeof(struct ropods_device_info));
	device_info_size = total_size;

	LOG_INF("Updated device info: %d devices, %zu bytes", count, device_info_size);
	return 0;
}

void ropods_ble_get_stats(uint32_t *packets_sent_out, uint32_t *bytes_sent_out, 
                         uint32_t *connection_time_out)
{
	if (packets_sent_out) {
		*packets_sent_out = packets_sent;
	}
	
	if (bytes_sent_out) {
		*bytes_sent_out = bytes_sent;
	}
	
	if (connection_time_out) {
		if (connection_start_time > 0) {
			*connection_time_out = (uint32_t)((k_uptime_get() - connection_start_time) / 1000);
		} else {
			*connection_time_out = 0;
		}
	}
}

void ropods_ble_reset_stats(void)
{
	packets_sent = 0;
	bytes_sent = 0;
	
	LOG_DBG("Statistics reset");
}
