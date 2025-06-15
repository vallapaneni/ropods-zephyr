/**
 * @file ble_sensor_service.c
 * @brief Implementation of BLE Service for streaming ICM20948 quaternion data
 */

#include "ble_sensor_service.h"
#include <zephyr/logging/log.h>
#include <zephyr/sys/byteorder.h>
#include <string.h>

LOG_MODULE_REGISTER(ble_sensor_service, LOG_LEVEL_INF);

/* Service state */
static struct {
	bool notifications_enabled;
	struct bt_conn *current_conn;
	int (*control_callback)(const struct control_command *cmd, struct control_response *resp);
	struct device_info device_info[8];  /* Support up to 8 devices */
	uint8_t device_count;
} service_state = {0};

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

/* Characteristic value variables */
static struct control_command control_cmd = {0};
static struct control_response control_resp = {0};

/* CCC changed callback for data stream */
static void data_stream_ccc_changed(const struct bt_gatt_attr *attr, uint16_t value)
{
	service_state.notifications_enabled = (value == BT_GATT_CCC_NOTIFY);
	LOG_INF("Data stream notifications %s", 
		service_state.notifications_enabled ? "enabled" : "disabled");
}

/* Control characteristic write callback */
static ssize_t control_write(struct bt_conn *conn,
			    const struct bt_gatt_attr *attr,
			    const void *buf, uint16_t len, uint16_t offset,
			    uint8_t flags)
{
	if (len != sizeof(struct control_command)) {
		LOG_WRN("Invalid control command length: %d (expected %d)", len, sizeof(struct control_command));
		return BT_GATT_ERR(BT_ATT_ERR_INVALID_ATTRIBUTE_LEN);
	}

	memcpy(&control_cmd, buf, sizeof(control_cmd));
	
	LOG_INF("Control command received: cmd=0x%02x, device=%d, channel/attr=0x%04x, value=%u", 
		control_cmd.command, control_cmd.device_id, control_cmd.channel_or_attr, control_cmd.value);

	/* Initialize response */
	control_resp.command = control_cmd.command;
	control_resp.device_id = control_cmd.device_id;
	control_resp.status = 0; /* Success by default */
	control_resp.value = 0;

	/* Call registered callback if available */
	if (service_state.control_callback) {
		int ret = service_state.control_callback(&control_cmd, &control_resp);
		if (ret != 0) {
			control_resp.status = (uint8_t)(-ret); /* Convert error to positive status code */
		}
	} else {
		LOG_WRN("No control callback registered");
		control_resp.status = 1; /* Error: no handler */
	}

	LOG_DBG("Control response: status=%d, value=%u", control_resp.status, control_resp.value);
	return len;
}

/* Control characteristic read callback (returns last response) */
static ssize_t control_read(struct bt_conn *conn,
			   const struct bt_gatt_attr *attr,
			   void *buf, uint16_t len, uint16_t offset)
{
	return bt_gatt_attr_read(conn, attr, buf, len, offset, 
				&control_resp, sizeof(control_resp));
}

/* Device info descriptor read callback */
static ssize_t device_info_read(struct bt_conn *conn,
			       const struct bt_gatt_attr *attr,
			       void *buf, uint16_t len, uint16_t offset)
{
	/* Return device info array */
	size_t total_size = service_state.device_count * sizeof(struct device_info);
	return bt_gatt_attr_read(conn, attr, buf, len, offset, 
				service_state.device_info, total_size);
}

/* GATT service definition */
BT_GATT_SERVICE_DEFINE(ppod_sensor_svc,
	BT_GATT_PRIMARY_SERVICE(BT_UUID_PPOD_SENSOR_SERVICE),
	
	/* Data Stream Characteristic (Read/Notify) */
	BT_GATT_CHARACTERISTIC(BT_UUID_PPOD_DATA_STREAM,
			       BT_GATT_CHRC_READ | BT_GATT_CHRC_NOTIFY,
			       BT_GATT_PERM_READ,
			       NULL, NULL, NULL),
	BT_GATT_CCC(data_stream_ccc_changed, BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),
	
	/* Control Characteristic (Read/Write) */
	BT_GATT_CHARACTERISTIC(BT_UUID_PPOD_CONTROL,
			       BT_GATT_CHRC_READ | BT_GATT_CHRC_WRITE,
			       BT_GATT_PERM_READ | BT_GATT_PERM_WRITE,
			       control_read, control_write, NULL),
	
	/* Device Info Descriptor (Read) */
	BT_GATT_DESCRIPTOR(BT_UUID_PPOD_DEVICE_INFO,
			   BT_GATT_PERM_READ,
			   device_info_read, NULL, NULL),
);

/* Advertising data */
static const struct bt_data ad[] = {
	BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
	BT_DATA_BYTES(BT_DATA_UUID128_ALL, BT_UUID_PPOD_SENSOR_SERVICE_VAL),
	BT_DATA(BT_DATA_NAME_COMPLETE, CONFIG_BT_DEVICE_NAME, sizeof(CONFIG_BT_DEVICE_NAME) - 1),
};

/* Scan response data */
static const struct bt_data sd[] = {
	BT_DATA(BT_DATA_MANUFACTURER_DATA, "ROPODS", 6),
};

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

	service_state.current_conn = bt_conn_ref(conn);
}

static void disconnected(struct bt_conn *conn, uint8_t reason)
{
	char addr[BT_ADDR_LE_STR_LEN];

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));
	LOG_INF("Disconnected from %s (reason 0x%02x)", addr, reason);

	if (service_state.current_conn) {
		bt_conn_unref(service_state.current_conn);
		service_state.current_conn = NULL;
	}
	
	service_state.notifications_enabled = false;

	/* Restart advertising */
	ble_sensor_start_advertising();
}

static void le_param_updated(struct bt_conn *conn, uint16_t interval,
			     uint16_t latency, uint16_t timeout)
{
	LOG_INF("LE connection parameters updated: interval=%d, latency=%d, timeout=%d",
		interval, latency, timeout);
}

/* Public API implementation */
int ble_sensor_service_init(void)
{
	int err;

	/* Initialize Bluetooth */
	err = bt_enable(NULL);
	if (err) {
		LOG_ERR("Bluetooth init failed (err %d)", err);
		return err;
	}

	LOG_INF("BLE sensor service initialized");
	return 0;
}

int ble_sensor_start_advertising(void)
{
	int err;
	
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

	LOG_INF("BLE advertising started");
	return 0;
}

int ble_sensor_send_data_packet(const struct sensor_data_packet *packet, size_t packet_size)
{
	if (!service_state.current_conn || !service_state.notifications_enabled) {
		return -ENOTCONN;
	}

	/* Send notification */
	return bt_gatt_notify(service_state.current_conn, &ppod_sensor_svc.attrs[1],
			     packet, packet_size);
}

void ble_sensor_register_control_callback(int (*callback)(const struct control_command *cmd, struct control_response *resp))
{
	service_state.control_callback = callback;
	LOG_INF("Control callback registered");
}

int ble_sensor_update_device_info(const struct device_info *devices, uint8_t count)
{
	if (count > ARRAY_SIZE(service_state.device_info)) {
		LOG_WRN("Too many devices: %d (max %d)", count, ARRAY_SIZE(service_state.device_info));
		return -EINVAL;
	}

	memcpy(service_state.device_info, devices, count * sizeof(struct device_info));
	service_state.device_count = count;
	
	LOG_INF("Updated device info: %d devices", count);
	return 0;
}

bool ble_sensor_is_connected(void)
{
	return (service_state.current_conn != NULL && service_state.notifications_enabled);
}
