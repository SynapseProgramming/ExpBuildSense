#include <stdio.h>
#include <string.h>

#include "esp_log.h"
#include "nvs_flash.h"

#include "esp_ble_mesh_defs.h"
#include "esp_bt_device.h"
#include "esp_ble_mesh_common_api.h"
#include "esp_ble_mesh_networking_api.h"
#include "esp_ble_mesh_provisioning_api.h"
#include "esp_ble_mesh_config_model_api.h"
#include "esp_ble_mesh_sensor_model_api.h"

#include "ble_mesh_example_init.h"
#include "sensors.h"

#define CID_ESP 0x02E5

void task_pub(void *ignore);

/* Sensor Property ID */
#define SENSOR_PROPERTY_ID_0 0x0056 /* Present Indoor Ambient Temperature */
#define SENSOR_PROPERTY_ID_1 0x005B /* Present Outdoor Ambient Temperature */

#define SENSOR_POSITIVE_TOLERANCE ESP_BLE_MESH_SENSOR_UNSPECIFIED_POS_TOLERANCE
#define SENSOR_NEGATIVE_TOLERANCE ESP_BLE_MESH_SENSOR_UNSPECIFIED_NEG_TOLERANCE
#define SENSOR_SAMPLE_FUNCTION ESP_BLE_MESH_SAMPLE_FUNC_UNSPECIFIED
#define SENSOR_MEASURE_PERIOD ESP_BLE_MESH_SENSOR_NOT_APPL_MEASURE_PERIOD
#define SENSOR_UPDATE_INTERVAL ESP_BLE_MESH_SENSOR_NOT_APPL_UPDATE_INTERVAL

static uint8_t dev_uuid[ESP_BLE_MESH_OCTET16_LEN] = {0x32, 0x10};
uint8_t bt_address = 0;
// characteristics is used for battery voltage.
esp_adc_cal_characteristics_t characteristics;
// send_count is used to sleep the BLE mesh.
int send_count = 0;

static esp_ble_mesh_cfg_srv_t config_server = {
    .relay = ESP_BLE_MESH_RELAY_ENABLED,
    .beacon = ESP_BLE_MESH_BEACON_ENABLED,
#if defined(CONFIG_BLE_MESH_FRIEND)
    .friend_state = ESP_BLE_MESH_FRIEND_ENABLED,
#else
    .friend_state = ESP_BLE_MESH_FRIEND_NOT_SUPPORTED,
#endif
#if defined(CONFIG_BLE_MESH_GATT_PROXY_SERVER)
    .gatt_proxy = ESP_BLE_MESH_GATT_PROXY_ENABLED,
#else
    .gatt_proxy = ESP_BLE_MESH_GATT_PROXY_NOT_SUPPORTED,
#endif
    .default_ttl = 7,
    /* 3 transmissions with 20ms interval */
    .net_transmit = ESP_BLE_MESH_TRANSMIT(2, 20),
    .relay_retransmit = ESP_BLE_MESH_TRANSMIT(2, 20),
};

NET_BUF_SIMPLE_DEFINE(sensor_data_0, 4);
static esp_ble_mesh_sensor_state_t sensor_states[1] = {
    /* Mesh Model Spec:
     * Multiple instances of the Sensor states may be present within the same model,
     * provided that each instance has a unique value of the Sensor Property ID to
     * allow the instances to be differentiated. Such sensors are known as multisensors.
     * In this example, two instances of the Sensor states within the same model are
     * provided.
     */
    [0] = {
        /* Mesh Model Spec:
         * Sensor Property ID is a 2-octet value referencing a device property
         * that describes the meaning and format of data reported by a sensor.
         * 0x0000 is prohibited.
         */
        .sensor_property_id = SENSOR_PROPERTY_ID_0,
        /* Mesh Model Spec:
         * Sensor Descriptor state represents the attributes describing the sensor
         * data. This state does not change throughout the lifetime of an element.
         */
        .descriptor.positive_tolerance = SENSOR_POSITIVE_TOLERANCE,
        .descriptor.negative_tolerance = SENSOR_NEGATIVE_TOLERANCE,
        .descriptor.sampling_function = SENSOR_SAMPLE_FUNCTION,
        .descriptor.measure_period = SENSOR_MEASURE_PERIOD,
        .descriptor.update_interval = SENSOR_UPDATE_INTERVAL,
        .sensor_data.format = ESP_BLE_MESH_SENSOR_DATA_FORMAT_A,
        .sensor_data.length = 3, /* 2 represents the length is 3 */
        .sensor_data.raw_value = &sensor_data_0,
    }};

/* 20 octets is large enough to hold two Sensor Descriptor state values. */
ESP_BLE_MESH_MODEL_PUB_DEFINE(sensor_pub, 20, ROLE_NODE);
static esp_ble_mesh_sensor_srv_t sensor_server = {
    .rsp_ctrl.get_auto_rsp = ESP_BLE_MESH_SERVER_RSP_BY_APP,
    .rsp_ctrl.set_auto_rsp = ESP_BLE_MESH_SERVER_RSP_BY_APP,
    .state_count = ARRAY_SIZE(sensor_states),
    .states = sensor_states,
};

ESP_BLE_MESH_MODEL_PUB_DEFINE(sensor_setup_pub, 20, ROLE_NODE);
static esp_ble_mesh_sensor_setup_srv_t sensor_setup_server = {
    .rsp_ctrl.get_auto_rsp = ESP_BLE_MESH_SERVER_RSP_BY_APP,
    .rsp_ctrl.set_auto_rsp = ESP_BLE_MESH_SERVER_RSP_BY_APP,
    .state_count = ARRAY_SIZE(sensor_states),
    .states = sensor_states,
};

static esp_ble_mesh_model_t root_models[] = {
    ESP_BLE_MESH_MODEL_CFG_SRV(&config_server),
    ESP_BLE_MESH_MODEL_SENSOR_SRV(&sensor_pub, &sensor_server),
    ESP_BLE_MESH_MODEL_SENSOR_SETUP_SRV(&sensor_setup_pub, &sensor_setup_server),
};

static esp_ble_mesh_elem_t elements[] = {
    ESP_BLE_MESH_ELEMENT(0, root_models, ESP_BLE_MESH_MODEL_NONE),
};

static esp_ble_mesh_comp_t composition = {
    .cid = CID_ESP,
    .elements = elements,
    .element_count = ARRAY_SIZE(elements),
};

static esp_ble_mesh_prov_t provision = {
    .uuid = dev_uuid,
};

// function is called when provisioning is done. (initial link with the main server)
static void prov_complete(uint16_t net_idx, uint16_t addr, uint8_t flags, uint32_t iv_index)
{
    ESP_LOGI(TAG, "net_idx 0x%03x, addr 0x%04x", net_idx, addr);
    ESP_LOGI(TAG, "flags 0x%02x, iv_index 0x%08x", flags, iv_index);

    // init the sensors
    ESP_ERROR_CHECK(i2c_master_init());
    ESP_ERROR_CHECK(init_BMA220());
    ESP_LOGI("INFO:", "I2C initialized successfully");
}
// function which is called when the sensor node is being provisioned with the server
static void example_ble_mesh_provisioning_cb(esp_ble_mesh_prov_cb_event_t event,
                                             esp_ble_mesh_prov_cb_param_t *param)
{
    switch (event)
    {
    case ESP_BLE_MESH_PROV_REGISTER_COMP_EVT:
        ESP_LOGI(TAG, "ESP_BLE_MESH_PROV_REGISTER_COMP_EVT, err_code %d", param->prov_register_comp.err_code);
        break;
    case ESP_BLE_MESH_NODE_PROV_ENABLE_COMP_EVT:
        ESP_LOGI(TAG, "ESP_BLE_MESH_NODE_PROV_ENABLE_COMP_EVT, err_code %d", param->node_prov_enable_comp.err_code);
        break;
    case ESP_BLE_MESH_NODE_PROV_LINK_OPEN_EVT:
        ESP_LOGI(TAG, "ESP_BLE_MESH_NODE_PROV_LINK_OPEN_EVT, bearer %s",
                 param->node_prov_link_open.bearer == ESP_BLE_MESH_PROV_ADV ? "PB-ADV" : "PB-GATT");
        break;
    case ESP_BLE_MESH_NODE_PROV_LINK_CLOSE_EVT:
        ESP_LOGI(TAG, "ESP_BLE_MESH_NODE_PROV_LINK_CLOSE_EVT, bearer %s",
                 param->node_prov_link_close.bearer == ESP_BLE_MESH_PROV_ADV ? "PB-ADV" : "PB-GATT");
        break;
    case ESP_BLE_MESH_NODE_PROV_COMPLETE_EVT:
        ESP_LOGI(TAG, "ESP_BLE_MESH_NODE_PROV_COMPLETE_EVT");
        prov_complete(param->node_prov_complete.net_idx, param->node_prov_complete.addr,
                      param->node_prov_complete.flags, param->node_prov_complete.iv_index);
        break;
    case ESP_BLE_MESH_NODE_PROV_RESET_EVT:
        ESP_LOGI(TAG, "ESP_BLE_MESH_NODE_PROV_RESET_EVT");
        break;
    case ESP_BLE_MESH_NODE_SET_UNPROV_DEV_NAME_COMP_EVT:
        ESP_LOGI(TAG, "ESP_BLE_MESH_NODE_SET_UNPROV_DEV_NAME_COMP_EVT, err_code %d", param->node_set_unprov_dev_name_comp.err_code);
        break;
    default:
        break;
    }
}

// function which is called when the server callback function is called
static void example_ble_mesh_config_server_cb(esp_ble_mesh_cfg_server_cb_event_t event,
                                              esp_ble_mesh_cfg_server_cb_param_t *param)
{
    if (event == ESP_BLE_MESH_CFG_SERVER_STATE_CHANGE_EVT)
    {
        switch (param->ctx.recv_op)
        {
        case ESP_BLE_MESH_MODEL_OP_APP_KEY_ADD:
            ESP_LOGI(TAG, "ESP_BLE_MESH_MODEL_OP_APP_KEY_ADD");
            ESP_LOGI(TAG, "net_idx 0x%04x, app_idx 0x%04x",
                     param->value.state_change.appkey_add.net_idx,
                     param->value.state_change.appkey_add.app_idx);
            ESP_LOG_BUFFER_HEX("AppKey", param->value.state_change.appkey_add.app_key, 16);
            break;
        case ESP_BLE_MESH_MODEL_OP_MODEL_APP_BIND:
            ESP_LOGI(TAG, "ESP_BLE_MESH_MODEL_OP_MODEL_APP_BIND");
            ESP_LOGI(TAG, "elem_addr 0x%04x, app_idx 0x%04x, cid 0x%04x, mod_id 0x%04x",
                     param->value.state_change.mod_app_bind.element_addr,
                     param->value.state_change.mod_app_bind.app_idx,
                     param->value.state_change.mod_app_bind.company_id,
                     param->value.state_change.mod_app_bind.model_id);
            break;
        case ESP_BLE_MESH_MODEL_OP_MODEL_SUB_ADD:
            ESP_LOGI(TAG, "ESP_BLE_MESH_MODEL_OP_MODEL_SUB_ADD");
            ESP_LOGI(TAG, "elem_addr 0x%04x, sub_addr 0x%04x, cid 0x%04x, mod_id 0x%04x",
                     param->value.state_change.mod_sub_add.element_addr,
                     param->value.state_change.mod_sub_add.sub_addr,
                     param->value.state_change.mod_sub_add.company_id,
                     param->value.state_change.mod_sub_add.model_id);
            break;
        case ESP_BLE_MESH_MODEL_OP_MODEL_PUB_SET:
            ESP_LOGI(TAG, "ESP_BLE_MESH_MODEL_OP_MODEL_PUB_SET");
            ESP_LOGI(TAG, "elem_addr 0x%04x, pub_addr 0x%04x, cid 0x%04x, mod_id 0x%04x",
                     param->value.state_change.mod_pub_set.element_addr,
                     param->value.state_change.mod_pub_set.pub_addr,
                     param->value.state_change.mod_pub_set.company_id,
                     param->value.state_change.mod_pub_set.model_id);

            // set the group address which is received by the provisioner
            sensor_server.model->pub->publish_addr = param->value.state_change.mod_pub_set.pub_addr;
            xTaskCreate(&task_pub, "task_get", 2048, NULL, 7, NULL);
            break;
        default:
            break;
        }
    }
}

struct example_sensor_descriptor
{
    uint16_t sensor_prop_id;
    uint32_t pos_tolerance : 12,
        neg_tolerance : 12,
        sample_func : 8;
    uint8_t measure_period;
    uint8_t update_interval;
} __attribute__((packed));

struct example_sensor_setting
{
    uint16_t sensor_prop_id;
    uint16_t sensor_setting_prop_id;
} __attribute__((packed));

static uint16_t example_ble_mesh_get_sensor_data(esp_ble_mesh_sensor_state_t *state, uint8_t *data)
{
    uint8_t mpid_len = 0, data_len = 0;
    uint32_t mpid = 0;

    if (state == NULL || data == NULL)
    {
        ESP_LOGE(TAG, "%s, Invalid parameter", __func__);
        return 0;
    }

    if (state->sensor_data.length == ESP_BLE_MESH_SENSOR_DATA_ZERO_LEN)
    {
        /* For zero-length sensor data, the length is 0x7F, and the format is Format B. */
        mpid = ESP_BLE_MESH_SENSOR_DATA_FORMAT_B_MPID(state->sensor_data.length, state->sensor_property_id);
        mpid_len = ESP_BLE_MESH_SENSOR_DATA_FORMAT_B_MPID_LEN;
        data_len = 0;
    }
    else
    {
        if (state->sensor_data.format == ESP_BLE_MESH_SENSOR_DATA_FORMAT_A)
        {
            mpid = ESP_BLE_MESH_SENSOR_DATA_FORMAT_A_MPID(state->sensor_data.length, state->sensor_property_id);
            mpid_len = ESP_BLE_MESH_SENSOR_DATA_FORMAT_A_MPID_LEN;
        }
        else
        {
            mpid = ESP_BLE_MESH_SENSOR_DATA_FORMAT_B_MPID(state->sensor_data.length, state->sensor_property_id);
            mpid_len = ESP_BLE_MESH_SENSOR_DATA_FORMAT_B_MPID_LEN;
        }
        /* Use "state->sensor_data.length + 1" because the length of sensor data is zero-based. */
        data_len = state->sensor_data.length + 1;
    }

    memcpy(data, &mpid, mpid_len);
    memcpy(data + mpid_len, state->sensor_data.raw_value->data, data_len);

    return (mpid_len + data_len);
}

void custom_ble_mesh_send_sensor_readings(int8_t state)
{
    // Set the new Motion State
    net_buf_simple_reset(&sensor_data_0);

    int8_t x_val = 0;
    int8_t y_val = 0;
    int8_t z_val = 0;
    uint8_t battery = 0;

    wake_BMA220();

    // update
    BMA220_getAcc(BMA220_SENSOR_GETX, &x_val);
    BMA220_getAcc(BMA220_SENSOR_GETY, &y_val);
    BMA220_getAcc(BMA220_SENSOR_GETZ, &z_val);

    ESP_LOGI("BMA220", "x value: %d", x_val);
    ESP_LOGI("BMA220", "y value: %d", y_val);
    ESP_LOGI("BMA220", "z value: %d", z_val);


    battery = battery_voltage(&characteristics);

    ESP_LOGI("BATTERY", " value: %d", battery);

    // TODO: use net_buf_simple_add to add data to the buffer
    net_buf_simple_add_u8(&sensor_data_0, (uint8_t)x_val);
    net_buf_simple_add_u8(&sensor_data_0, (uint8_t)y_val);
    net_buf_simple_add_u8(&sensor_data_0, (uint8_t)z_val);
    net_buf_simple_add_u8(&sensor_data_0, (uint8_t)battery);

    // Prep the data to be sent
    uint8_t *status = NULL;
    uint16_t buf_size = 0;
    uint16_t length = 0;
    esp_err_t err;
    int i;

    /**
     * Sensor Data state from Mesh Model Spec
     * |--------Field--------|-Size (octets)-|------------------------Notes-------------------------|
     * |----Property ID 1----|-------2-------|--ID of the 1st device property of the sensor---------|
     * |-----Raw Value 1-----|----variable---|--Raw Value field defined by the 1st device property--|
     * |----Property ID 2----|-------2-------|--ID of the 2nd device property of the sensor---------|
     * |-----Raw Value 2-----|----variable---|--Raw Value field defined by the 2nd device property--|
     * | ...... |
     * |----Property ID n----|-------2-------|--ID of the nth device property of the sensor---------|
     * |-----Raw Value n-----|----variable---|--Raw Value field defined by the nth device property--|
     */
    for (i = 0; i < ARRAY_SIZE(sensor_states); i++)
    {
        esp_ble_mesh_sensor_state_t *state = &sensor_states[i];
        if (state->sensor_data.length == ESP_BLE_MESH_SENSOR_DATA_ZERO_LEN)
        {
            buf_size += ESP_BLE_MESH_SENSOR_DATA_FORMAT_B_MPID_LEN;
        }
        else
        {
            /* Use "state->sensor_data.length + 1" because the length of sensor data is zero-based. */
            if (state->sensor_data.format == ESP_BLE_MESH_SENSOR_DATA_FORMAT_A)
            {
                buf_size += ESP_BLE_MESH_SENSOR_DATA_FORMAT_A_MPID_LEN + state->sensor_data.length + 1;
            }
            else
            {
                buf_size += ESP_BLE_MESH_SENSOR_DATA_FORMAT_B_MPID_LEN + state->sensor_data.length + 1;
            }
        }
    }

    status = calloc(1, buf_size);
    if (!status)
    {
        ESP_LOGE(TAG, "No memory for sensor status!");
        return;
    }

    for (i = 0; i < ARRAY_SIZE(sensor_states); i++)
    {
        length += example_ble_mesh_get_sensor_data(&sensor_states[i], status + length);
    }

    ESP_LOG_BUFFER_HEX("Sensor Data", status, length);
    err = esp_ble_mesh_model_publish(sensor_server.model, ESP_BLE_MESH_MODEL_OP_SENSOR_STATUS, length, status, ROLE_NODE);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to publish Sensor Status, err %d", err);
    }
    free(status);
}

static void example_ble_mesh_send_sensor_column_status(esp_ble_mesh_sensor_server_cb_param_t *param)
{
    uint8_t *status = NULL;
    uint16_t length = 0;
    esp_err_t err;

    length = ESP_BLE_MESH_SENSOR_PROPERTY_ID_LEN + param->value.get.sensor_column.raw_value_x->len;

    status = calloc(1, length);
    if (!status)
    {
        ESP_LOGE(TAG, "No memory for sensor column status!");
        return;
    }

    memcpy(status, &param->value.get.sensor_column.property_id, ESP_BLE_MESH_SENSOR_PROPERTY_ID_LEN);
    memcpy(status + ESP_BLE_MESH_SENSOR_PROPERTY_ID_LEN, param->value.get.sensor_column.raw_value_x->data,
           param->value.get.sensor_column.raw_value_x->len);

    err = esp_ble_mesh_server_model_send_msg(param->model, &param->ctx,
                                             ESP_BLE_MESH_MODEL_OP_SENSOR_COLUMN_STATUS, length, status);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to send Sensor Column Status");
    }
    free(status);
}

static void example_ble_mesh_send_sensor_series_status(esp_ble_mesh_sensor_server_cb_param_t *param)
{
    esp_err_t err;

    err = esp_ble_mesh_server_model_send_msg(param->model, &param->ctx,
                                             ESP_BLE_MESH_MODEL_OP_SENSOR_SERIES_STATUS,
                                             ESP_BLE_MESH_SENSOR_PROPERTY_ID_LEN,
                                             (uint8_t *)&param->value.get.sensor_series.property_id);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to send Sensor Column Status");
    }
}

static void example_ble_mesh_sensor_server_cb(esp_ble_mesh_sensor_server_cb_event_t event,
                                              esp_ble_mesh_sensor_server_cb_param_t *param)
{
    ESP_LOGI(TAG, "Sensor server, event %d, src 0x%04x, dst 0x%04x, model_id 0x%04x",
             event, param->ctx.addr, param->ctx.recv_dst, param->model->model_id);

    switch (event)
    {
    case ESP_BLE_MESH_SENSOR_SERVER_RECV_GET_MSG_EVT:
        switch (param->ctx.recv_op)
        {
        case ESP_BLE_MESH_MODEL_OP_SENSOR_GET:
            ESP_LOGI(TAG, "ESP_BLE_MESH_MODEL_OP_SENSOR_GET");
            break;
        case ESP_BLE_MESH_MODEL_OP_SENSOR_COLUMN_GET:
            ESP_LOGI(TAG, "ESP_BLE_MESH_MODEL_OP_SENSOR_COLUMN_GET");
            example_ble_mesh_send_sensor_column_status(param);
            break;
        case ESP_BLE_MESH_MODEL_OP_SENSOR_SERIES_GET:
            ESP_LOGI(TAG, "ESP_BLE_MESH_MODEL_OP_SENSOR_SERIES_GET");
            example_ble_mesh_send_sensor_series_status(param);
            break;
        default:
            ESP_LOGE(TAG, "Unknown Sensor Get opcode 0x%04x", param->ctx.recv_op);
            return;
        }
        break;
    case ESP_BLE_MESH_SENSOR_SERVER_RECV_SET_MSG_EVT:
        switch (param->ctx.recv_op)
        {
        case ESP_BLE_MESH_MODEL_OP_SENSOR_CADENCE_SET_UNACK:
            ESP_LOGI(TAG, "ESP_BLE_MESH_MODEL_OP_SENSOR_CADENCE_SET_UNACK");
            break;
        case ESP_BLE_MESH_MODEL_OP_SENSOR_SETTING_SET_UNACK:
            ESP_LOGI(TAG, "ESP_BLE_MESH_MODEL_OP_SENSOR_SETTING_SET_UNACK");
            break;
        default:
            ESP_LOGE(TAG, "Unknown Sensor Set opcode 0x%04x", param->ctx.recv_op);
            break;
        }
        break;
    default:
        ESP_LOGE(TAG, "Unknown Sensor Server event %d", event);
        break;
    }
}

static void example_ble_mesh_custom_model_cb(esp_ble_mesh_model_cb_event_t event,
                                             esp_ble_mesh_model_cb_param_t *param)
{
    switch (event)
    {
    case ESP_BLE_MESH_MODEL_PUBLISH_COMP_EVT:
        ESP_LOGI("INFO", "DATA SENT!");
        if (send_count >= 10)
        {
            ESP_LOGI("INFO", "SENT ENOUGH DATA! SLEEPING!");
            // esp_deep_sleep_start();
            send_count = 0;
            esp_light_sleep_start();
        }
        break;
    default:
        break;
    }
}

// TODO: 1. main function for ble mesh
static esp_err_t ble_mesh_init(void)
{
    esp_err_t err;

    // register function callbacks
    esp_ble_mesh_register_prov_callback(example_ble_mesh_provisioning_cb);
    esp_ble_mesh_register_config_server_callback(example_ble_mesh_config_server_cb);
    esp_ble_mesh_register_sensor_server_callback(example_ble_mesh_sensor_server_cb);
    esp_ble_mesh_register_custom_model_callback(example_ble_mesh_custom_model_cb);

    // provision is device uuid.
    err = esp_ble_mesh_init(&provision, &composition);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to initialize mesh stack");
        return err;
    }

    err = esp_ble_mesh_node_prov_enable(ESP_BLE_MESH_PROV_ADV | ESP_BLE_MESH_PROV_GATT);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to enable mesh node");
        return err;
    }

    ESP_LOGI(TAG, "BLE Mesh sensor server initialized");

    return ESP_OK;
}

void task_pub(void *ignore)
{
    while (1)
    {

        custom_ble_mesh_send_sensor_readings(4);
        send_count++;

        vTaskDelay(500 / portTICK_PERIOD_MS);
    }
    vTaskDelete(NULL);
}

void app_main(void)
{
    esp_err_t err;

    ESP_LOGI(TAG, "Initializing...");

    init_ADC(&characteristics);


    err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES)
    {
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(err);

    err = bluetooth_init();
    if (err)
    {
        ESP_LOGE(TAG, "esp32_bluetooth_init failed (err %d)", err);
        return;
    }

    ble_mesh_get_dev_uuid(dev_uuid);

    bt_address = *esp_bt_dev_get_address();
    ESP_LOGI("BT ADDRESS: ", "%d", bt_address);
    esp_sleep_enable_timer_wakeup(5e6);

    // sleep_BMA220();

    /* Initialize the Bluetooth Mesh Subsystem */
    err = ble_mesh_init();
    if (err)
    {
        ESP_LOGE(TAG, "Bluetooth mesh init failed (err %d)", err);
    }
}