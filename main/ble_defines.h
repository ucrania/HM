/*
 * ble_defines.h
 *
 *  Created on: 26/10/2018
 *      Author: pi
 */

#ifndef MAIN_BLE_DEFINES_H_
#define MAIN_BLE_DEFINES_H_


#define GATTS_TAG 					"GATT_Server"
#define TEST_DEVICE_NAME            "HM_BLE_Koval"
//#define TEST_DEVICE_NAME            "HM_BLE_Calil"

#define CHAR2_FLAGS			HEART_RATE_8BIT | SENSOR_CONTACT_NOT_DETECTED | ENERGY_EXPENDED_NOT_PRESENT | RR_INTERVAL_NOT_PRESENT
#define CHAR5_FLAGS			HEART_RATE_8BIT | SENSOR_CONTACT_NOT_DETECTED | ENERGY_EXPENDED_NOT_PRESENT | RR_INTERVAL_NOT_PRESENT

#define CHAR3_FLAGS			0x0
#define CHAR6_FLAGS			0x0

static uint32_t ble_add_char_pos;
static uint8_t n_notify = 0;
static bool notify_task_running = false;

uint8_t *raw_ptr0_IR,*raw_ptr0_RED,*raw_ptr1_IR,*raw_ptr1_RED;
static uint8_t body_location1_str[] = {FINGER}; 									//Body Location:
static uint8_t HR1_str[] = {CHAR2_FLAGS,0,0,100,0};	//Heart Rate, value: 111bpm , ->3601 Kj expended Energy
static uint8_t PLX1_str[] = {CHAR3_FLAGS,0,0,0,0};		//Pulse Measurement , value: 99 , 11-> data for testing
static uint8_t body_location2_str[] = {WRIST};							 			//Body Location:
static uint8_t HR2_str[] = {CHAR5_FLAGS,0,3602&0x0F,3602&0xF0,0}; 	//Heart Rate, value: 111bpm , ->3602 Kj expended Energy
static uint8_t PLX2_str[] = {CHAR6_FLAGS,0,0,0,0};		//Pulse Measurement
static uint8_t BAT_lvl_str[] = {99};								//Battery level %
static uint8_t BAT_state_str[] = {BATT_STATE_PRESENT|BATT_STATE_DISCHARGING|BATT_STATE_NOT_CHARGING|BATT_STATE_GOOD_LEVEL};								//Battery state


esp_attr_value_t char1_BL_val = {
	.attr_max_len = 22,
	.attr_len		= sizeof(body_location1_str),
	.attr_value     = body_location1_str,
};

esp_attr_value_t char2_HR1_val = {
	.attr_max_len = 22,
	.attr_len		= sizeof(HR1_str),
	.attr_value     = HR1_str,
};

esp_attr_value_t char3_PLX1_val = {
	.attr_max_len = 22,
	.attr_len		= sizeof(PLX1_str),
	.attr_value     = PLX1_str,
};

esp_attr_value_t char4_BL2_val = {
	.attr_max_len = 22,
	.attr_len		= sizeof(body_location2_str),
	.attr_value     = body_location2_str,
};

esp_attr_value_t char5_HR2_val = {
	.attr_max_len = 22,
	.attr_len		= sizeof(HR2_str),
	.attr_value     = HR2_str,
};

esp_attr_value_t char6_PLX2_val = {
	.attr_max_len = 22,
	.attr_len		= sizeof(PLX2_str),
	.attr_value     = PLX2_str,
};

esp_attr_value_t char7_RAW1_val = {
	.attr_max_len = FIFO_A_FULL*8,
	.attr_len		= FIFO_A_FULL,
	.attr_value     = NULL,
};
esp_attr_value_t char8_RAW2_val = {
	.attr_max_len = FIFO_A_FULL*8,
	.attr_len		= FIFO_A_FULL,
	.attr_value     = NULL,
};
esp_attr_value_t char9_BAT_lvl_val = {
	.attr_max_len = 22,
	.attr_len		= sizeof(BAT_lvl_str),
	.attr_value     = BAT_lvl_str,
};
esp_attr_value_t char10_BAT_state_val = {
	.attr_max_len = 22,
	.attr_len		= sizeof(BAT_state_str),
	.attr_value     = BAT_state_str,
};

static uint8_t descr1_str[] = {0,0};

struct gatts_char_inst {
    esp_bt_uuid_t char_uuid;
    esp_gatt_perm_t char_perm;
	esp_gatt_char_prop_t char_property;
	esp_attr_value_t *char_val;
    esp_attr_control_t *char_control;
    uint16_t char_handle;
    esp_gatts_cb_t char_read_callback;
	esp_gatts_cb_t char_write_callback;
    esp_bt_uuid_t descr_uuid;
    esp_gatt_perm_t descr_perm;
	esp_attr_value_t *descr_val;
    esp_attr_control_t *descr_control;
    uint16_t descr_handle;
    esp_gatts_cb_t descr_read_callback;
	esp_gatts_cb_t descr_write_callback;
	bool is_notify;						//notify flag
};



#endif /* MAIN_BLE_DEFINES_H_ */
