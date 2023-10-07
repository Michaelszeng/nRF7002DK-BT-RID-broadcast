#include <stddef.h>
#include <string.h>
#include <errno.h>
#include <zephyr/kernel.h>
#include <zephyr/types.h>
#include <zephyr/sys/printk.h>

#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/bluetooth/services/bas.h>
#include <zephyr/bluetooth/services/hrs.h>

#include <dk_buttons_and_leds.h>


#include "opendroneid.h"

#define RUN_STATUS_LED DK_LED1

static struct bt_le_ext_adv *adv;  // create advertising set struct

static void start_advertising(struct k_work *work);
static K_WORK_DEFINE(start_advertising_worker, start_advertising);

// message counters for ODID messages (per ASTM requirement)
static uint8_t message_ctr_legacy_basic_id;
static uint8_t message_ctr_legacy_location;
static uint8_t message_ctr_legacy_system;
static uint8_t message_ctr_coded;


static int create_advertisement_coded(void) {
	/**
	 * Runs once, to perform all 1-time setup for coded advertisig.
 	 */

	int err;

	message_ctr_coded = 0;

	// both primary and extended packets should be LE Coded PHY
	struct bt_le_adv_param param = BT_LE_ADV_PARAM_INIT(BT_LE_ADV_OPT_NONE |
				    									BT_LE_ADV_OPT_EXT_ADV |
				    									BT_LE_ADV_OPT_CODED,
														1000,  // 1 sec interval min
														1000,  // 1 sec interval max
														NULL);

	err = bt_le_ext_adv_create(&param, NULL, &adv);  // set parameters for adv
	if (err) {
		printk("bt_le_ext_adv_create failed (err %d)\n", err);
		return err;
	}

	return 0;
}


static int create_advertisement_legacy(void) {
	/**
	 * Runs once, to perform all 1-time setup for legacy advertising.
 	 */
	message_ctr_legacy_basic_id = 0;
	message_ctr_legacy_location = 0;
	message_ctr_legacy_system = 0;

	return 0;
}


static int set_advertising_data(void) {
	/**
	 * Runs every loop to update the data contained in the advertisement.
 	 */

	int err;

	// Create Basic ID message
	ODID_BasicID_data basic_id_data;
	odid_initBasicIDData(&basic_id_data);
	basic_id_data.UAType = 0;
	basic_id_data.IDType = 1;  // SN
	char uas_id_string[] = "1819ZNG000007";
	int uas_id_len = sizeof(uas_id_string) / sizeof(char);
	memcpy(basic_id_data.UASID, uas_id_string, uas_id_len);
	ODID_BasicID_encoded basic_id_encoded;
	err = encodeBasicIDMessage(&basic_id_encoded, &basic_id_data);
	if (err == ODID_FAIL) printk("encodeBasicIDMessage failed.\n");
	printk("\nUAType: %d\n", basic_id_encoded.UAType);
	printk("IDType: %d\n", basic_id_encoded.IDType);
	printk("UASID: ");
	for (int i=0; i<20; i++) {
		printk("%c", basic_id_encoded.UASID[i]);
	}
	printk("\n\n");

	// Create Location Message
	ODID_Location_data location_data;
	odid_initLocationData(&location_data);  // Automatically sets AltitudeBaro invalid
	location_data.Status = 2;  // Airborne
	location_data.Direction = 142;
	location_data.SpeedHorizontal = 0;
	location_data.SpeedVertical = 0;
	location_data.Latitude = 36.345;
	location_data.Longitude = 153.456;
	location_data.AltitudeGeo = 50.2;  // meter (WGS84-HAE)
	location_data.HeightType = 0;  // 0 = height reference above takeoff, 1 = height reference above ground
	location_data.Height = 0;
	location_data.HorizAccuracy = ODID_HOR_ACC_3_METER;
	location_data.VertAccuracy = ODID_VER_ACC_UNKNOWN;
	location_data.BaroAccuracy = ODID_VER_ACC_UNKNOWN;
	location_data.SpeedAccuracy = ODID_SPEED_ACC_10_METERS_PER_SECOND;
	location_data.TSAccuracy = ODID_TIME_ACC_UNKNOWN;
	location_data.TimeStamp = 0;  // seconds after the full hour relative to UTC. 0xFFFF for unknown.
	ODID_Location_encoded location_encoded;
	err = encodeLocationMessage(&location_encoded, &location_data);
	if (err == ODID_FAIL) printk("encodeLocationMessage failed.\n");
	printk("\nSpeedMult: %d\n", location_encoded.SpeedMult);
	printk("EWDirection: %d\n", location_encoded.EWDirection);
	printk("HeightType: %d\n", location_encoded.HeightType);  // 0 = height reference above takeoff
	printk("Status: %d\n", location_encoded.Status);  // 2 = Airborne
	printk("Direction: %d\n", location_encoded.Direction);
	printk("SpeedHorizontal: %d\n", location_encoded.SpeedHorizontal);
	printk("SpeedVertical: %d\n", location_encoded.SpeedVertical);
	printk("Latitude: %d\n", location_encoded.Latitude);
	printk("Longitude: %d\n", location_encoded.Longitude);
	printk("AltitudeBaro: %d\n", location_encoded.AltitudeBaro);
	printk("AltitudeGeo: %d\n", location_encoded.AltitudeGeo);  // Some weird encoding quirk
	printk("Height: %d\n", location_encoded.Height);  // Some weird encoding quirk
	printk("HorizAccuracy: %d\n", location_encoded.HorizAccuracy);
	printk("VertAccuracy: %d\n", location_encoded.VertAccuracy);
	printk("SpeedAccuracy: %d\n", location_encoded.SpeedAccuracy);
	printk("BaroAccuracy: %d\n", location_encoded.BaroAccuracy);
	printk("TimeStamp: %d\n", location_encoded.TimeStamp);
	printk("TSAccuracy: %d\n", location_encoded.TSAccuracy);

	// Create System Message
	ODID_System_data system_data;
	odid_initSystemData(&system_data);  // Automatically sets Area count, radius, ceiling, floor to invalid
	system_data.OperatorLocationType = ODID_OPERATOR_LOCATION_TYPE_TAKEOFF;
	system_data.ClassificationType = ODID_CLASSIFICATION_TYPE_UNDECLARED;
	system_data.OperatorLatitude = 0;
	system_data.OperatorLongitude = 0;
	system_data.OperatorAltitudeGeo = 0;
	system_data.Timestamp = 0;  // Relative to 00:00:00 01/01/2019 UTC/Unix Time
	ODID_System_encoded system_encoded;
	err = encodeSystemMessage(&system_encoded, &system_data);
	if (err == ODID_FAIL) printk("encodeSystemMessage failed.\n");
	printk("\nOperatorLocationType: %d\n", system_encoded.OperatorLocationType);  // 0 = Takeoff
	printk("ClassificationType: %d\n", system_encoded.ClassificationType);  // 0 = Undeclared
	printk("OperatorLatitude: %d\n", system_encoded.OperatorLatitude);
	printk("OperatorLongitude: %d\n", system_encoded.OperatorLongitude);
	printk("OperatorAltitudeGeo: %d\n", system_encoded.OperatorAltitudeGeo);  // Some weird encoding quirk
	printk("AreaCount: %d\n", system_encoded.AreaCount);
	printk("AreaRadius: %d\n", system_encoded.AreaRadius);
	printk("AreaCeiling: %d\n", system_encoded.AreaCeiling);
	printk("AreaFloor: %d\n", system_encoded.AreaFloor);
	printk("Timestamp: %d\n", system_encoded.Timestamp);
	
	// Increment message pack counter (until they reach 0xFF, in which case reset to 0)
	if (message_ctr_coded >= 0xFF) {
		message_ctr_coded = 0;
	}
	else {
		message_ctr_coded++;
	}

	// Increment legacy message counters (until they reach 0xFF, in which case reset to 0)
	if (message_ctr_legacy_basic_id >= 0xFF) {
		message_ctr_legacy_basic_id = 0;
	}
	else {
		message_ctr_legacy_basic_id++;
	}
	if (message_ctr_legacy_location >= 0xFF) {
		message_ctr_legacy_location = 0;
	}
	else {
		message_ctr_legacy_location++;
	}
	if (message_ctr_legacy_system >= 0xFF) {
		message_ctr_legacy_system = 0;
	}
	else {
		message_ctr_legacy_system++;
	}

	// Create Hex Array that will be broadcast over BT
	//                                          ctr   pack  size  num_msgs    
	uint8_t codedAdvBuff[82]={0xFA, 0xFF, 0x0D, 0x77, 0xF2, 0x19, 0x3};
	memcpy(codedAdvBuff+3, &message_ctr_coded, sizeof(message_ctr_coded));  // Set message counter
    memcpy(codedAdvBuff+7, &basic_id_encoded, sizeof(basic_id_encoded));
	memcpy(codedAdvBuff+7+25, &location_encoded, sizeof(location_encoded));
	memcpy(codedAdvBuff+7+50, &system_encoded, sizeof(system_encoded));

    struct bt_data dt_bt_ad[] = {
        {
            .type = 0x16,  // Service Data
            .data_len = sizeof(codedAdvBuff),
            .data = codedAdvBuff,
        }
    };

	err = bt_le_ext_adv_set_data(adv, dt_bt_ad, ARRAY_SIZE(dt_bt_ad), NULL, 0);  // set data in adv
	if (err) {
		printk("bt_le_ext_adv_set_data failed (err %d)\n", err);
		return err;
	}

	return 0;
}


static void start_advertising(struct k_work *work) {
	/**
	 * Function called in work queue repeatedly to update coded advertising data and broadcast.
	 */

	int err;

	err = set_advertising_data();
	if (err) {
		printk("set_advertising_data failed (err %d)\n", err);
		return;
	}
	

	///////////////////////////////////////////////////////////////////
	// CODED PHY
	///////////////////////////////////////////////////////////////////
	err = bt_le_ext_adv_start(adv, NULL);
	if (err) {
		printk("bt_le_ext_adv_start failed (err %d)\n", err);
		return;
	}
	printk("Coded advertiser %p set started.\n", adv);

	k_sleep(K_MSEC(300));

	bt_le_ext_adv_delete(adv);  // must delete advertising set so that Legacy advertisement can happen


	///////////////////////////////////////////////////////////////////
	// LEGACY PHY
	///////////////////////////////////////////////////////////////////
	
	//                                             ctr   type  SN
	uint8_t basic_id_adv_buff[]={0xFA, 0xFF, 0x0D, 0x77, 0x02, 0x10, 0x31, 0x35,
                             	 0x39, 0x36, 0x46, 0x33, 0x35, 0x30, 0x34, 0x35,
                             	 0x37, 0x37, 0x39, 0x31, 0x31, 0x35, 0x31, 0x35,
							 	 0x32, 0x33, 0x00, 0x00, 0x00};
    memcpy(basic_id_adv_buff+3, &message_ctr_legacy_basic_id, sizeof(message_ctr_legacy_basic_id));

    struct bt_data dt_bt_ad[] = {
        {
            .type = 0x16,  // Service Data
            .data_len = sizeof(basic_id_adv_buff),  // 30 bytes
            .data = basic_id_adv_buff,
        }
    };

	struct bt_le_adv_param legacy_adv_param = *(BT_LE_ADV_NCONN);
	legacy_adv_param.interval_min = 1000;
	legacy_adv_param.interval_max = 1000;

	err = bt_le_adv_start(&legacy_adv_param, dt_bt_ad, ARRAY_SIZE(dt_bt_ad), NULL, 0);
	if (err) {
		printk("bt_le_adv_start failed (err %d)\n", err);
		return;
	}
	printk("Legacy advertiser set started.\n");
}


int main(void) {
	uint32_t led_status = 0;
	int err;

	printk("\n\n\n==================PROGRAM STARTING==================\n");

	err = dk_leds_init();
	if (err) {
		printk("dk_leds_init failed (err %d)\n", err);
		return -1;
	}

	// ENABLE BLUETOOTH
	err = bt_enable(NULL);
	if (err) {
		printk("bt_enable failed (err %d)\n", err);
		return -1;
	}
	printk("Bluetooth initialized\n");

	// CREATE ADVERTISEMENTS
	err = create_advertisement_coded();
	if (err) {
		printk("create_advertisement_coded failed (err %d)\n", err);
		return -1;
	}
	err = create_advertisement_legacy();
	if (err) {
		printk("create_advertisement_legacy failed (err %d)\n", err);
		return -1;
	}

	// BEGIN THREADS TO BROADCAST ADVERTISEMENTS
	k_work_submit(&start_advertising_worker);

	for (;;) {
		dk_set_led(RUN_STATUS_LED, (++led_status) % 2);
		k_sleep(K_MSEC(1000));
	}

	return 0;
}