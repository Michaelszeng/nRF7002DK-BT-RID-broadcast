/*
 * Copyright (c) 2020 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */


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

#define DEVICE_NAME             CONFIG_BT_DEVICE_NAME
#define DEVICE_NAME_LEN         (sizeof(DEVICE_NAME) - 1)

#define RUN_STATUS_LED          DK_LED1
#define CON_STATUS_LED          DK_LED2
#define RUN_LED_BLINK_INTERVAL  1000
#define NOTIFY_INTERVAL         1000

static void start_advertising_coded(struct k_work *work);

static K_WORK_DEFINE(start_advertising_worker, start_advertising_coded);

static struct bt_le_ext_adv *adv;  // create struct instance


static int create_advertising_coded(void) {
	int err;


	//                                         ctr   type  SN
	uint8_t manufDataBuff[]={0xFA, 0xFF, 0x0D, 0x77, 0x02, 0x10, 0x31, 0x35,
                             0x39, 0x36, 0x46, 0x33, 0x35, 0x30, 0x34, 0x35,
                             0x37, 0x37, 0x39, 0x31, 0x31, 0x35, 0x31, 0x35,
							 0x32, 0x33, 0x00, 0x00, 0x00};
    // memcpy(manufDataBuff+4, odid_basic_data.UASID, sizeof(manufDataBuff)-4);
    struct bt_data dt_bt_conn_ad[] = {
        {
            .type = 0x16,  // Service Data
            .data_len = sizeof(manufDataBuff),  // 30 bytes
            .data = manufDataBuff,
        }
    };

	// both primary and extended packets should be LE Coded PHY
	// advertise at 1 second interval
	struct bt_le_adv_param param = BT_LE_ADV_PARAM_INIT(BT_LE_ADV_OPT_NONE |
				    									BT_LE_ADV_OPT_EXT_ADV |
				    									BT_LE_ADV_OPT_CODED,
														1000,
														1000,
														NULL);

	err = bt_le_ext_adv_create(&param, NULL, &adv);  // set parameters for adv
	if (err) {
		printk("Failed to create advertiser set (err %d)\n", err);
		return err;
	}

	printk("Created adv: %p\n", adv);

	err = bt_le_ext_adv_set_data(adv, dt_bt_conn_ad, ARRAY_SIZE(dt_bt_conn_ad), NULL, 0);  // set data in adv
	if (err) {
		printk("Failed to set advertising data (err %d)\n", err);
		return err;
	}

	return 0;
}


static void start_advertising_coded(struct k_work *work) {
	int err;

	err = bt_le_ext_adv_start(adv, NULL);
	if (err) {
		printk("Failed to start advertising set (err %d)\n", err);
		return;
	}

	printk("Advertiser %p set started\n", adv);
}


void main(void) {
	uint32_t led_status = 0;
	int err;

	printk("Starting Bluetooth Peripheral HR coded example\n");

	err = dk_leds_init();
	if (err) {
		printk("LEDs init failed (err %d)\n", err);
		return;
	}

	err = bt_enable(NULL);
	if (err) {
		printk("Bluetooth init failed (err %d)\n", err);
		return;
	}

	printk("Bluetooth initialized\n");

	err = create_advertising_coded();
	if (err) {
		printk("Advertising failed to create (err %d)\n", err);
		return;
	}

	k_work_submit(&start_advertising_worker);
	// k_work_schedule(&notify_work, K_NO_WAIT);

	for (;;) {
		dk_set_led(RUN_STATUS_LED, (++led_status) % 2);
		k_sleep(K_MSEC(RUN_LED_BLINK_INTERVAL));
	}
}
