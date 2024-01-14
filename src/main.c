/* main.c - Application main entry point */

/* Based on an example from Zephyr toolkit, modified by frank duignan
 * SPDX-License-Identifier: Apache-2.0
 */
/* This example advertises three services:
 * 0x1800 Generic ACCESS (GAP)
 * 0x1801 Generic Attribute (GATT - this is part of the software device and is not used nor is it apparently removable see https://devzone.nordicsemi.com/f/nordic-q-a/15076/removing-the-generic-attribute-0x1801-primary-service-if-the-service-changed-characteristic-is-not-present
 * And a custom service 1-2-3-4-0 
 * This custom service contains a custom characteristic called char_value
 */
#include <zephyr/types.h>
#include <stddef.h>
#include <string.h>
#include <errno.h>
#include <zephyr/sys/printk.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/zephyr.h>

#include <zephyr/settings/settings.h>

#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/hci.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/device.h>
#include <zephyr/drivers/sensor.h>
#include <stdio.h>
#include "matrix.h"
#include "adc.h"
#include "pwm.h"
#include "mydevice.h"

// ********************[ Start of Temperature characteristic ]**************************************
#define BT_UUID_mytemperature_ID  	   BT_UUID_128_ENCODE(1, 2, 3, 4, (uint64_t)0x8001)
static struct bt_uuid_128 mytemperature_id=BT_UUID_INIT_128(BT_UUID_mytemperature_ID); // the 128 bit UUID for this gatt value
uint32_t mytemperature;
static ssize_t read_mytemperature(struct bt_conn *conn, const struct bt_gatt_attr *attr, void *buf, uint16_t len, uint16_t offset);
static ssize_t read_mytemperature(struct bt_conn *conn, const struct bt_gatt_attr *attr, void *buf, uint16_t len, uint16_t offset)
// Reading temperature
{
	printk("Got an mytemperature read %d\n",mytemperature);
	const char *value = (const char *)&mytemperature;
	return bt_gatt_attr_read(conn, attr, buf, len, offset, value, sizeof(mytemperature)); // pass the value back up through the BLE stack
	return 0;
}

// Callback that is activated when the characteristic is written by central
static ssize_t write_mytemperature(struct bt_conn *conn, const struct bt_gatt_attr *attr, const void *buf, uint16_t len, uint16_t offset, uint8_t flags)
{
	uint8_t *value = attr->user_data;
	printk("Got an mytemperature write\n");
	memcpy(value, buf, sizeof(mytemperature)); // copy the incoming value in the memory occupied by our characateristic variable    
	return len;
}

// Arguments to BT_GATT_CHARACTERISTIC = _uuid, _props, _perm, _read, _write, _value
#define BT_GATT_mytemperature BT_GATT_CHARACTERISTIC(&mytemperature_id.uuid, BT_GATT_CHRC_READ | BT_GATT_CHRC_WRITE, BT_GATT_PERM_READ | BT_GATT_PERM_WRITE , read_mytemperature, write_mytemperature, &mytemperature)
// ********************[ End of Temperature characteristic ]**************************************

// ********************[ Start of Pressure characteristic ]**************************************
#define BT_UUID_mypressure_ID  	   BT_UUID_128_ENCODE(1, 2, 3, 4, (uint64_t)0x8002)
static struct bt_uuid_128 mypressure_id=BT_UUID_INIT_128(BT_UUID_mypressure_ID); // the 128 bit UUID for this gatt value
uint32_t mypressure;
static ssize_t read_mypressure(struct bt_conn *conn, const struct bt_gatt_attr *attr, void *buf, uint16_t len, uint16_t offset);
static ssize_t read_mypressure(struct bt_conn *conn, const struct bt_gatt_attr *attr, void *buf, uint16_t len, uint16_t offset)
//Reading pressure
{
	printk("Got an mypressure read %d\n",mypressure);
	const char *value = (const char *)&mypressure;
	return bt_gatt_attr_read(conn, attr, buf, len, offset, value, sizeof(mypressure)); // pass the value back up through the BLE stack
	return 0;
}

// Callback that is activated when the characteristic is written by central
static ssize_t write_mypressure(struct bt_conn *conn, const struct bt_gatt_attr *attr, const void *buf, uint16_t len, uint16_t offset, uint8_t flags)
{
	uint8_t *value = attr->user_data;
	printk("Got an mypressure write\n");
	memcpy(value, buf, sizeof(mypressure)); // copy the incoming value in the memory occupied by our characateristic variable    
	return len;
}

// Arguments to BT_GATT_CHARACTERISTIC = _uuid, _props, _perm, _read, _write, _value
#define BT_GATT_mypressure BT_GATT_CHARACTERISTIC(&mypressure_id.uuid, BT_GATT_CHRC_READ | BT_GATT_CHRC_WRITE, BT_GATT_PERM_READ | BT_GATT_PERM_WRITE , read_mypressure, write_mypressure, &mypressure)
// ********************[ End of Pressure characteristic ]**************************************

// ********************[ Service definition ]********************
#define BT_UUID_mytemperature_SERVICE_VAL BT_UUID_128_ENCODE(1, 2, 3, 4, (uint64_t)0x8000)
static struct bt_uuid_128 my_mytemperature_service_uuid = BT_UUID_INIT_128( BT_UUID_mytemperature_SERVICE_VAL);
    
BT_GATT_SERVICE_DEFINE(my_mytemperature_service_svc,
    BT_GATT_PRIMARY_SERVICE(&my_mytemperature_service_uuid),
    BT_GATT_mytemperature,
    BT_GATT_mypressure
);


#define BT_UUID_CUSTOM_SERVICE_VAL BT_UUID_128_ENCODE(1, 2, 3, 4, (uint64_t)0)
static struct bt_uuid_128 my_service_uuid = BT_UUID_INIT_128( BT_UUID_CUSTOM_SERVICE_VAL);
static struct bt_uuid_128 char_id=BT_UUID_INIT_128(BT_UUID_128_ENCODE(1, 2, 3, 4, (uint64_t)5)); // the 128 bit UUID for this gatt value
uint32_t char_value; // the gatt characateristic value that is being shared over BLE	
static ssize_t read_char(struct bt_conn *conn, const struct bt_gatt_attr *attr, void *buf, uint16_t len, uint16_t offset);
static ssize_t write_char(struct bt_conn *conn, const struct bt_gatt_attr *attr, const void *buf, uint16_t len, uint16_t offset, uint8_t flags);
static void subscribe_changed(const struct bt_gatt_attr *attr,uint16_t value);
volatile int Subscribed = 0;

// bt_data is an array of data structures used in advertising.
static const struct bt_data ad[] = {
	BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)), /* specify BLE advertising flags = discoverable, BR/EDR not supported (BLE only) */
	BT_DATA_BYTES(BT_DATA_UUID128_ALL, BT_UUID_CUSTOM_SERVICE_VAL /* A 128 Service UUID for the our custom service follows */),
};
	
	
BT_GATT_SERVICE_DEFINE(my_service_svc,
	BT_GATT_PRIMARY_SERVICE(&my_service_uuid),
		BT_GATT_CHARACTERISTIC(&char_id.uuid,		
		BT_GATT_CHRC_READ | BT_GATT_CHRC_WRITE |  BT_GATT_CHRC_NOTIFY,
		BT_GATT_PERM_READ | BT_GATT_PERM_WRITE,
		read_char, write_char, &char_value),
        BT_GATT_CCC(subscribe_changed,BT_GATT_PERM_READ | BT_GATT_PERM_WRITE)
);

struct bt_conn *active_conn=NULL; // use this to maintain a reference to the connection with the central device (if any)

// Callback that is activated when the characteristic is read by central
static ssize_t read_char(struct bt_conn *conn, const struct bt_gatt_attr *attr, void *buf, uint16_t len, uint16_t offset)
{
	printf("Got a read %p\n",attr);
	// Could use 'const char *value =  attr->user_data' also here if there is the char value is being maintained with the BLE STACK
	const char *value = (const char *)&char_value; // point at the value in memory
	return bt_gatt_attr_read(conn, attr, buf, len, offset, value, sizeof(char_value)); // pass the value back up through the BLE stack
}

// Callback that is activated when the characteristic is written by central
static ssize_t write_char(struct bt_conn *conn, const struct bt_gatt_attr *attr, const void *buf, uint16_t len, uint16_t offset, uint8_t flags)
{
	uint8_t *value = attr->user_data;
	printf("Got a write\n");
    len = sizeof(value); // don't want to write past the end of "value" so determine appropriate size here
	memcpy(value, buf, len); // copy the incoming value in the memory occupied by our characateristic variable
	return len;
}

// Callback that is activated when a connection with a central device is established
static void connected(struct bt_conn *conn, uint8_t err)
{
	if (err) {
		printk("Connection failed (err 0x%02x)\n", err);
	} else {
		printk("Connected\n");
		active_conn = conn;
	}
}

// Callback that is activated when a connection with a central device is taken down
static void disconnected(struct bt_conn *conn, uint8_t reason)
{
	printk("Disconnected (reason 0x%02x)\n", reason);
	active_conn = NULL;
}

// Callback that is activated when a connection with a central device subscribed
static void subscribe_changed(const struct bt_gatt_attr *attr, uint16_t value)
{   if (value==BT_GATT_CCC_NOTIFY)
    {
       printk("Subscribed\n");
       Subscribed = 1;
    }
    else
    {
        printk("Not Subscribed\n");
        Subscribed = 0;
    }
}

// structure used to pass connection callback handlers to the BLE stack
static struct bt_conn_cb conn_callbacks = {
	.connected = connected,
	.disconnected = disconnected,
};

// This is called when the BLE stack has finished initializing
static void bt_ready(void)
{
	int err;
	printk("Bluetooth initialized\n");

// start advertising see https://developer.nordicsemi.com/nRF_Connect_SDK/doc/latest/zephyr/reference/bluetooth/gap.html

// Start BLE advertising using the ad array 
	err = bt_le_adv_start(BT_LE_ADV_CONN_NAME, ad, ARRAY_SIZE(ad), NULL, 0);
	if (err) {
		printk("Advertising failed to start (err %d)\n", err);
		return;
	}
	printk("Advertising successfully started\n");
}

void main(void)
{
	int err;
	err = device_begin();
	//error handling
	if(err)
	{
		printk("Error starting I2C1 bus %d\n",err);
	}
	else
	{
		printk("I2C1 success\n");
	}
	uint8_t test;
	err = mydevice_readRegister(9, &test);
	printk("Test Number %d\n",test);
	if(err)
	{
		printk("Error reading device %d\n",err);
	}
	else
	{
		printk("Device read ok\n");
	}
	err = adc_begin();
	err = pwm_begin();
	matrix_begin();
	matrix_all_off();
	err = bt_enable(NULL);
	if (err) {
		printk("Bluetooth init failed (err %d)\n", err);
		return;
	}

	//printing readings
	bt_ready();
	bt_conn_cb_register(&conn_callbacks);
	printk("Zephyr Microbit V2 minimal BLE example! %s\n", CONFIG_BOARD);			
	while (1) {
        printf("Temp = %d\n",mydevice_readTemperature());
		printf("Pressure = %d\n",mydevice_readPressure());
		
		mytemperature = mydevice_readTemperature();
		mypressure = mydevice_readPressure();
		k_sleep(K_SECONDS(1));
		if(char_value == 0)
		{
			matrix_all_off();
		}
		else
		{
			matrix_put_pattern(0x1f,0);
		}
		char_value++;
		
		if (active_conn)
		{
            if (Subscribed)
            {
                bt_gatt_notify(active_conn,&my_service_svc.attrs[2], &char_value,sizeof(char_value));

            }
        }
	}
	
}
