/*
 * Copyright (c) 2018 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

#include <zephyr/types.h>
#include <stddef.h>
#include <string.h>
#include <errno.h>
#include <zephyr/sys/printk.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/kernel.h>
//
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/gpio.h>
#include <soc.h>

#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/hci.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/bluetooth/gatt.h>

#include "lbs.h"

#include <nrfx_timer.h>
#include <nrfx_systick.h>
#include <zephyr/settings/settings.h>
#include <zephyr/logging/log.h>

//#include <dk_buttons_and_leds.h>

//#include <zephyr/dk_buttons_and_leds.h>

#define CONFIG_APP_VERSION	"1.0.0"

#define DEVICE_NAME             CONFIG_BT_DEVICE_NAME
#define DEVICE_NAME_LEN         (sizeof(DEVICE_NAME) - 1)

#define BUTTON0_GPIO_PIN 		11
#define BUTTON1_GPIO_PIN		12
#define BUTTON2_GPIO_PIN		24
#define BUTTON3_GPIO_PIN		25

#define USER_BUTTON		 		BUTTON0_GPIO_PIN

//
#define LED0_GPIO_PIN 			13
#define LED1_GPIO_PIN			14
#define LED2_GPIO_PIN			15
#define LED3_GPIO_PIN			16
	
#define LED_ONE	 				LED0_GPIO_PIN
#define LED_TWO 				LED1_GPIO_PIN
#define LED_THREE 				LED2_GPIO_PIN
#define LED_FOUR 				LED3_GPIO_PIN

// TopSensor 
#define TRIG_TOP_SENSOR 		8 // Output pin
#define ECHO_TOP_SENSOR 		7 // Input pin
// Left Sensor
#define TRIG_LEFT_SENSOR 		6 // Output pin
#define ECHO_LEFT_SENSOR 		5 // Input pin
// Front Sensor
#define TRIG_FRONT_SENSOR 		4 // Output pin
#define ECHO_FRONT_SENSOR 		3 // Input pin
// Right Sensor
#define TRIG_RIGHT_SENSOR 		2 // Output pin
#define ECHO_RIGHT_SENSOR 		1 // Input pin

#define RUN_STATUS_LED          LED_ONE
#define CON_STATUS_LED          LED_TWO
#define RUN_LED_BLINK_INTERVAL  1000

#define BUZZER					28
#define VIBRATION_MOTOR			29
#define USER_LED                LED_THREE

//#define USER_BUTTON             DK_BTN1_MSK

/* 1000 msec = 1 sec */
#define SLEEP_TIME_MS 			1000
#define SLEEP_TIME_S			1000
#define SLEEP_TIME_HALF_S		500
#define SLEEP_TIME_QUOTA_S		250

#define MY_STACK_SIZE 	512
#define MY_PRIORITY 	5

K_THREAD_STACK_DEFINE(my_stack_area, MY_STACK_SIZE);

struct k_work_q my_work_q;

/* Option 1: by node label */
#define DEVICE_GPIO0 DT_NODELABEL(gpio0)
#define DEVICE_GPIO1 DT_NODELABEL(gpio1)

#define CONFIG_A_SMART_WHITE_CANE_LOG_LEVEL	4

// const struct device *gpio_dev;
const struct device *gpio0_dev = DEVICE_DT_GET(DEVICE_GPIO0);
// const struct device *gpio_dev;
const struct device *gpio1_dev = DEVICE_DT_GET(DEVICE_GPIO1);

static bool app_button_state;

LOG_MODULE_REGISTER(a_smart_white_cane, CONFIG_A_SMART_WHITE_CANE_LOG_LEVEL);

// counter
//static volatile uint32_t tCount = 0;
static uint32_t tCount = 0;
// count to us (micro seconds) conversion factor
// set in start_timer()
static volatile float countToUs = 1;
static float dist;
static float dist_top;
static float dist_1eft;
static float dist_front;
static float dist_right;
static uint8_t prescaler = 1;
static uint16_t comp1 = 500;
// 
uint32_t dist_top_sensor = 0;
uint32_t dist_1eft_sensor = 0;
uint32_t dist_front_sensor = 0;
uint32_t dist_right_sensor = 0;

// 
static struct k_timer timer_top_sensor;
static struct k_timer timer_1eft_sensor;
static struct k_timer timer_front_sensor;
static struct k_timer timer_right_sensor;

/********************************************************************************
 *
 ********************************************************************************/
static bool getDistanceTopSensor(float* dist);
static bool getDistanceLeftSensor(float* dist);
static bool getDistanceFrontSensor(float* dist);
//static bool getDistanceRightSensor(float* dist);
static void getDistanceRightSensor(void);
static void timer1_init(void);
static void set_conversion_factor(void);

/********************************************************************************
 *
 ********************************************************************************/
// set conversion factor
static void set_conversion_factor(void)
{
	countToUs = 0.0625*comp1*(1 << prescaler);
}
/********************************************************************************
 *
 ********************************************************************************/
ISR_DIRECT_DECLARE(timer1_handler)
{
	if (NRF_TIMER1->EVENTS_COMPARE[1] && (NRF_TIMER1->INTENSET) & (TIMER_INTENSET_COMPARE1_Msk))
	{
		NRF_TIMER1->TASKS_CLEAR = 1;
		NRF_TIMER1->EVENTS_COMPARE[1] = 0;
		tCount++;
		//LOG_INF("Timer count: >%d<", tCount);
	}
	ISR_DIRECT_PM();
	return 1;
}

/********************************************************************************
 * Set up and start Timer1
 ********************************************************************************/
static void timer1_init(void)
{
	IRQ_DIRECT_CONNECT(TIMER1_IRQn, IRQ_PRIO_LOWEST, timer1_handler, 0);
	irq_enable(TIMER1_IRQn);
	NRF_TIMER1->MODE = TIMER_MODE_MODE_Timer;
	NRF_TIMER1->TASKS_CLEAR = 1;
	NRF_TIMER1->PRESCALER = prescaler;
	NRF_TIMER1->BITMODE = TIMER_BITMODE_BITMODE_16Bit;
	NRF_TIMER1->CC[1] = comp1;
	NRF_TIMER1->SHORTS = TIMER_SHORTS_COMPARE1_CLEAR_Enabled << TIMER_SHORTS_COMPARE1_CLEAR_Pos;
	NRF_TIMER1->INTENSET = TIMER_INTENSET_COMPARE1_Enabled << TIMER_INTENSET_COMPARE1_Pos;
	set_conversion_factor();
	//printf("timer tick = %f us\n", countToUs);
	NRF_TIMER1->TASKS_START = 1;
}

/********************************************************************************
 *
 ********************************************************************************/
static void stop_timer(void)
{
	NRF_TIMER1->TASKS_STOP = 1;
}

/********************************************************************************
 * Stop Timer1
 ********************************************************************************/
static void start_timer(void)
{
	NRF_TIMER1->TASKS_START = 1;
}

/********************************************************************************
 * @} 
 ********************************************************************************/
static bool getDistanceTopSensor(float* dist)
{
	gpio_pin_set(gpio1_dev, TRIG_TOP_SENSOR, true);
	nrfx_systick_delay_us(20);
	gpio_pin_set(gpio1_dev, TRIG_TOP_SENSOR, false);
	nrfx_systick_delay_us(12);
	gpio_pin_set(gpio1_dev, TRIG_TOP_SENSOR, true);
	nrfx_systick_delay_us(20);
   
	//while(!gpio_pin_get(gpio1_dev, ECHO_TOP_SENSOR));
	// reset counter
	tCount = 0;
	// wait till Echo pin goes low
	//while(gpio_pin_get(gpio1_dev, ECHO_TOP_SENSOR));
	float duration = countToUs*tCount;
	float distance = duration*0.017;
	if(distance < 400.0) {
		// save
		*dist = distance;
		return true;
	}
	else {
		return false;
	}
}

/********************************************************************************
 * @} 
 ********************************************************************************/
static bool getDistanceLeftSensor(float* dist)
{
	gpio_pin_set(gpio1_dev, TRIG_LEFT_SENSOR, true);
	nrfx_systick_delay_us(20);
	gpio_pin_set(gpio1_dev, TRIG_LEFT_SENSOR, false);
	nrfx_systick_delay_us(12);
	gpio_pin_set(gpio1_dev, TRIG_LEFT_SENSOR, true);
	nrfx_systick_delay_us(20);
   
	//while(!gpio_pin_get(gpio1_dev, ECHO_LEFT_SENSOR));
	// reset counter
	tCount = 0;
	// wait till Echo pin goes low
	//while(gpio_pin_get(gpio1_dev, ECHO_LEFT_SENSOR));
	float duration = countToUs*tCount;
	float distance = duration*0.017;
	if(distance < 400.0) {
		// save
		*dist = distance;
		return true;
	}
	else {
		return false;
	}
}

/********************************************************************************
 * @} 
 ********************************************************************************/
static bool getDistanceFrontSensor(float* dist)
{
	gpio_pin_set(gpio1_dev, TRIG_FRONT_SENSOR, true);
	nrfx_systick_delay_us(20);
	gpio_pin_set(gpio1_dev, TRIG_FRONT_SENSOR, false);
	nrfx_systick_delay_us(12);
	gpio_pin_set(gpio1_dev, TRIG_FRONT_SENSOR, true);
	nrfx_systick_delay_us(20);
   
	//while(!gpio_pin_get(gpio1_dev, ECHO_FRONT_SENSOR));
	// reset counter
	tCount = 0;
	// wait till Echo pin goes low
	//while(gpio_pin_get(gpio1_dev, ECHO_FRONT_SENSOR));
	float duration = countToUs*tCount;
	float distance = duration*0.017;
	if(distance < 400.0) {
		// save
		*dist = distance;
		return true;
	}
	else {
		return false;
	}
}

/********************************************************************************
 * @} 
 ********************************************************************************/
static void getDistanceRightSensor(void)
{
	gpio_pin_set(gpio1_dev, TRIG_RIGHT_SENSOR, true);
	nrfx_systick_delay_us(20);
	gpio_pin_set(gpio1_dev, TRIG_RIGHT_SENSOR, false);
	nrfx_systick_delay_us(12);
	gpio_pin_set(gpio1_dev, TRIG_RIGHT_SENSOR, true);
	nrfx_systick_delay_us(20);
   
	while(!gpio_pin_get(gpio1_dev, ECHO_RIGHT_SENSOR));
	// reset counter
	tCount = 0;
	// wait till Echo pin goes low
	while(gpio_pin_get(gpio1_dev, ECHO_RIGHT_SENSOR));
	float duration = countToUs*tCount;
	float distance = duration*0.017;
	if(distance < 400.0) {
		// save
		dist_right = (uint32_t)distance;
		dist_right_sensor  = (uint32_t)dist_right;
		LOG_INF("dist_right_sensor = %d cm\n", dist_right_sensor);
		//return;
	}
	else {
		LOG_INF("dist_right_sensor > 400.0\n");
		//return;
	}
}

/********************************************************************************
 * 
 ********************************************************************************/
static void timer_top_sensor_expiry_function(struct k_timer *timer_id)
{
	dist_top_sensor = 0;
	//LOG_INF("timer_top_sensor_expiry_function\n");
	if(getDistanceTopSensor(&dist_top)) {
		dist_top_sensor  = (uint32_t)dist_top;	
		LOG_INF("dist_top_sensor = %d cm\n", dist_top_sensor);
	}		
	//k_timer_start(&timer_top_sensor, K_MSEC(100), K_NO_WAIT);
}

/********************************************************************************
 * 
 ********************************************************************************/
static void timer_1eft_sensor_expiry_function(struct k_timer *timer_id)
{
	dist_1eft_sensor = 0;	
	//LOG_INF("timer_1eft_sensor_expiry_function\n");
	if(getDistanceLeftSensor(&dist_1eft)) {
		dist_1eft_sensor  = (uint32_t)dist_1eft;	
		LOG_INF("dist_1eft_sensor = %d cm\n", dist_1eft_sensor);
	}		
	//k_timer_start(&timer_1eft_sensor, K_MSEC(100), K_NO_WAIT);
}

/********************************************************************************
 * 
 ********************************************************************************/
static void timer_front_sensor_expiry_function(struct k_timer *timer_id)
{
	dist_front_sensor = 0;
	//LOG_INF("timer_front_sensor_expiry_function\n");
	if(getDistanceLeftSensor(&dist_front)) {
		dist_front_sensor  = (uint32_t)dist_front;	
		LOG_INF("dist_front_sensor = %d cm\n", dist_front_sensor);
	}	
	//k_timer_start(&timer_front_sensor, K_MSEC(100), K_NO_WAIT);
}

/********************************************************************************
 * 
 ********************************************************************************/
static void timer_right_sensor_expiry_function(struct k_timer *timer_id)
{
	//dist_right_sensor = 0;
	//LOG_INF("timer_right_sensor_expiry_function\n");
	/*if(getDistanceRightSensor(&dist_right)) {
		dist_right_sensor  = (uint32_t)dist_right;	
		LOG_INF("dist_right_sensor = %d cm\n", dist_right_sensor);
	}*/
	//LOG_INF("k_work_submit_to_queue\n");
	//k_work_submit_to_queue(&my_work_q, getDistanceRightSensor);
	//k_timer_start(&timer_right_sensor, K_MSEC(100), K_NO_WAIT);
}

/********************************************************************************
 *
 ********************************************************************************/
static const struct bt_data ad[] = {
	BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
	BT_DATA(BT_DATA_NAME_COMPLETE, DEVICE_NAME, DEVICE_NAME_LEN),
};

static const struct bt_data sd[] = {
	BT_DATA_BYTES(BT_DATA_UUID128_ALL, BT_UUID_LBS_VAL),
};

/********************************************************************************
 *
 ********************************************************************************/
static void set_led_on(uint8_t led) {
	uint8_t err;
	err = gpio_pin_set(gpio0_dev, led, false);
	if (err) {
		return;
	}
}

/********************************************************************************
 *
 ********************************************************************************/
static void set_led_off(uint8_t led) {
	uint8_t err;
	err = gpio_pin_set(gpio0_dev, led, true);
	if (err) {
		return;
	}
}

/********************************************************************************
 *
 ********************************************************************************/
static void set_led(uint8_t led, int led_state) {
	uint8_t err;
	err = gpio_pin_set(gpio0_dev, led, led_state);
	if (err) {
		return;
	}
}

/********************************************************************************
 *
 ********************************************************************************/
static void connected(struct bt_conn *conn, uint8_t err)
{
	if (err) {
		printk("Connection failed (err %u)\n", err);
		return;
	}

	printk("Connected\n");

	set_led_on(CON_STATUS_LED);
}

/********************************************************************************
 *
 ********************************************************************************/
static void disconnected(struct bt_conn *conn, uint8_t reason)
{
	printk("Disconnected (reason %u)\n", reason);

	set_led_off(CON_STATUS_LED);
}

/********************************************************************************
 *
 ********************************************************************************/
static void security_changed(struct bt_conn *conn, bt_security_t level,
			     enum bt_security_err err)
{
	char addr[BT_ADDR_LE_STR_LEN];

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

	if (!err) {
		printk("Security changed: %s level %u\n", addr, level);
	} else {
		printk("Security failed: %s level %u err %d\n", addr, level,
			err);
	}
}

/********************************************************************************
 *
 ********************************************************************************/
BT_CONN_CB_DEFINE(conn_callbacks) = {
	.connected        = connected,
	.disconnected     = disconnected,
	.security_changed = security_changed,
};

static void auth_passkey_display(struct bt_conn *conn, unsigned int passkey)
{
	char addr[BT_ADDR_LE_STR_LEN];

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

	printk("Passkey for %s: %06u\n", addr, passkey);
}

/********************************************************************************
 *
 ********************************************************************************/
static void auth_cancel(struct bt_conn *conn)
{
	char addr[BT_ADDR_LE_STR_LEN];

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

	printk("Pairing cancelled: %s\n", addr);
}

/********************************************************************************
 *
 ********************************************************************************/
static struct bt_conn_auth_cb conn_auth_callbacks = {
	.passkey_display = auth_passkey_display,
	.cancel = auth_cancel,
};

/********************************************************************************
 *
 ********************************************************************************/
static void app_led_cb(bool led_state)
{
	set_led(USER_LED, led_state);
}

/********************************************************************************
 *
 ********************************************************************************/
static bool app_button_cb(void)
{
	return app_button_state;
}

/********************************************************************************
 *
 ********************************************************************************/
static struct bt_lbs_cb lbs_callbacs = {
	.led_cb    = app_led_cb,
	.button_cb = app_button_cb,
};

/********************************************************************************
 *
 ********************************************************************************/
static void button_changed(uint32_t button_state, uint32_t has_changed)
{
	//if (has_changed & USER_BUTTON) {
	if (has_changed) {
		//uint32_t user_button_state = button_state & USER_BUTTON;
		uint32_t user_button_state = button_state;

		bt_lbs_send_button_state(user_button_state);
		app_button_state = user_button_state ? true : false;
	}
}

/********************************************************************************
 *
 ********************************************************************************/
static int init_button(void)
{
	int err;

	//err = dk_buttons_init(button_changed);
	if (err) {
		printk("Cannot init buttons (err: %d)\n", err);
	}

	return err;
}

/********************************************************************************
 * void main(void)
 ********************************************************************************/
void main(void)
{
	int err;
	uint64_t i, j;
	int blink_status = 0;
	
	nrfx_systick_init();
	LOG_INF("\n\n\nA Smart white Cane Application started, version: %s\n", CONFIG_APP_VERSION);
	
	// 
	if (!device_is_ready(gpio0_dev)) {
		return;
	}
	// 
	if (!device_is_ready(gpio1_dev)) {
		return;
	}
	
	// set up HC-SR04 pins
	gpio_pin_configure(gpio1_dev, TRIG_TOP_SENSOR, GPIO_OUTPUT_INACTIVE);
	gpio_pin_configure(gpio1_dev, ECHO_TOP_SENSOR, GPIO_INPUT | GPIO_PULL_DOWN);	
	gpio_pin_set(gpio0_dev, TRIG_TOP_SENSOR, false);
	
	// set up HC-SR04 pins
	gpio_pin_configure(gpio1_dev, TRIG_LEFT_SENSOR, GPIO_OUTPUT_INACTIVE);
	gpio_pin_configure(gpio1_dev, ECHO_LEFT_SENSOR, GPIO_INPUT | GPIO_PULL_DOWN);	
	gpio_pin_set(gpio1_dev, TRIG_LEFT_SENSOR, false);
	
	// set up HC-SR04 pins
	gpio_pin_configure(gpio1_dev, TRIG_FRONT_SENSOR, GPIO_OUTPUT_INACTIVE);
	gpio_pin_configure(gpio1_dev, ECHO_FRONT_SENSOR, GPIO_INPUT | GPIO_PULL_DOWN);	
	gpio_pin_set(gpio1_dev, TRIG_FRONT_SENSOR, false);
	
	// set up HC-SR04 pins
	gpio_pin_configure(gpio1_dev, TRIG_RIGHT_SENSOR, GPIO_OUTPUT_INACTIVE);
	gpio_pin_configure(gpio1_dev, ECHO_RIGHT_SENSOR, GPIO_INPUT | GPIO_PULL_DOWN);	
	gpio_pin_set(gpio1_dev, TRIG_RIGHT_SENSOR, false);
	
	// set up HC-SR04 pins
	gpio_pin_configure(gpio0_dev, VIBRATION_MOTOR, GPIO_OUTPUT_INACTIVE);	
	gpio_pin_set(gpio0_dev, VIBRATION_MOTOR, false);
	
	// set up HC-SR04 pins
	gpio_pin_configure(gpio0_dev, BUZZER, GPIO_OUTPUT_INACTIVE);	
	gpio_pin_set(gpio0_dev, BUZZER, false);
	
	gpio_pin_configure(gpio0_dev, LED_ONE, GPIO_OUTPUT_ACTIVE);
	gpio_pin_configure(gpio0_dev, LED_TWO, GPIO_OUTPUT_ACTIVE);
	gpio_pin_configure(gpio0_dev, LED_THREE, GPIO_OUTPUT_ACTIVE);
	gpio_pin_configure(gpio0_dev, LED_FOUR, GPIO_OUTPUT_ACTIVE);
	
	k_msleep(SLEEP_TIME_MS * 2);

	//
	for(i = 0; i < 5; i++) {
		gpio_pin_set(gpio0_dev, LED_ONE, false);
		gpio_pin_set(gpio0_dev, LED_TWO, false);
		gpio_pin_set(gpio0_dev, LED_THREE, false);
		gpio_pin_set(gpio0_dev, LED_FOUR, false);
		//gpio_pin_set(gpio0_dev, BUZZER, true);
		gpio_pin_set(gpio0_dev, VIBRATION_MOTOR, true);
		for(j = 0; j < 1000; j++) {
			nrfx_systick_delay_us(1000);
		}
		//k_msleep(SLEEP_TIME_MS);	
		gpio_pin_set(gpio0_dev, LED_ONE, true);
		gpio_pin_set(gpio0_dev, LED_TWO, true);
		gpio_pin_set(gpio0_dev, LED_THREE, true);
		gpio_pin_set(gpio0_dev, LED_FOUR, true);
		//gpio_pin_set(gpio0_dev, BUZZER, false);
		gpio_pin_set(gpio0_dev, VIBRATION_MOTOR, false);
		for(j = 0; j < 1000; j++) {
			nrfx_systick_delay_us(1000);
		}
		//k_msleep(SLEEP_TIME_MS);
	}
	
	//
	LOG_INF("Init timers\n");
	//k_timer_init(&timer_top_sensor, timer_top_sensor_expiry_function, NULL);
	//k_timer_init(&timer_1eft_sensor, timer_1eft_sensor_expiry_function, NULL);
	//k_timer_init(&timer_front_sensor, timer_front_sensor_expiry_function, NULL);
	k_timer_init(&timer_right_sensor, timer_right_sensor_expiry_function, NULL);
	
	//
	k_work_queue_init(&my_work_q);
	
	k_work_queue_start(&my_work_q, my_stack_area,
						K_THREAD_STACK_SIZEOF(my_stack_area), MY_PRIORITY,
						NULL);

	//
	//k_timer_start(&timer_top_sensor, K_MSEC(1000), K_MSEC(100));
	//k_timer_start(&timer_1eft_sensor, K_MSEC(1000), K_MSEC(100));
	//k_timer_start(&timer_front_sensor, K_MSEC(1000), K_MSEC(100));
	k_timer_start(&timer_right_sensor, K_MSEC(1000), K_MSEC(100));
	
	//
	LOG_INF("Timer1\n");
	timer1_init();
	
	printk("Starting Bluetooth Peripheral LBS example\n");
	LOG_INF("starting...\n");
	
	//err = dk_leds_init();
	if (err) {
		printk("LEDs init failed (err %d)\n", err);
		return;
	}

	//err = init_button();
	if (err) {
		printk("Button init failed (err %d)\n", err);
		return;
	}

	err = bt_conn_auth_cb_register(&conn_auth_callbacks);
	if (err) {
		printk("Failed to register authorization callbacks.\n");
		return;
	}

	err = bt_enable(NULL);
	if (err) {
		printk("Bluetooth init failed (err %d)\n", err);
		return;
	}

	printk("Bluetooth initialized\n");
	
	err = bt_lbs_init(&lbs_callbacs);
	if (err) {
		printk("Failed to init LBS (err:%d)\n", err);
		return;
	}

	err = bt_le_adv_start(BT_LE_ADV_CONN, ad, ARRAY_SIZE(ad),
			      sd, ARRAY_SIZE(sd));
	if (err) {
		printk("Advertising failed to start (err %d)\n", err);
		return;
	}

	printk("Advertising successfully started\n");
	
	for (;;) {
		set_led(RUN_STATUS_LED, (++blink_status) % 2);
		k_sleep(K_MSEC(RUN_LED_BLINK_INTERVAL));
		getDistanceRightSensor();
	}
}
