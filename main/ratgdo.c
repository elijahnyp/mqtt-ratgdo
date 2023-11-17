/************************************
 * Rage
 * Against
 * The
 * Garage
 * Door
 * Opener
 *
 * Copyright (C) 2022  Paul Wieland
 *
 * GNU GENERAL PUBLIC LICENSE
 ************************************/

#include "common.h"
#include "ratgdo.h"

#include "wangwood_bootstrap.h"

#include "esp_log.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "esp_err.h"
#include "esp_timer.h"
#include "esp_random.h"
#include "hal/uart_hal.h"

#include <stdio.h>
#include <string.h>
#include <unistd.h>


#define TAG "RATGDO"

#define HIGH 1
#define LOW 0

const uart_port_t uart_num = UART_NUM_2;


#ifdef __cplusplus
extern "C" {
#endif
void mqtt_handler(char* topic, uint8_t* payload, unsigned int length);

#define DOOR_BIT 0
#define LIGHT_BIT 1
#define LOCK_BIT 2
#define OBSTRUCTION_BIT 3

ww_cust_status_t ratgdo_status = {
		.state = NULL,
		.num_entries = 4,
		.entries[0] = {
			.on = "Closed",
			.off = "Open",
			.name = "Door",
			.bit = (1<<0)
		},
		.entries[1] = {
			.on = "Light On",
			.off = "Light Off",
			.name = "Light",
			.bit = (1<<1)
		},
		.entries[2] = {
			.on = "Locked",
			.off = "Unlocked",
			.name = "Lock",
			.bit = (1<<2)
		},
		.entries[3] = {
			.on = "Clear",
			.off = "Obstructed",
			.name = "Obstructed",
			.bit = (1<<3)
		}
	};

void app_main(void){
	wangwood_startup(&mqtt_handler);
	ratgdo_status.state = xEventGroupCreate();
	wangwood_register_custom_status(&ratgdo_status);
	setup();
	while(true){
		loop();
		vTaskDelay(10);
	}
}

void mqtt_handler(char* topic, uint8_t* payload, unsigned int length){
//   ESP_LOGI(TAG,"MQTT Message Received");
//   ESP_LOGI(TAG,"Topic: %s", topic);
//   ESP_LOGI(TAG,"Length: %i", length);
//   char* payload_c;
//   payload_c = (char*)malloc(sizeof(char) * length);
//   for(int i=0; i<length; i++){
//     payload_c[i] = (char)payload[i];
//   }
  // ESP_LOGI(TAG,"Message: %*.s, parsed: %*.s.", length, (char*)payload, length, payload_c);

  if(strncmp((char*)payload,"open", length) == 0){
	ESP_LOGI(TAG,"OPEN");
	openDoor();
  }
  else if(strncmp((char*)payload,"close",length) == 0){
	ESP_LOGI(TAG,"CLOSE");
	closeDoor();
  } else if(strncmp((char*)payload,"lighton",length) == 0){
	ESP_LOGI(TAG,"LIGHT ON");
	lightOn();
  } else if(strncmp((char*)payload,"lightoff",length) == 0){
	ESP_LOGI(TAG,"LIGHT OFF");
	lightOff();
  } else if(strncmp((char*)payload,"lock",length) == 0){
	ESP_LOGI(TAG,"LOCK");
	lock();
  } else if(strncmp((char*)payload,"unlock",length) == 0){
	ESP_LOGI(TAG,"UNLOCK");
	unlock();
  } else if(strncmp((char*)payload,"sync",length) == 0){
	sync();
  } else if(strncmp((char*)payload,"reset",length) == 0){
	deleteCounter("idCode");
	deleteCounter("rolling");

  }
}

#ifdef __cplusplus
}
#endif



void setup(){	
	gpio_reset_pin(INPUT_OBST);
	gpio_set_direction(INPUT_OBST,GPIO_MODE_INPUT);
	gpio_set_intr_type(INPUT_OBST,GPIO_INTR_ANYEDGE);
	gpio_isr_handler_add(INPUT_OBST,isrObstruction, NULL);
	
	// clear rolling code arrays
	memset(txSP2RollingCode,0,sizeof(txSP2RollingCode));
	memset(rxSP2RollingCode,0,sizeof(txSP2RollingCode));

	uart_config_t uart_config = {
		.baud_rate = 9600,
		.data_bits = UART_DATA_8_BITS,
		.parity = UART_PARITY_DISABLE,
		.stop_bits = UART_STOP_BITS_1,
		.flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
	};
	esp_err_t err = uart_param_config(UART_NUM_2,&uart_config);
	if(err != ESP_OK){
		ESP_LOGE(TAG,"ERROR CONFIGURING SERIAL");
	}
	err = uart_set_pin(uart_num, OUTPUT_GDO, INPUT_GDO, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
	uart_set_line_inverse(uart_num, UART_SIGNAL_RXD_INV|UART_SIGNAL_TXD_INV);
	if(err != ESP_OK){
		ESP_LOGE(TAG,"ERROR SETTING PINS FOR SERIAL");
	}
	const int uart_buffer_size = (256); //maybe adjust down?
	// QueueHandle_t uart_queue;
	// err = uart_driver_install(uart_num, uart_buffer_size, uart_buffer_size, 10, &uart_queue, 0);
	err = uart_driver_install(uart_num, uart_buffer_size, uart_buffer_size, 10, NULL, 0);
	// err = uart_driver_install(uart_num, uart_buffer_size, uart_buffer_size, 10, NULL, ESP_INTR_FLAG_IRAM);
	if(err != ESP_OK){
		ESP_LOGE(TAG,"ERROR INSTALLING UART DRIVER");
	}

	ESP_LOGI(TAG,"Using security+ 2.0");

	readCounterFromFlash("idCode", &idCode);
	if(idCode >= 65535 || idCode == 0){
		ESP_LOGI(TAG,"Generating random id");
		short unsigned int r = esp_random();
		// idCode = random(0x1,0xFFFF);
		idCode = r;
		writeCounterToFlash("idCode", &idCode);
	}
	ESP_LOGI(TAG,"idCode: %u", idCode);
	readCounterFromFlash("rolling", &rollingCodeCounter);

	vTaskDelay(pdMS_TO_TICKS(1000)); //to let spiffs sort itself out - NOT GREAT HONESTLY, complete work-around

	ESP_LOGI(TAG,"Syncing rolling code counter after reboot...");
	sync(); // send reboot/sync to the opener on startup

	//light status is screwey, so turn on the lights then sync again
	lightOn();
	vTaskDelay(pdMS_TO_TICKS(1000));
	sync();
	vTaskDelay(pdMS_TO_TICKS(1000));
	lightOff();

	vTaskDelay(pdMS_TO_TICKS(1000));
	sendDoorStatus();
	sendLightStatus();
	sendLockStatus();
	sendObstructionStatus();
}


/*************************** MAIN LOOP ***************************/
void loop(){
	obstructionLoop();
	gdoStateLoop();
	statusUpdateLoop();
}

void gdoStateLoop(){
	// blink(false);
	// if(!swSerial.available()) return;
	int buflen = 0;
	uart_get_buffered_data_len(uart_num, (size_t*)&buflen);
	if(buflen == 0) return;
	// uint8_t serData = swSerial.read(); //serial
	uint8_t serData;
	int bytesRead = uart_read_bytes(uart_num, &serData, sizeof(uint8_t), 100);
	if(bytesRead > 0){
		// ESP_LOGI(TAG,"read data %i character of serial data",bytesRead);
	}
	static uint32_t msgStart;
	static bool reading = false;
	static uint16_t byteCount = 0;

	if(!reading){
		// ESP_LOGI(TAG,"looking for message start");
		// shift serial uint8_tonto msg start
		msgStart <<= 8;
		msgStart |= serData;

		// if(controlProtocol == "secplus2"){
			// truncate to 3 bytes
			msgStart &= 0x00FFFFFF;

			// if we are at the start of a message, capture the next 16 bytes
			if(msgStart == 0x550100){
				// ESP_LOGI(TAG,"message start detected");
				byteCount = 3;
				rxSP2RollingCode[0] = 0x55;
				rxSP2RollingCode[1] = 0x01;
				rxSP2RollingCode[2] = 0x00;

				reading = true;
				return;
			}
	}

	if(reading){
		rxSP2RollingCode[byteCount] = serData;
		byteCount++;

		if(byteCount == SECPLUS2_CODE_LEN){
			// ESP_LOGI(TAG,"message compiled");
			reading = false;
			msgStart = 0;
			byteCount = 0;

			readRollingCode(rxSP2RollingCode, &doorState, &lightState, &lockState, &motionState, &obstructionState);
		}
	}
}


/*************************** OBSTRUCTION DETECTION ***************************/
// void ESP_INTR_FLAG_IRAM isrObstruction(){
void isrObstruction(){
	if(gpio_get_level(INPUT_OBST)){
		lastObstructionHigh = esp_timer_get_time() / 1000;
	}else{
		obstructionSensorDetected = true;
		obstructionLowCount++;
	}
	
}

void obstructionLoop(){
	if(!obstructionSensorDetected) return;
	long currentMillis = esp_timer_get_time() / 1000;
	static unsigned long lastMillis = 0;

	// the obstruction sensor has 3 states: clear (HIGH with LOW pulse every 7ms), obstructed (HIGH), asleep (LOW)
	// the transitions between awake and asleep are tricky because the voltage drops slowly when falling asleep
	// and is high without pulses when waking up

	// If at least 3 low pulses are counted within 50ms, the door is awake, not obstructed and we don't have to check anything else

	// Every 50ms
	if(currentMillis - lastMillis > 50){
		// check to see if we got between 3 and 8 low pulses on the line
		if(obstructionLowCount >= 3 && obstructionLowCount <= 8){
			obstructionState = 1;

		// if there have been no pulses the line is steady high or low			
		}else if(obstructionLowCount == 0){
			// if the line is high and the last high pulse was more than 70ms ago, then there is an obstruction present
			if(gpio_get_level(INPUT_OBST) && currentMillis - lastObstructionHigh > 70){
				obstructionState = 0;
			}else{
				// asleep
			}
		}

		lastMillis = currentMillis;
		obstructionLowCount = 0;
	}
}

/*************************** STATUS UPDATES ***************************/
void statusUpdateLoop(){
	// initialize to unknown
	static uint8_t previousDoorState = 0;
	static uint8_t previousLightState = 2;
	static uint8_t previousLockState = 2;
	static uint8_t previousObstructionState = 2;

	if(doorState != previousDoorState) sendDoorStatus();
	if(lightState != previousLightState) sendLightStatus();
	if(lockState != previousLockState) sendLockStatus();
	if(obstructionState != previousObstructionState) sendObstructionStatus();

	if(motionState == 1){
		sendMotionStatus();
		motionState = 0;
	}

	previousDoorState = doorState;
	previousLightState = lightState;
	previousLockState = lockState;
	previousObstructionState = obstructionState;
}

void sendDoorStatus(){
	ESP_LOGI(TAG,"Door state: %s",doorStates[doorState]);
	send_mqtt_event(doorStates[doorState]);
	send_mqtt_status_topic("door",doorStates[doorState]);
	if(doorState != DOOR_OPEN){
		xEventGroupSetBits(ratgdo_status.state,ratgdo_status.entries[DOOR_BIT].bit);
	} else {
		xEventGroupClearBits(ratgdo_status.state,ratgdo_status.entries[DOOR_BIT].bit);
	}
	// if(doorState == 1) digitalWrite(STATUS_DOOR, HIGH); // Open
	// if(doorState == 2) digitalWrite(STATUS_DOOR, LOW); // Closed
	
	// if(isConfigFileOk){
	// 	bootstrapManager.publish(doorStatusTopic.c_str(), doorStates[doorState].c_str(), true);
	// }
}

void sendLightStatus(){
	ESP_LOGI(TAG,"Light state: %s",lightStates[lightState]);
	send_mqtt_event(lightStates[lightState]);
	send_mqtt_status_topic("light",lightStates[lightState]);
	if(lightState != LIGHT_OFF){
		xEventGroupSetBits(ratgdo_status.state,ratgdo_status.entries[LIGHT_BIT].bit);
	} else {
		xEventGroupClearBits(ratgdo_status.state,ratgdo_status.entries[LIGHT_BIT].bit);
	}
	// if(isConfigFileOk){
	// 	bootstrapManager.publish(lightStatusTopic.c_str(), lightStates[lightState].c_str(), true);
	// }
}

void sendLockStatus(){
	ESP_LOGI(TAG,"Lock state: %s",lockStates[lockState]);
	send_mqtt_event(lockStates[lockState]);
	send_mqtt_status_topic("lock",lockStates[lockState]);
	if(lockState != LOCK_UNLOCKED){
		xEventGroupSetBits(ratgdo_status.state,ratgdo_status.entries[LOCK_BIT].bit);
	} else {
		xEventGroupClearBits(ratgdo_status.state,ratgdo_status.entries[LOCK_BIT].bit);
	}
	// if(isConfigFileOk){
	// 	bootstrapManager.publish(lockStatusTopic.c_str(), lockStates[lockState].c_str(), true);
	// }
}

void sendMotionStatus(){
	ESP_LOGI(TAG,"Motion state: %s",motionStates[motionState]);
	send_mqtt_event(motionStates[motionState]);
	// if(isConfigFileOk){
	// 	bootstrapManager.publish(motionStatusTopic.c_str(), motionStates[motionState].c_str(), true);
	// }

	motionState = 0; // reset motion state

	// query to sync light state
	vTaskDelay(pdMS_TO_TICKS(100));
	getRollingCode("reboot2");
	transmit(txSP2RollingCode,SECPLUS2_CODE_LEN);

}

void sendObstructionStatus(){
	ESP_LOGI(TAG,"Obstruction status: %s",obstructionStates[obstructionState]);
	send_mqtt_event(obstructionStates[obstructionState]);
	send_mqtt_status_topic("obstruction",obstructionStates[obstructionState]);
	if(obstructionState != OBST_OBSTRUCTED){
		xEventGroupSetBits(ratgdo_status.state,ratgdo_status.entries[OBSTRUCTION_BIT].bit);
	} else {
		xEventGroupClearBits(ratgdo_status.state,ratgdo_status.entries[OBSTRUCTION_BIT].bit);
	}
	// if(obstructionState == 0) digitalWrite(STATUS_OBST,HIGH); // obstructed
	// if(obstructionState == 1) digitalWrite(STATUS_OBST,LOW); // clear

	// if(isConfigFileOk){
	// 	bootstrapManager.publish(obstructionStatusTopic.c_str(), obstructionStates[obstructionState].c_str(), true);
	// }
}


void transmit(uint8_t* payload, unsigned int length){
	esp_err_t err;

	// lead transmissions with a break - not supported by uart driver directly, so we do a little flip.  This isn't good practice, but it works for now.
	uart_set_line_inverse(uart_num, UART_SIGNAL_INV_DISABLE); //pulse high (but makes for serial low due to transistor)
	usleep(1305);
	uart_set_line_inverse(uart_num, UART_SIGNAL_RXD_INV|UART_SIGNAL_TXD_INV); //put back inversion
	usleep(1260);

	ESP_LOGD(TAG,"TEST: WRITING %u TO UART",length);
	int status = uart_write_bytes(uart_num, payload, length);
	ESP_LOGD(TAG,"TEST: WAITING FOR %i TO TRANSMIT",status);
	err = uart_wait_tx_done(uart_num, pdMS_TO_TICKS(50));
	if(err != ESP_OK){
		ESP_LOGE(TAG,"UART TIMEOUT TRANSMITTING");
	}
}

void pullLow(){
	gpio_set_level(OUTPUT_GDO, HIGH);
	vTaskDelay(pdMS_TO_TICKS(500));
	gpio_set_level(OUTPUT_GDO, LOW);
}

void sync(){

	getRollingCode("reboot1");
	transmit(txSP2RollingCode,SECPLUS2_CODE_LEN);
	vTaskDelay(pdMS_TO_TICKS(100));

	getRollingCode("reboot2");
	transmit(txSP2RollingCode,SECPLUS2_CODE_LEN);
	vTaskDelay(pdMS_TO_TICKS(100));

	getRollingCode("reboot3");
	transmit(txSP2RollingCode,SECPLUS2_CODE_LEN);
	vTaskDelay(pdMS_TO_TICKS(100));

	getRollingCode("reboot4");
	transmit(txSP2RollingCode,SECPLUS2_CODE_LEN);
	vTaskDelay(pdMS_TO_TICKS(100));

	getRollingCode("reboot5");
	transmit(txSP2RollingCode,SECPLUS2_CODE_LEN);
	vTaskDelay(pdMS_TO_TICKS(100));

	getRollingCode("reboot6");
	transmit(txSP2RollingCode,SECPLUS2_CODE_LEN);
	vTaskDelay(pdMS_TO_TICKS(100));

	// vTaskDelay(pdMS_TO_TICKS(1000)); //complete work around NOT GREAT to let spiffs sort itself out

	writeCounterToFlash("rolling",&rollingCodeCounter);
}

// Door functions
void openDoor(){
	ESP_LOGI(TAG,"IN OPEN DOOR");
	if(doorState == DOOR_OPEN || doorState == DOOR_OPENING){
		ESP_LOGI(TAG,"The door is already %s",doorStates[(int)doorState]);
		return;
	}
	ESP_LOGI(TAG,"Opening door");

	//if we're cancelling movement, we need to 'toggle twice'
	if(doorState == DOOR_CLOSING){
		vTaskDelay(pdMS_TO_TICKS(1000));
		toggleDoor();
	}

	toggleDoor();
}

void closeDoor(){
	if(doorState == DOOR_CLOSED || doorState == DOOR_CLOSING){
		ESP_LOGI(TAG,"The door is already %s",doorStates[doorState]);
		return;
	}
	ESP_LOGI(TAG,"Closing door");
	
	//if we're cancelling movement, we need to 'toggle twice'
	if(doorState == DOOR_OPENING){
		vTaskDelay(pdMS_TO_TICKS(1000));
		toggleDoor();
	}
	
	toggleDoor();
}

void stopDoor(){
	if(doorState == DOOR_OPENING || doorState == DOOR_CLOSING){
		toggleDoor();
	}else{
		ESP_LOGI(TAG,"The door is not moving.");
	}
}

void toggleDoor(){
	ESP_LOGI(TAG,"send door1");
	getRollingCode("door1");
	transmit(txSP2RollingCode, SECPLUS2_CODE_LEN);//serial

	vTaskDelay(pdMS_TO_TICKS(40));

	ESP_LOGI(TAG,"send door2");
	getRollingCode("door2");
	transmit(txSP2RollingCode, SECPLUS2_CODE_LEN); //serial

	vTaskDelay(pdMS_TO_TICKS(1000)); //complete work around to let SPIFFS sort itself out - NOT GREAT

	writeCounterToFlash("rolling",&rollingCodeCounter);
}

// Light functions
void lightOn(){
	if(lightState == LIGHT_ON){
		ESP_LOGI(TAG,"Light already on");
	}else{
		toggleLight();
	}
}

void lightOff(){
	if(lightState == LIGHT_OFF){
		ESP_LOGI(TAG,"Light already off");
	}else{
		toggleLight();
	}
}

void toggleLight(){
	getRollingCode("light");
	transmit(txSP2RollingCode,SECPLUS2_CODE_LEN); //serial
	vTaskDelay(pdMS_TO_TICKS(1000)); //complete work around to let SPIFFS sort itself out - NOT GREAT
	writeCounterToFlash("rolling",&rollingCodeCounter);
}

// Lock functions
void lock(){
	if(lockState == LOCK_LOCKED){
		ESP_LOGI(TAG,"Door already locked");
	}else{
		toggleLock();
	}
}

void unlock(){
	if(lockState == LOCK_UNLOCKED){
		ESP_LOGI(TAG,"Door already unlocked");
	}else{
		toggleLock();
	}
}

void toggleLock(){
	getRollingCode("lock");
	transmit(txSP2RollingCode,SECPLUS2_CODE_LEN); //serial
	vTaskDelay(pdMS_TO_TICKS(1000)); //complete work around to let SPIFFS sort itself out - NOT GREAT
	writeCounterToFlash("rolling",&rollingCodeCounter);
}