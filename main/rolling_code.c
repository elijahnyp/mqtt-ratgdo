#include "common.h"
#include "rolling_code.h"
#include <stdio.h>
#include "esp_spiffs.h"
#include "nvs_flash.h"
#include "esp_log.h"


// #include "Helpers.h"
#define TAG "RollingCode"

bool nvs_initiated = false;
esp_err_t readCounterFromFlash(const char *type, unsigned int *counter){
	*counter = 0;
	return ESP_OK;
	// esp_err_t err;
	// if(!nvs_initiated){
	// 	err = nvs_flash_init();
	// 	if(err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
	// 		ESP_ERROR_CHECK(nvs_flash_erase());
	// 		err = nvs_flash_init();
	// 		nvs_initiated =  true;
	// 	}
	// 	if(err != ESP_OK){
	// 		return ESP_ERR_INVALID_STATE;
	// 	}
	// }
	// nvs_handle_t handle;
	// err = nvs_open(type, NVS_READONLY, &handle);
	// switch (err){
	// 	case ESP_OK:
	// 		break;
	// 	case ESP_ERR_NVS_NOT_FOUND:
	// 		err = nvs_open(type,NVS_READWRITE,&handle);
	// 		if(err != ESP_OK){
	// 			return ESP_ERR_INVALID_STATE;
	// 		}
	// 		err = nvs_set_u32(handle,type,0);
	// 		if(err != ESP_OK){
	// 			ESP_LOGE(TAG,"Error setting %s in nvram, defaulting to 0",type);
	// 			counter = 0;
	// 			return ESP_ERR_INVALID_STATE;
	// 		} else {
	// 			counter = 0;
	// 			return ESP_OK;
	// 		}
	// 		break;
	// 	default:
	// 		ESP_LOGE(TAG,"error opening NVS handle %s",type);
	// }
	// nvs_get_u32(handle,type,(uint32_t*)counter);
	// nvs_commit(handle);
	// nvs_close(handle);
	// return ESP_OK;
}

esp_err_t writeCounterToFlash(const char *type, unsigned int *counter){
	return ESP_OK;
	// esp_err_t err;
	// if(!nvs_initiated){
	// 	err = nvs_flash_init();
	// 	if(err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
	// 		ESP_ERROR_CHECK(nvs_flash_erase());
	// 		err = nvs_flash_init();
	// 		nvs_initiated =  true;
	// 	}
	// 	if(err != ESP_OK){
	// 		return ESP_ERR_INVALID_STATE;
	// 	}
	// }
	// nvs_handle_t handle;
	// err = nvs_open(type, NVS_READWRITE, &handle);
	// if(err != ESP_OK){
	// 	ESP_LOGE(TAG,"error opening NVS handle to write %s",type);
	// }
	// err = nvs_set_u32(handle,type,(uint32_t)counter);
	// nvs_commit(handle);
	// nvs_close(handle);
	// if(err != ESP_OK){
	// 	return ESP_ERR_NO_MEM;
	// } else {
	// 	return ESP_OK;
	// }
}

esp_err_t deleteCounter(const char* type){
	return ESP_OK;
	// esp_err_t err;
	// if(!nvs_initiated){
	// 	err = nvs_flash_init();
	// 	if(err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
	// 		ESP_ERROR_CHECK(nvs_flash_erase());
	// 		err = nvs_flash_init();
	// 		nvs_initiated =  true;
	// 	}
	// 	if(err != ESP_OK){
	// 		return ESP_ERR_INVALID_STATE;
	// 	}
	// }
	// nvs_handle_t handle;
	// err = nvs_open(type,NVS_READWRITE,&handle);
	// if(err == ESP_ERR_NVS_NOT_FOUND){
	// 	return ESP_OK;
	// }
	// err = nvs_erase_key(handle,type);
	// if(err != ESP_OK){
	// 	return ESP_ERR_INVALID_STATE;
	// }
	// nvs_commit(handle);
	// nvs_close(handle);
	// return ESP_OK;
}

// esp_err_t readCounterFromFlash(const char *type, unsigned int *counter){
// 	ESP_LOGI(TAG,"READING FILE %s",type);
// 	// File file = LittleFS.open(type, "r");
// 	if(! esp_spiffs_mounted(NULL)){
// 		esp_vfs_spiffs_conf_t conf = {
// 			.base_path = "/spiffs",
// 			.partition_label = NULL,
// 			.max_files = 20,
// 			.format_if_mount_failed = true
// 		};
//         esp_err_t ret = esp_vfs_spiffs_register(&conf);
// 		if(ret != ESP_OK){
//             if(ret == ESP_FAIL){
//                 ESP_LOGE(TAG,"Failed to mount or format SPIFFS");
//             } else if (ret == ESP_ERR_NOT_FOUND){
//                 ESP_LOGE(TAG,"Failed to find SPIFFS partition");
//             } else {
//                 ESP_LOGE(TAG,"Failed to initialize SPIFFS (%s)",esp_err_to_name(ret));
//             }
//             return ESP_ERR_NO_MEM;
//         }
// 	}
// 	char fname[32];
// 	strcpy(fname,"/spiffs/");
// 	strcat(fname,type);
// 	FILE* file = fopen(fname,"rb");

// 	//Check if the file exists
// 	if(!file){
// 		// Serial.print(type);
// 		// Serial.println(" doesn't exist. creating...");
// 		printf(type);
// 		printf(" doesn't exist. creating...\n");

// 		writeCounterToFlash(type,counter);
// 		// file = fopen(fname,"rb");
// 		if(!file){
// 			return ESP_ERR_INVALID_STATE;
// 		}
// 		return ESP_OK;
// 	}

// 	fread(counter,sizeof(unsigned int),1,file);

// 	//Close the file
// 	fclose(file);
// 	return ESP_OK;
// }

// esp_err_t writeCounterToFlash(const char *type, unsigned int *counter){
// 	//Open the file 
// 	// File file = LittleFS.open(type, "w");
// 	if(! esp_spiffs_mounted(NULL)){
// 		esp_vfs_spiffs_conf_t conf = {
// 			.base_path = "/spiffs",
// 			.partition_label = NULL,
// 			.max_files = 20,
// 			.format_if_mount_failed = true
// 		};
//         esp_err_t ret = esp_vfs_spiffs_register(&conf);
// 		if(ret != ESP_OK){
//             if(ret == ESP_FAIL){
//                 ESP_LOGE(TAG,"Failed to mount or format SPIFFS");
//             } else if (ret == ESP_ERR_NOT_FOUND){
//                 ESP_LOGE(TAG,"Failed to find SPIFFS partition");
//             } else {
//                 ESP_LOGE(TAG,"Failed to initialize SPIFFS (%s)",esp_err_to_name(ret));
//             }
//             return ESP_ERR_NO_MEM;
//         }
// 	}
// 	char fname[32];
// 	strcpy(fname,"/spiffs/");
// 	strcat(fname,type);
// 	FILE* file = fopen(fname,"wb");
// 	//Write to the file
// 	// fprintf(file,"%d",*counter);
// 	ESP_LOGI(TAG,"%s: %u",type,*counter);
// 	// if(file != NULL){
// 	// 	ESP_LOGI(TAG,"WORKING");
// 	// }
// 	fwrite(counter,sizeof(unsigned int),1,file);
// 	// delay(1);
// 	//Close the file
// 	fclose(file);
	
// 	// Serial.print(type);
// 	// Serial.println(" write successful");
// 	printf(type);
// 	printf(" write successful\n");
// 	return ESP_OK;
// }


// esp_err_t deleteCounter(const char* type){
// 	if(! esp_spiffs_mounted(NULL)){
// 		esp_vfs_spiffs_conf_t conf = {
// 			.base_path = "/spiffs",
// 			.partition_label = NULL,
// 			.max_files = 20,
// 			.format_if_mount_failed = true
// 		};
//         esp_err_t ret = esp_vfs_spiffs_register(&conf);
// 		if(ret != ESP_OK){
//             if(ret == ESP_FAIL){
//                 ESP_LOGE(TAG,"Failed to mount or format SPIFFS");
//             } else if (ret == ESP_ERR_NOT_FOUND){
//                 ESP_LOGE(TAG,"Failed to find SPIFFS partition");
//             } else {
//                 ESP_LOGE(TAG,"Failed to initialize SPIFFS (%s)",esp_err_to_name(ret));
//             }
//             return ESP_ERR_NO_MEM;
//         }
// 	}
// 	char fname[32];
// 	strcpy(fname,"/spiffs/");
// 	strcat(fname,type);
// 	remove(fname);
// 	return ESP_OK;
// }

void readRollingCode(uint8_t rxSP2RollingCode[SECPLUS2_CODE_LEN], uint8_t *door, uint8_t *light, uint8_t *lock, uint8_t *motion, uint8_t *obstruction){
	uint32_t rolling = 0;
	uint64_t fixed = 0;
	uint32_t data = 0;

	uint16_t cmd = 0;
	uint8_t nibble = 0;
	// uint8_t byte1 = 0;
	uint8_t byte2 = 0;

	decode_wireline(rxSP2RollingCode, &rolling, &fixed, &data);

	cmd = ((fixed >> 24) & 0xf00) | (data & 0xff);

	nibble = (data >> 8) & 0xf;
	// byte1 = (data >> 16) & 0xff;
	byte2 = (data >> 24) & 0xff;

	printRollingCode(rxSP2RollingCode);

	if(cmd == 0x81){
		*door = nibble;
		*light = (byte2 >> 1) & 1;
		*lock = byte2 & 1;
		*motion = 0; // when the status message is read, reset motion state to 0|clear
		// obstruction = (byte1 >> 6) & 1; // unreliable due to the time it takes to register an obstruction

		// Serial.print(" | STATUS:");
		// Serial.print(" door:");
		// Serial.print(nibble);
		// Serial.print(" light:");
		// Serial.print((byte2 >> 1) & 1);
		// Serial.print(" lock:");
		// Serial.print((byte2 & 1));
		// Serial.print(" obs:");
		// Serial.print((byte1 >> 6) & 1);

	}else if(cmd == 0x281){
		*light ^= 1; // toggle bit

		// Serial.print(" | LIGHT:");
		// Serial.print(light);
	}else if(cmd == 0x84){
	}else if(cmd == 0x285){
		*motion = 1; // toggle bit
		// Serial.print(" | MOTION:");
		// Serial.print(motion);
	}

	// Serial.println("");
}

void getRollingCode(const char *command){
	// Serial.print("rolling code for ");
	// Serial.print(idCode);
	// Serial.print(" ");
	// Serial.print(rollingCodeCounter);
	// Serial.print("|");
	// Serial.print(command);
	// Serial.print(" : ");

	uint64_t id = idCode;
	uint64_t fixed = 0;
	uint32_t data = 0;

	if(strcmp(command,"reboot1") == 0){
		fixed = 0x400000000;
		data = 0x0000618b;
	}else if(strcmp(command,"reboot2") == 0){
		fixed = 0;
		data = 0x01009080;
	}else if(strcmp(command,"reboot3") == 0){
		fixed = 0;
		data = 0x0000b1a0;
	}else if(strcmp(command,"reboot4") == 0){
		fixed = 0;
		data = 0x01009080;
	}else if(strcmp(command,"reboot5") == 0){
		fixed = 0x300000000;
		data = 0x00008092;
	}else if(strcmp(command,"reboot6") == 0){
		fixed = 0x300000000;
		data = 0x00008092;
	}else if(strcmp(command,"door1") == 0){
		fixed = 0x200000000;
		data = 0x01018280;
	}else if(strcmp(command,"door2") == 0){
		fixed = 0x200000000;
		data = 0x01009280;
	}else if(strcmp(command,"light") == 0){
		fixed = 0x200000000;
		data = 0x00009281;
	}else if(strcmp(command,"lock") == 0){
		fixed = 0x0100000000;
		data = 0x0000728c;
	}else{
		// Serial.println("ERROR: Invalid command");
		printf("ERROR: Invalid command\n");
		return;
	}

	fixed = fixed | id;
	// ESP_LOGI(TAG,"MARK^");
	// printRollingCode(txSP2RollingCode);
	int8_t err = encode_wireline(rollingCodeCounter, fixed, data, txSP2RollingCode);
	if(err < 0){
		ESP_LOGE(TAG,"error encoding wireline: %i",err);
	}

	printRollingCode(txSP2RollingCode);
	// ESP_LOGI(TAG,"MARKv");
	// printf("\n");
	// Serial.println("");

	if(strcmp(command,"door1") != 0){ // door2 is created with same counter and should always be called after door1
		rollingCodeCounter = (rollingCodeCounter + 1) & 0xfffffff;
	}
	return;
}

void printRollingCode(uint8_t code[SECPLUS2_CODE_LEN]){
	char str[150] = {0};
	char t1[32];
	// str[0] = '\0';
	for(int i = 0; i < SECPLUS2_CODE_LEN; i++){

		// if(code[i] <= 0x0f) sprintf(t1,"0");
		// ESP_LOGI(TAG,"CCCC %i",i);
		sprintf(t1,"%ix%02x ",i,code[i]);
		// ESP_LOGI(TAG,"DDDD %s", t1);
		strcat(str,t1);
		// if(code[i] <= 0x0f) ESP_LOGI(TAG,"0");
		// ESP_LOGI(TAG,"%x",code[i]);
	}
	ESP_LOGI(TAG,"Rolling Code: %s",str);
}