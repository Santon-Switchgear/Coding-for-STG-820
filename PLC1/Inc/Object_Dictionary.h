#ifndef __DICTIONARY_H
#define __DICTIONARY_H
  /* Includes ------------------------------------------------------------------*/

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Object Dictionary define ------------------------------------------------------------*/
#define CAN_controller_type_index 0x1000
#define CAN_controller_type_access 'R'  //READ ONLY
#define CAN_controller_type_config 'S'  //SDO config not mapped for PDO
#define CAN_controller_type_value 'MC' 

#define CAN_consumer_heartbeat_time_index 0x1016 
#define CAN_consumer_heartbeat_time_access 'R'
#define CAN_consumer_heartbeat_time_config 'S'
#define CAN_consumer_heartbeat_time_value 50 //minimal acceptance rate for recieving heartbeat messages

#define CAN_PDO1_rx_comm_index 0x1400 //PDO1: receive communication parameter.RPDO (link)
#define CAN_PDO1_rx_comm_access 'R+W'
#define CAN_PDO1_rx_comm_config 'S'
#define CAN_PDO1_rx_comm_value 0



#define CAN_error_register_index 0x1001
#define CAN_error_register_access 'R'
#define CAN_error_register_config 'S'
#define CAN_error_register_value 0

#define CAN_communication_cycle_period_index 0x1006
#define CAN_communication_cycle_period_access 'R'
#define CAN_communication_cycle_period_config 'S'
#define CAN_communication_cycle_period_value 1 //Value in ms , Spacing between signals

#define CAN_manufacturers_device_name_index 0x1008
#define CAN_manufacturers_device_name_access 'R'
#define CAN_manufacturers_device_name_config 'S'
#define CAN_manufacturers_device_name_value 'Master Controller' 

#define CAN_software_version_index 0x100A
#define CAN_software_version_access 'R'
#define CAN_software_version_config 'S'
#define CAN_software_version_value int 95

#define CAN_node_id_jumper_open_index 0x100B
#define CAN_node_id_jumper_open_access 'R'
#define CAN_node_id_jumper_open_config 'S'
#define CAN_node_id_jumper_open_value 0x00003A

#define CAN_node_id_jumper_closed_index 0x100B // 0x100C???
#define CAN_node_id_jumper_closed_access 'R'
#define CAN_node_id_jumper_closed_config 'S'
#define CAN_node_id_jumper_closed_value 0x000038

#define CAN_cmd_store_parameters_index 0x1010 // Saves new parameters to EEPROM
#define CAN_cmd_store_parameters_access 'R+W'
#define CAN_cmd_store_parameters_config 'S'
#define CAN_cmd_store_parameters_value 0

#define CAN_cmd_restore_parameters_index 0x1011 //Restores factory setting parameters
#define CAN_cmd_restore_parameters_access 'R+W'
#define CAN_cmd_restore_parameters_config 'S'
#define CAN_cmd_restore_parameters_value1 0 //Saved Factory Setting parameter Values
#define CAN_cmd_restore_parameters_value2 0 // Parameter definitions INCOMPLETE
#define CAN_cmd_restore_parameters_value3 0
#define CAN_cmd_restore_parameters_value4 0
#define CAN_cmd_restore_parameters_value5 0



#define CAN_producer_heartbeat_time_index 0x1017 
#define CAN_producer_heartbeat_time_access 'R'
#define CAN_producer_heartbeat_time_config 'S'
#define CAN_producer_heartbeat_time_value 50 //generation rate for sending heartbeat messages

#define CAN_error_behavior_index 0x1029 //Defines behavior in a case of serious device failure, according to CiA DS 301
#define CAN_error_behavior_access 'R+W'
#define CAN_error_behavior_config 'S'
#define CAN_error_behavior_value 'Error code' //Defines behavior in a case of serious device failure

#define CAN_SDO1_server_index 0x1200 //SDO1: server parameter. (link)
#define CAN_SDO1_server_access 'R'
#define CAN_SDO1_server_config 'S'
#define CAN_SDO1_server_value 0



#define CAN_PDO1_rx_map_index 0x1600 //PDO1: receive communication parameter. (link)
#define CAN_PDO1_rx_map_access 'R+W'
#define CAN_PDO1_rx_map_config 'S'
#define CAN_PDO1_rx_map_value 0

#define CAN_PDO1_Rx_comm_index 0x1800 //PDO1: transmit communication parameter.
#define CAN_PDO1_Rx_comm_access 'R+W'
#define CAN_PDO1_Rx_comm_config 'S'
#define CAN_PDO1_Rx_comm_value 0

#define CAN_PDO1_tx_map_index 0x1A00 //PDO1: transmit mapping parameter.
#define CAN_PDO1_tx_map_access 'R+W'
#define CAN_PDO1_tx_map_config 'S'
#define CAN_PDO1_tx_map_value 0

#define CAN_encoder_value_index 0x2001 //Bytes describing encoder value Enc_Value
#define CAN_encoder_value_access 'R+W'
#define CAN_encoder_value_config 'P'
#define CAN_encoder_value_value 1023

#define CAN_encoder_validation_index 0x2001 //Bytes describing encoder value validation Enc_DataVal
#define CAN_encoder_validation_access 'R+W'
#define CAN_encoder_validation_config 'P'
#define CAN_encoder_validation_value 0



/*
#define CAN_250K 4
#define CAN_125K 8
#define CAN_100K 10
#define CAN_50K 20
#define ADC_IN1 1
#define ADC_IN2 2
*/
/*
//#define Out1_HS_Pin GPIO_PIN_13
//#define Out1_HS_GPIO_Port GPIOC
#define Out1_HS_Pin GPIO_PIN_14
#define Out1_HS_GPIO_Port GPIOC
#define Out2_HS_Pin GPIO_PIN_15
#define Out2_HS_GPIO_Port GPIOC
#define Out3_HS_Pin GPIO_PIN_4
#define Out3_HS_GPIO_Port GPIOA
#define OUT4_DAC_Pin GPIO_PIN_5
#define OUT4_DAC_GPIO_Port GPIOA
#define LED_Pin GPIO_PIN_8
#define LED_GPIO_Port GPIOA
#define CAN_S_Pin GPIO_PIN_6
#define CAN_S_GPIO_Port GPIOB
*/
/* USER CODE BEGIN Private defines */
/*
#define PRODUCTION_VERSION 0 // Version for Debugging without Watch dog
//#define PRODUCTION_VERSION 1 // Version for Production with Watch dog

#define DIN4_Pin GPIO_PIN_12
#define DIN4_Port GPIOA
#define DIN5_Pin GPIO_PIN_15
#define DIN5_Port GPIOA
#define DIN6_HS_Pin GPIO_PIN_13
#define DIN6_Port GPIOC

#define EEPROM_ADDRESS			0xA0
#define EEPROM_BUFFER_SIZE	32  // Page size
#define EEPROM_WRITE_TIME		5   // Page write time in ms
#define EEPROM_TIMEOUT			10  // timeout for wirite
*/
/* USER CODE END Private defines */

/**
  * @}
  */ 

/**
  * @}
*/ 

#endif /* __OBJECT DICTIONARY */