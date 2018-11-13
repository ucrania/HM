//todo fazer diagrama
//todo meter a piscar o led conforme as fases em que o codigo está.
//todo corrigir double advertising
//todo fazer verificaçao para nao notificar dados repetidos
//*******************+/-feito***********************
//todo organizar o codigo com handlers
//todo enviar raw data com pacotes de tamanho variavel
//todo fazer processamento dos dados
//todo desligar o sensor e ligar so quando o ble for conectado?! para poupar energy
//todo enviar notify a dizer que o sensor foi desconectado
//todo falar com o stor, meter a chamar optimized task aqui, ou meter suspend task e resume quando tiver os dados
//todo resolver a media de ter de meter 2x o dedo no sensor


#define EN_MAX30102_READING_TASK	//enable i2c sensor readings
#define EN_SENSOR0		//enable i2c sensor0	SDA=25	SCL=26	INT=34
#define EN_SENSOR1		//enable i2c sensor1	SDA=18 	SCL=19	INT=35

#define EN_BLE_TASK					//enable bluetooth
#define EN_BLE_BOND_TASK			//enable bond in bluetooth
#define EN_NOTIFY					//enable notify task
#define EN_BATTERY_MEASURMENT_TASK	//enable the battery measurment value
#define PLOT 						//disables other printf
//#define PRINT_ALL_SENSOR_DATA 	//prints all fifo data

//#include "algorithm_by_RF.h"
//#include "algorithm_IK_C.h"
//#include "algorithm_by_RF.cpp"

#include "defines.h"
#include "algorithm_IK_C.h"
#define TRESHOLD_ON 15000

static uint16_t RAW0_str[FIFO_A_FULL/2],RAW1_str[FIFO_A_FULL/2];				//RAW1,RAW2
static bool sensor_have_finger[2]; //flag for finger presence on sensor
static int buffer_pos_s0=-1,buffer_pos_s1=-1;

TaskHandle_t notify_TaskHandle = NULL;

/*********************BLE DEFINES**************************************************************************************/
#include "ble_defines.h"

////end ble part


static xQueueHandle gpio_evt_queue = NULL;


static void i2c_init(i2c_port_t i2c_num){
	i2c_config_t conf;
	conf.mode = I2C_MODE_MASTER;
	conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
	conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
	conf.master.clk_speed = I2C_FREQ_HZ;
	if (i2c_num==I2C_NUM_1){
		conf.sda_io_num = I2C_SDA_IO_1;
		conf.scl_io_num = I2C_SCL_IO_1;
	} else {
		conf.sda_io_num = I2C_SDA_IO_0;
		conf.scl_io_num = I2C_SCL_IO_0;
	}
	i2c_param_config(i2c_num, &conf);
	i2c_driver_install(i2c_num, conf.mode,
			I2C_RX_BUF_DISABLE,
			I2C_TX_BUF_DISABLE, 0);
}
static void max30102_shutdown(i2c_port_t i2c_num){
	uint8_t data_h=0x00;
	max30102_read_reg(REG_MODE_CONFIG,i2c_num, &data_h);
	max30102_write_reg(REG_MODE_CONFIG,i2c_num,0x80 | data_h);	//shutdown and keep the same mode
}
static void max30102_reset(i2c_port_t i2c_num){
	uint8_t data_h=0x00;
	max30102_read_reg(REG_INTR_STATUS_1,i2c_num,&data_h);//clear interrupt pin
	int ret = max30102_read_reg(REG_MODE_CONFIG, i2c_num, &data_h);
	max30102_write_reg(REG_MODE_CONFIG,i2c_num,0x40 | data_h);	//reset and keep the same mode
}
static void max30102_init(i2c_port_t i2c_num){
	printf("***MAX30102 initialization***\n");
	int ret;
	do{
		ret = max30102_write_reg(REG_INTR_ENABLE_1,i2c_num, PROX_INT_EN );	//0b1001 0000
		max30102_write_reg(REG_INTR_ENABLE_2,i2c_num,0x00);		//Temperature interrupt
		max30102_write_reg(REG_FIFO_WR_PTR,i2c_num,0x00);		//
		max30102_write_reg(REG_OVF_COUNTER,i2c_num,0x00);		//
		max30102_write_reg(REG_FIFO_RD_PTR,i2c_num,0x00);		//
		max30102_write_reg(REG_FIFO_CONFIG,i2c_num,get_FIFO_CONF_REG());		//4 average sample + FIFO_A_FULL 15
		max30102_write_reg(REG_MODE_CONFIG,i2c_num,0x03);		//0x07 -> Multi-LED  //0x03 -> SpO2 mode //0x02 -> HR mode
		max30102_write_reg(REG_SPO2_CONFIG,i2c_num,get_SPO2_CONF_REG());		//01 adc range + 100 samples/sec + 18  bit resolution
		max30102_write_reg(REG_LED1_PA,i2c_num,get_LED1_PA());			//RED LED current
		max30102_write_reg(REG_LED2_PA,i2c_num,get_LED2_PA());			//IR  LED current
		max30102_write_reg(REG_PILOT_PA,i2c_num,get_LED1_PA());		//Multimode registers (not used)
		max30102_write_reg(REG_PROX_INT_THRESH,i2c_num,0x30);
	}while(ret != ESP_OK);
}

static void IRAM_ATTR gpio_isr_handler(void* arg){
	uint32_t gpio_num = (uint32_t) arg;
	//printf("INTERRUPTION on pin %d",(int)arg);
	xTaskCreate(isr_task_manager, "isr_task_manager", 1024 * 2, (void* ) arg, 10, NULL);

	//xQueueSendFromISR(gpio_evt_queue, &gpio_num, NULL);
}


void app_main()
{
	raw_ptr0_IR = malloc(FIFO_A_FULL);
	raw_ptr0_RED =malloc(FIFO_A_FULL);
	raw_ptr1_IR = malloc(FIFO_A_FULL);
	raw_ptr1_RED =malloc(FIFO_A_FULL);
	char7_RAW1_val.attr_value = raw_ptr0_IR;
	char8_RAW2_val.attr_value = raw_ptr1_IR;

	/*float SPO2_value=99.87654321;
	int32_t spo2_32;
	spo2_32 = memcpy(&spo2_32,&SPO2_value,sizeof(float));
	short spo_aux;
	spo_aux    =  ((spo2_32 & 0x7fffffff) >> 13) - (0x38000000 >> 13);
	spo_aux    |= ((spo2_32 & 0x80000000) >> 16);
	printf("spo_aux = %d\t= 0x%x\n",spo_aux,spo_aux);

	spo_aux    =  ((spo2_32 & 0x7fffffff) >> 13) - (0x38000000 >> 13);
	spo_aux    |= ((spo2_32 & 0x80000000) >> 16);
	//printf("spo_aux = 0x%x\n",spo_aux);
	return;*/

#ifdef EN_BATTERY_MEASURMENT_TASK

#endif
#ifdef EN_BLE_TASK
	bt_main();
#ifndef CONFIG_BT_ENABLED
	esp_light_sleep_start();
#endif //CONFIG_BT_ENABLED
#endif //EN_BLE_TASK

#ifdef EN_MAX30102_READING_TASK
	printf("Start app_main\n");
	intr_init();
#ifdef EN_SENSOR0
	i2c_init(I2C_NUM_0);
	max30102_shutdown(I2C_NUM_0);
#endif
#ifdef EN_SENSOR1
	i2c_init(I2C_NUM_1);
	max30102_shutdown(I2C_NUM_1);
#endif
#endif
	//xTaskCreate(i2c_task_0, "i2c_test_task_0", 1024 * 2, (void* ) 0, 10, NULL);
	//xTaskCreate(i2c_task_1, "i2c_test_task_1", 1024 * 2, (void* ) 0, 10, NULL);
	//xTaskCreate(blink_task, "blink_task"	 , 1024 * 2, (void* ) 0, 10, NULL);

}

void i2c_task_0(void* arg)
{
	printf("Start task 0!\n");
	int i = 0;
	int ret;
	/*uint32_t task_idx = (uint32_t) arg;
	//uint8_t* data = (uint8_t*) malloc(DATA_LENGTH);
	//uint8_t* data_wr = (uint8_t*) malloc(DATA_LENGTH);
	//uint8_t* data_rd = (uint8_t*) malloc(DATA_LENGTH);*/

	uint8_t sensor_data_h;
	while(1){
		ret = max30102_read_reg(REG_LED1_PA, I2C_NUM_1, &sensor_data_h);
		check_ret(ret,sensor_data_h);
		vTaskDelay(pdMS_TO_TICKS(1000));
	}
	vTaskDelete(NULL);

}

void i2c_task_1(void* arg)
{	double mean1, mean2;
#ifndef PLOT
printf("Start i2c_task_1!\n");
#endif
int *buffer_pos;
i2c_port_t port = (i2c_port_t)arg;	//I2C_NUM_0 or I2C_NUM_1
uint16_t RAWsensorDataRED[FIFO_A_FULL/2], RAWsensorDataIR[FIFO_A_FULL/2];
max30102_read_fifo(port, RAWsensorDataRED,RAWsensorDataIR);
if (port == I2C_NUM_0){ //if sensor0
	buffer_pos = &buffer_pos_s0;
	if(*buffer_pos >= 0){
		raw_ptr0_IR = realloc(raw_ptr0_IR,sizeof(RAWsensorDataIR)*(*buffer_pos+1));
		raw_ptr0_RED = realloc(raw_ptr0_RED,sizeof(RAWsensorDataRED)*(*buffer_pos+1));
		ESP_LOGI("reallocated"," %d bytes\tsensor: %d\n",sizeof(RAWsensorDataIR)*(*buffer_pos+1),port);
		memcpy(raw_ptr0_IR+(sizeof(RAWsensorDataIR)*(*buffer_pos)),RAWsensorDataIR,sizeof(RAWsensorDataIR));
		memcpy(raw_ptr0_RED+(sizeof(RAWsensorDataRED)*(*buffer_pos)),RAWsensorDataRED,sizeof(RAWsensorDataRED));
		//print_array(raw_ptr0,(sizeof(RAWsensorDataIR)/2)*(*buffer_pos+1));
	}
}else{
	buffer_pos = &buffer_pos_s1;
	if(*buffer_pos >= 0){
		raw_ptr1_IR = realloc(raw_ptr1_IR,sizeof(RAWsensorDataIR)*(*buffer_pos+1));
		raw_ptr1_RED = realloc(raw_ptr1_RED,sizeof(RAWsensorDataRED)*(*buffer_pos+1));
		ESP_LOGI("reallocated","%d bytes\tsensor: %d\n",sizeof(RAWsensorDataIR)*(*buffer_pos+1),port);
		memcpy(raw_ptr1_IR+(sizeof(RAWsensorDataIR)*(*buffer_pos)),RAWsensorDataIR,sizeof(RAWsensorDataIR));
		memcpy(raw_ptr1_RED+(sizeof(RAWsensorDataRED)*(*buffer_pos)),RAWsensorDataRED,sizeof(RAWsensorDataRED));
		//print_array(raw_ptr1,(sizeof(RAWsensorDataIR)/2)*(*buffer_pos+1));
	}
}
if (notify_TaskHandle != NULL){
	*buffer_pos +=1;
}
if(*buffer_pos > 0){//ignore 1st read

	//double SPO2 = process_data(RAWsensorDataRED,RAWsensorDataIR,&mean1,&mean2);
	data_mean(RAWsensorDataRED,RAWsensorDataIR,&mean1,&mean2);
	printf("Mean:\t%f,\t%f\n",mean1,mean2);
	//fprintf(stdout,"\tSPO2: %02f%%\n\n",SPO2);
	if(mean1<TRESHOLD_ON && mean2<TRESHOLD_ON){	//IF NO FINGER
		if (port == I2C_NUM_0){
			sensor_have_finger[0] = false;
			*buffer_pos = -1;
		}else {
			sensor_have_finger[1] = false;
			*buffer_pos = -1;
		}
		max30102_write_reg(REG_INTR_ENABLE_1,port, PROX_INT_EN);	//Enable proximity interruption
	}
	if(notify_TaskHandle != NULL && *buffer_pos >= PACKS_TO_SEND){//Packet size
		float SPO2_value=0;

		int32_t HR_value=0,data_len= (sizeof(RAW0_str)*(int)(*buffer_pos)/2); //data lenght
		float ratio,correl;
		int8_t spo2_valid, hr_valid;

		uint32_t IRdata_to_process[data_len],REDdata_to_process[data_len];
		uint16_t *raw_ptr_aux0_IR=raw_ptr0_IR,*raw_ptr_aux0_RED = raw_ptr0_RED
				,*raw_ptr_aux1_IR=raw_ptr1_IR,*raw_ptr_aux1_RED = raw_ptr1_RED;
		for(int i=0;i<data_len;i++){ //convert data to 32bit pointer
			if (port == I2C_NUM_0){//todo optimizar com ponteiros estes ifs
				IRdata_to_process [i]= raw_ptr_aux0_IR[i];
				REDdata_to_process[i]= raw_ptr_aux0_RED[i];
			}else{
				IRdata_to_process [i]= raw_ptr_aux1_IR[i];
				REDdata_to_process[i]= raw_ptr_aux1_RED[i];
			}
		}
		heart_rate_and_oxygen_saturation(IRdata_to_process,data_len,REDdata_to_process,&SPO2_value,&spo2_valid,&HR_value,&hr_valid,&ratio,&correl);

		if(spo2_valid && hr_valid){
			ESP_LOGW("Processamento","HR: %d\tSPO2: %f\n",HR_value,SPO2_value);
			uint32_t spo2_32;
			uint16_t spo2_16;
			uint8_t *spo2_8;
			memcpy(&spo2_32,&SPO2_value,sizeof(float));
			//passing from float32 to float16 and assigning in the array
			spo2_16    =  ((spo2_32 & 0x7fffffff) >> 13) - (0x38000000 >> 13);
			spo2_16    |= ((spo2_32 & 0x80000000) >> 16);
			spo2_8 = &spo2_16;

			//PLX2_str[1]=SPO2_value;
			if (port == I2C_NUM_0){
				HR1_str[1]=HR_value;
				PLX1_str[1]=spo2_8[0];
				PLX1_str[2]=spo2_8[1];
			}else{
				HR2_str[1]=HR_value;
				PLX2_str[1]=spo2_8[0];
				PLX2_str[2]=spo2_8[1];
			}
		}else{
			ESP_LOGE("Processamento","HR: %d\tSPO2: %f\n",HR_value,SPO2_value);
		}
		char7_RAW1_val.attr_value=raw_ptr0_IR;
		char8_RAW2_val.attr_value=raw_ptr1_IR;
		char7_RAW1_val.attr_len = sizeof(RAW0_str)*(int)(*buffer_pos);	//attribute lenght in bytes
		char8_RAW2_val.attr_len = sizeof(RAW1_str)*(int)(*buffer_pos);
		//esp_ble_gatt_set_local_mtu(sizeof(RAW1_str) *buffer_pos + 3);
		vTaskResume(notify_TaskHandle);
		*buffer_pos = 0;
	}
}else{
	printf("******Ignoring first read!*****\n\n");
}

#ifndef CONFIG_BT_ENABLED
esp_light_sleep_start();
#endif
vTaskDelete(NULL);
}

void blink_task(void* arg)
{
	gpio_pad_select_gpio(BLINK_GPIO);
	gpio_set_direction(BLINK_GPIO, GPIO_MODE_OUTPUT);
	printf("Start blinking\n");
	int led = 1,i=0;

	while(1){
		//printf("Blink\t%d\n",i++);
		gpio_set_level(BLINK_GPIO, led=!led);
		vTaskDelay(pdMS_TO_TICKS(1000));
	}
	vTaskDelete(NULL);

}

void isr_task_manager(void* arg)
{
	ESP_LOGI("ISR_TASK_MANAGER","on core %d\tpin: %d",xPortGetCoreID(),(int)arg);
	uint8_t data=0x00;
	i2c_port_t port = (i2c_port_t)arg == INT_PIN_0 ? I2C_NUM_0: I2C_NUM_1;

	max30102_read_reg(REG_INTR_STATUS_1,port,&data); //clear interruption in the sensor

	bool fifo_a_full_int = data>>7 & 0x01;
	bool prox_int = data>>4 & 0x01;
	bool alc_ovf = data>>5 & 0x01;
#ifndef PLOT
	printf("\tINT Reason: 0x%02x\t",data);
	fifo_a_full_int ? printf("\tFIFO A FULL\n") : NULL;
	prox_int 		? printf("\tPROX INT\n") : NULL;
#endif
	if (prox_int){
		if ((i2c_port_t)arg == INT_PIN_0){
			sensor_have_finger[0] = true ;
		}else{
			sensor_have_finger[1] = true ;
		}
		//clear buffer
		max30102_write_reg(REG_INTR_ENABLE_1,port, FIFO_A_FULL_EN);	//0b1000 0000 //disable prox interrupt and enable fifo_a_full
	}

	//xTaskCreate(i2c_task_0, "i2c_test_task_0", 1024 * 4, (void* ) 0, 10, NULL);
	xTaskCreate(i2c_task_1, "i2c_task_1", 1024 * 4, (void* ) port, 10, NULL);	//4kB stack size
	vTaskDelete(NULL);
}

esp_err_t max30102_write_reg(uint8_t uch_addr, i2c_port_t i2c_num, uint8_t puch_data){

	i2c_cmd_handle_t cmd = i2c_cmd_link_create();
	i2c_master_start(cmd);
	i2c_master_write_byte(cmd, MAXREFDES117_ADDR << 1 | WRITE_BIT, ACK_CHECK_EN);
	i2c_master_write_byte(cmd, uch_addr, ACK_CHECK_EN);
	i2c_master_write_byte(cmd, puch_data, ACK_CHECK_EN);
	i2c_master_stop(cmd);
	int ret = i2c_master_cmd_begin(i2c_num, cmd, 1000 / portTICK_RATE_MS);
	i2c_cmd_link_delete(cmd);
	//printf("writing to 0x%02x\n",uch_addr);
	if (ret != ESP_OK) {
		printf("ESP NOT OK\n");
		return ret;
	}

	return ret;
}

esp_err_t max30102_read_reg(uint8_t uch_addr,i2c_port_t i2c_num, uint8_t* data_h)
{
	int ret;
	i2c_cmd_handle_t cmd = i2c_cmd_link_create();
	i2c_master_start(cmd);
	i2c_master_write_byte(cmd, MAXREFDES117_ADDR << 1 | WRITE_BIT, ACK_CHECK_EN);
	i2c_master_write_byte(cmd, uch_addr, ACK_CHECK_EN);
	i2c_master_stop(cmd);	//send the stop bit
	ret = i2c_master_cmd_begin(i2c_num, cmd, 100 / portTICK_RATE_MS);
	i2c_cmd_link_delete(cmd);
	if (ret != ESP_OK) {
		printf("ESP NOT OK READING\n");
		return ret;
	}
	vTaskDelay(50 / portTICK_RATE_MS);
	cmd = i2c_cmd_link_create();
	i2c_master_start(cmd);
	i2c_master_write_byte(cmd, MAXREFDES117_ADDR << 1 | READ_BIT, ACK_CHECK_EN);
	i2c_master_read_byte(cmd, data_h, NACK_VAL);
	i2c_master_stop(cmd);
	ret = i2c_master_cmd_begin(i2c_num, cmd, 100 / portTICK_RATE_MS);
	i2c_cmd_link_delete(cmd);
	return ret;
}

esp_err_t max30102_read_fifo(i2c_port_t i2c_num, uint16_t sensorDataRED[],uint16_t sensorDataIR[])
{

	uint8_t LED_1[FIFO_A_FULL/2][3],LED_2[FIFO_A_FULL/2][3];

	i2c_cmd_handle_t cmd = i2c_cmd_link_create();
	i2c_master_start(cmd);
	i2c_master_write_byte(cmd, MAXREFDES117_ADDR << 1 | WRITE_BIT, ACK_CHECK_EN);
	i2c_master_write_byte(cmd, REG_FIFO_DATA, ACK_CHECK_EN);
	i2c_master_stop(cmd);	//send the stop bit
	int ret = i2c_master_cmd_begin(i2c_num, cmd, 100 / portTICK_RATE_MS);
	i2c_cmd_link_delete(cmd);
	if (ret != ESP_OK) {
		printf("ESP NOT OK!!\n");
		return ret;
	}
	vTaskDelay(25 / portTICK_RATE_MS);

	cmd = i2c_cmd_link_create();
	i2c_master_start(cmd);
	i2c_master_write_byte(cmd, MAXREFDES117_ADDR << 1 | READ_BIT, ACK_CHECK_EN);

	for(int i=0; i < FIFO_A_FULL/2; i++){
		i2c_master_read_byte(cmd, &LED_1[i][0], ACK_VAL);
		i2c_master_read_byte(cmd, &LED_1[i][1], ACK_VAL);
		i2c_master_read_byte(cmd, &LED_1[i][2], ACK_VAL);

		i2c_master_read_byte(cmd, &LED_2[i][0], ACK_VAL);
		i2c_master_read_byte(cmd, &LED_2[i][1], ACK_VAL);
		i2c_master_read_byte(cmd, &LED_2[i][2], i==FIFO_A_FULL/2-1? NACK_VAL: ACK_VAL);	//NACK_VAL on the last iteration
	}

	i2c_master_stop(cmd);
	ret = i2c_master_cmd_begin(i2c_num, cmd, 100 / portTICK_RATE_MS);
	i2c_cmd_link_delete(cmd);

	for(int i=0; i < FIFO_A_FULL/2; i++){
		/*sensorDataRED[i] = (((LED_1[i][0] &  0b00000011) <<16) + (LED_1[i][1] <<8) + LED_1[i][2])>>(18-SPO2_RES)<<(18-SPO2_RES);
		sensorDataIR[i]= (((LED_2[i][0] &  0b00000011) <<16) + (LED_2[i][1] <<8) + LED_2[i][2])>>(18-SPO2_RES)<<(18-SPO2_RES);	//*/
		//printf("\t%x %x %x\n",LED_1[i][0]&0x03,LED_1[i][1],LED_1[i][2]);

		sensorDataRED[i] = 	((((LED_1[i][0] &  0b00000011) <<16) + (LED_1[i][1] <<8) + LED_1[i][2])>>(18-SPO2_RES)<<(18-SPO2_RES))>>2;
		sensorDataIR[i]= 	((((LED_2[i][0] &  0b00000011) <<16) + (LED_2[i][1] <<8) + LED_2[i][2])>>(18-SPO2_RES)<<(18-SPO2_RES))>>2;	//
		/*#if SPO2_RES <= 16
		sensorDataRED[i] =sensorDataRED[i] >>2;
		sensorDataIR[i]  =sensorDataIR[i]  >>2;
#endif*/
		//fprintf(stdout,"%x, %x, %x\t%x, %x, %x\n",LED_1[i][0],LED_1[i][1],LED_1[i][2],LED_2[i][0],LED_2[i][1],LED_2[i][2]);
#ifdef PRINT_ALL_SENSOR_DATA
		fprintf(stdout,"0x%x\t0x%x\n",sensorDataRED[i],sensorDataIR[i]);
#endif
	}


	/*int data1 = (((LED_1[i][0] && 0b00000011) <<16) + (LED_1[i][1] <<8) + LED_1[i][2])>>(18-SPO2_RES);
		/int data2 = (((LED_2[i][0] && 0b00000011) <<16) + (LED_2[i][1] <<8) + LED_2[i][2])>>(18-SPO2_RES);	*/

	//vTaskDelay(6000 / portTICK_RATE_MS);//magic delay

	return ret;
}

void check_ret(int ret,uint8_t sensor_data_h){
	if(ret == ESP_ERR_TIMEOUT) {
		printf("I2C timeout\n");
	} else if(ret == ESP_OK) {
		printf("******************* \n");
		printf("TASK[%d]  MASTER READ SENSOR( EN_MAX30102_READING_TASK )\n", 0);
		printf("*******************\n");
		printf("data: %02x\n", sensor_data_h);
	} else {
		printf("%s: No ack, sensor not connected...skip...\n", esp_err_to_name(ret));
	}
}

void check_fifo(int ret,uint8_t sensor_data_h, uint8_t sensor_data_m, uint8_t sensor_data_l){
	if(ret == ESP_ERR_TIMEOUT) {
		printf("I2C timeout\n");
	} else if(ret == ESP_OK) {
		printf("******************* \n");
		printf("READING FIFO ( EN_MAX30102_READING_TASK )\n");
		printf("*******************\n");
		printf("data_h: %02x\n", sensor_data_h);
		printf("data_m: %02x\n", sensor_data_m);
		printf("data_l: %02x\n", sensor_data_l);

		printf("sensor val: %d\n", (sensor_data_h << 16 |sensor_data_m << 8 | sensor_data_l));
	} else {
		printf("%s: No ack, sensor not connected...skip...\n", esp_err_to_name(ret));
	}
}

uint8_t get_SPO2_CONF_REG(){
	uint8_t spo2_adc_rge, spo2_sr, spo2_res;
#ifndef SPO2_SAMPLE_RATE
#define SPO2_SAMPLE_RATE 100
#endif
	switch (SPO2_SAMPLE_RATE){
	case 50:
		spo2_sr =  0x00;
		break;
	case 100:
		spo2_sr =  0x01;
		break;
	case 200:
		spo2_sr =  0x02;
		break;
	case 400:
		spo2_sr =  0x03;
		break;
	case 800:
		spo2_sr =  0x04;
		break;
	case 1000:
		spo2_sr =  0x05;
		break;
	case 1600:
		spo2_sr =  0x06;
		break;
	case 3600:
		spo2_sr =  0x07;
		break;
	default:
		spo2_sr =  0x01;
	}
#ifndef SPO2_RES
#define SPO2_RES 18
#endif
	switch (SPO2_RES){
	case 15:
		spo2_res =  0x00;
		break;
	case 16:
		spo2_res =  0x01;
		break;
	case 17:
		spo2_res =  0x02;
		break;
	case 18:
		spo2_res =  0x03;
		break;
	default:
		spo2_res =  0x00;
	}
#ifndef SPO2_ADC_RGE
#define SPO2_ADC_RGE 0x01
#endif
	switch (SPO2_ADC_RGE){
	case 0x00:
		spo2_adc_rge =  0x00;
		break;
	case 0x01:
		spo2_adc_rge =  0x01;
		break;
	case 0x02:
		spo2_adc_rge =  0x02;
		break;
	case 0x03:
		spo2_adc_rge =  0x03;
		break;
	default:
		spo2_adc_rge =  0x00;
	}

	//printf("REGISTER is: 0x%x\n",spo2_adc_rge<<5 | spo2_sr<<2 | spo2_res);
	return spo2_adc_rge<<5 | spo2_sr<<2 | spo2_res;
}

uint8_t get_FIFO_CONF_REG(){
	uint8_t smp_ave,fifo_rollover_en,fifo_a_full;
#ifndef SMP_AVE
#define SMP_AVE 8
#endif
	switch (SMP_AVE){
	case 1:
		smp_ave =  0x00;
		break;
	case 2:
		smp_ave =  0x01;
		break;
	case 4:
		smp_ave =  0x02;
		break;
	case 8:
		smp_ave =  0x03;
		break;
	case 16:
		smp_ave =  0x04;
		break;
	case 32:
		smp_ave =  0x05;
		break;
	default:
		smp_ave =  0x03;
	}

#ifndef FIFO_ROLLOVER_EN
#define FIFO_ROLLOVER_EN 0
#endif
	fifo_rollover_en = FIFO_ROLLOVER_EN;

#ifndef FIFO_A_FULL
#define FIFO_A_FULL 30
#endif
	fifo_a_full = FIFO_A_FULL >32 ? 2: 32-FIFO_A_FULL;

	//printf("REGISTER is: 0x%x\n",smp_ave<<5 | fifo_rollover_en<<4 | fifo_a_full);
	return smp_ave<<5 | fifo_rollover_en<<4 | fifo_a_full;
}

uint8_t get_LED1_PA(){	//RED LED CURRENT

#if  !defined(LED1_CURRENT) && !defined(LED1_PA)
#define LED1_CURRENT 7	//0-50 mA	0.2mA resolution
#define LED1_PA 0x24	//0-255 brightness
	return LED1_CURRENT*5;
#elif	defined(LED1_CURRENT)
	return LED1_CURRENT*5;
#elif	defined(LED1_PA)
	return LED1_PA;
#else
	return 35;
#endif
}

uint8_t get_LED2_PA(){ //IR LED CURRENT
#if  !defined(LED2_CURRENT) && !defined(LED2_PA)
#define LED2_CURRENT 7	//0-50 mA	0.2mA resolution
#define LED2_PA 0x24	//0-255 brightness
	return LED2_CURRENT*5;
#elif	defined(LED2_CURRENT)
	return LED2_CURRENT*5;
#elif	defined(LED2_PA)
	return LED2_PA;
#else
	return 35;
#endif
}

void intr_init(){
	gpio_config_t io_conf;
	io_conf.intr_type = GPIO_PIN_INTR_NEGEDGE;		//interrupt of falling edge
	io_conf.pin_bit_mask = GPIO_INPUT_PIN_SEL;		//bit mask of the pins, use GPIO25/26 here
	io_conf.mode = GPIO_MODE_INPUT;				//set as input mode
	io_conf.pull_down_en = 0;						//disable pull-down mode
	io_conf.pull_up_en = 1;						//enable pull-up mode
	gpio_config(&io_conf);
	//gpio_set_intr_type(INT_PIN_0, GPIO_INTR_NEGEDGE);	//interrupt of falling edge
	//gpio_set_intr_type(INT_PIN_1, GPIO_INTR_NEGEDGE);	//interrupt of falling edge
	rtc_gpio_pullup_en(INT_PIN_0);
	rtc_gpio_pullup_en(INT_PIN_1);
	gpio_install_isr_service(ESP_INTR_FLAG_DEFAULT);		//install gpio isr service
	gpio_isr_handler_add(INT_PIN_0, gpio_isr_handler, (void*) INT_PIN_0);	//hook isr handler for specific gpio pin
	gpio_isr_handler_add(INT_PIN_1, gpio_isr_handler, (void*) INT_PIN_1);	//hook isr handler for specific gpio pin


	esp_sleep_enable_ext1_wakeup(GPIO_INPUT_PIN_SEL,ESP_EXT1_WAKEUP_ALL_LOW); //enable external wake up by pin 34 and 35
}

double process_data(uint16_t RAWsensorDataRED[],uint16_t RAWsensorDataIR[],double *mean1, double *mean2){
	double rms1, rms2, dc1, dc2, R,SPO2;
		data_rms(RAWsensorDataRED,RAWsensorDataIR,&rms1,&rms2);
		data_mean(RAWsensorDataRED,RAWsensorDataIR,&dc1,&dc2);
		R = (rms1/dc1)/(rms2/dc2);
		SPO2 = 110-(25*R);
	#ifndef PLOT
		//printf("\tVALOR RMS: %f e %f\n",rms1,rms2);
		//printf("\tVALOR DC: %f e %f\n",dc1,dc2);
		//printf("\tVALOR R: %f \n",R);
		printf("\tVALOR SPO2: %f \n",SPO2);
	#endif
		*mean1 = dc1;
		*mean2 = dc2;
	return SPO2;
}

void data_rms(uint16_t RAWsensorDataRED[],uint16_t RAWsensorDataIR[],double *rms_1, double *rms_2){
	double dataLength = sizeof(RAWsensorDataRED)/sizeof(int);
	double sum_led1=0 , sum_led2 =0;
	for (int i = 0; i < dataLength; ++i) {
		sum_led1 +=(RAWsensorDataRED[i])^2;
		sum_led2 +=(RAWsensorDataIR[i])^2;
	}

	*rms_1= sqrt((1/dataLength)*sum_led1);
	*rms_2= sqrt((1/dataLength)*sum_led2);
	return;
}

void data_mean(uint16_t RAWsensorDataRED[],uint16_t RAWsensorDataIR[],double *mean1, double *mean2){
	double dataLength = FIFO_A_FULL/2;
	double sum1=0 ,sum2=0 ;
	for (int i = 0; i < dataLength; ++i) {
		sum1 += RAWsensorDataRED[i];
		sum2 += RAWsensorDataIR[i];
	}
	*mean1 = sum1/dataLength;
	*mean2 = sum2/dataLength;
}

/*********************BLE PART**************************************************************************************/

//***************DESCRIPTORS**********************//
esp_attr_value_t gatts_demo_descr1_val = {
		.attr_max_len = 22,
		.attr_len		= sizeof(descr1_str),
		.attr_value     = descr1_str,
};

static uint8_t adv_config_done = 0;
#define adv_config_flag      (1 << 0)
#define scan_rsp_config_flag (1 << 1)

#ifdef CONFIG_SET_RAW_ADV_DATA
static uint8_t raw_adv_data[] = {
		0x02, 0x01, 0x06,
		0x02, 0x0a, 0xeb, 0x03, 0x03, 0xab, 0xcd
};
static uint8_t raw_scan_rsp_data[] = {
		0x0f, 0x09, 0x45, 0x53, 0x50, 0x5f, 0x47, 0x41, 0x54, 0x54, 0x53, 0x5f, 0x44,
		0x45, 0x4d, 0x4f
};
#else

static uint8_t adv_service_uuid128[32] = {
		/* LSB <--------------------------------------------------------------------------------> MSB */
		//first uuid, 16bit, [12],[13] is the value
		0xfb, 0x34, 0x9b, 0x5f, 0x80, 0x00, 0x00, 0x80, 0x00, 0x10, 0x00, 0x00, 0xEE, 0x00, 0x00, 0x00,
		//second uuid, 32bit, [12], [13], [14], [15] is the value
		0xfb, 0x34, 0x9b, 0x5f, 0x80, 0x00, 0x00, 0x80, 0x00, 0x10, 0x00, 0x00, 0xFF, 0x00, 0x00, 0x00,
};

static uint8_t heart_rate_service_uuid[48] = {
		/* LSB <--------------------------------------------------------------------------------> MSB */
		//first uuid, 16bit, [12],[13] is the value
		0xfb, 0x34, 0x9b, 0x5f, 0x80, 0x00, 0x00, 0x80, 0x00, 0x10, 0x00, 0x00, 0x0D, 0x18, 0x00, 0x00,
		//second uuid, 32bit, [12], [13], [14], [15] is the value
		0xfb, 0x34, 0x9b, 0x5f, 0x80, 0x00, 0x00, 0x80, 0x00, 0x10, 0x00, 0x00, 0x37, 0x2A, 0x00, 0x00,
		//third uuid, 32bit, [12], [13], [14], [15] is the value
		0xfb, 0x34, 0x9b, 0x5f, 0x80, 0x00, 0x00, 0x80, 0x00, 0x10, 0x00, 0x00, 0x38, 0x2A, 0x00, 0x00,
};
// The length of adv data must be less than 31 bytes
//static uint8_t test_manufacturer[TEST_MANUFACTURER_DATA_LEN] =  {0x12, 0x23, 0x45, 0x56};
//adv data
static esp_ble_adv_data_t adv_data = {
		.set_scan_rsp = false,
		.include_name = true,
		.include_txpower = true,
		.min_interval = 0x20,	//0x20
		.max_interval = 0x40, 	//0x40
		.appearance = 0x0340,//0x0C40		//http://dev.ti.com/tirex/content/simplelink_cc2640r2_sdk_1_35_00_33/docs/blestack/ble_sw_dev_guide/doxygen/group___g_a_p___appearance___values.html#gafc2f463732a098c1b42d30a766e90a6e
		.manufacturer_len = 0, //TEST_MANUFACTURER_DATA_LEN,
		.p_manufacturer_data =  NULL, //&test_manufacturer[0],
		.service_data_len = 0,
		.p_service_data = NULL,
		.service_uuid_len = sizeof(heart_rate_service_uuid),
		.p_service_uuid = heart_rate_service_uuid,
		.flag = (ESP_BLE_ADV_FLAG_GEN_DISC | ESP_BLE_ADV_FLAG_BREDR_NOT_SPT),
};
// scan response data
static esp_ble_adv_data_t scan_rsp_data = {
		.set_scan_rsp = true,
		.include_name = true,
		.include_txpower = true,
		.min_interval = 0x05,
		.max_interval = 0x10,
		.appearance = 0x0340,
		.manufacturer_len = 0, //TEST_MANUFACTURER_DATA_LEN,
		.p_manufacturer_data =  NULL, //&test_manufacturer[0],
		.service_data_len = 0,
		.p_service_data = NULL,
		.service_uuid_len = sizeof(heart_rate_service_uuid),
		.p_service_uuid = heart_rate_service_uuid,
		.flag = (ESP_BLE_ADV_FLAG_GEN_DISC | ESP_BLE_ADV_FLAG_BREDR_NOT_SPT), //TODO ver as flags
};


#endif /* CONFIG_SET_RAW_ADV_DATA */

static esp_ble_adv_params_t adv_params = {
		.adv_int_min        = 0x0180,
		.adv_int_max        = 0x0580,
		.adv_type           = ADV_TYPE_IND,
		.own_addr_type      = BLE_ADDR_TYPE_PUBLIC,
		//.peer_addr          = {0x79, 0xdb, 0xa7, 0x91, 0x13, 0x18},
		//.peer_addr_type     = BLE_ADDR_TYPE_PUBLIC,
		.channel_map        = ADV_CHNL_ALL,
		.adv_filter_policy  = ADV_FILTER_ALLOW_SCAN_ANY_CON_ANY,
};


uint8_t char1_test_str[] = {0xE,1,123,0,13551>>8,13551&0x0F};	//Heart Rate//TODO delete this line

struct gatts_profile_inst {
	esp_gatts_cb_t gatts_cb;
	uint16_t gatts_if;
	uint16_t app_id;
	uint16_t conn_id;
	uint16_t service_handle;
	uint16_t char_num;
	uint16_t char_num_total;
	esp_gatt_srvc_id_t service_id;
	esp_bt_uuid_t char_uuid;
	esp_gatt_perm_t perm;
	esp_gatt_char_prop_t property;
	uint16_t descr_handle;
	esp_bt_uuid_t descr_uuid;
	uint16_t char_handle[2];
	int sensor_id;
};

static struct gatts_profile_inst gl_profile_tab[] = {
		[PROFILE_HR1_APP_ID] = {
				.char_num =0,
				.char_num_total = GATTS_CHAR_NUM_HR1,
				.gatts_cb = gatts_profile_a_event_handler,
				.gatts_if = ESP_GATT_IF_NONE,       /* Not get the gatt_if, so initial is ESP_GATT_IF_NONE */
				.sensor_id = 0,
		},
		[PROFILE_PLX1_APP_ID] = {
				.char_num =0,
				.char_num_total = GATTS_CHAR_NUM_PLX1,
				.gatts_cb = gatts_profile_b_event_handler,
				.gatts_if = ESP_GATT_IF_NONE,       /* Not get the gatt_if, so initial is ESP_GATT_IF_NONE */
				.sensor_id = 0,
		},
		[PROFILE_HR2_APP_ID] = {
				.char_num =0,
				.char_num_total = GATTS_CHAR_NUM_HR2,
				.gatts_cb = gatts_profile_c_event_handler,
				.gatts_if = ESP_GATT_IF_NONE,       /* Not get the gatt_if, so initial is ESP_GATT_IF_NONE */
				.sensor_id = 1,
		},
		[PROFILE_PLX2_APP_ID] = {
				.char_num =0,
				.char_num_total = GATTS_CHAR_NUM_PLX1,
				.gatts_cb = gatts_profile_d_event_handler,
				.gatts_if = ESP_GATT_IF_NONE,       /* Not get the gatt_if, so initial is ESP_GATT_IF_NONE */
				.sensor_id = 1,
		},
		[PROFILE_RAW_APP_ID] = {
				.char_num =0,
				.char_num_total = GATTS_CHAR_NUM_RAW,
				.gatts_cb = gatts_profile_e_event_handler,
				.gatts_if = ESP_GATT_IF_NONE,       /* Not get the gatt_if, so initial is ESP_GATT_IF_NONE */
				.sensor_id = -1,
		},
		[PROFILE_BATT_APP_ID] = {
				.char_num =0,
				.char_num_total = GATTS_CHAR_NUM_BATT,
				.gatts_cb = gatts_profile_f_event_handler,
				.gatts_if = ESP_GATT_IF_NONE,       /* Not get the gatt_if, so initial is ESP_GATT_IF_NONE */
				.sensor_id = -1,
		}

};


static struct gatts_char_inst gl_char[] = {
		{		//0 Body Location Characteristic
				.char_uuid.len = ESP_UUID_LEN_16,  // TX
				.char_uuid.uuid.uuid16 =  GATTS_CHAR_UUID_BODY_LOCATION,
				.char_perm = ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
				.char_property = ESP_GATT_CHAR_PROP_BIT_READ | ESP_GATT_CHAR_PROP_BIT_WRITE,
				.char_val = &char1_BL_val,
				.char_control=NULL,
				.char_handle = 0,
				.char_read_callback=char1_read_handler,
				.char_write_callback=char1_write_handler,
				.descr_uuid.len = 0,
				.descr_uuid.uuid.uuid16 = ESP_GATT_UUID_CHAR_CLIENT_CONFIG, // ESP_GATT_UUID_CHAR_DESCRIPTION,  0x2902
				.descr_perm=ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
				.descr_val = &gatts_demo_descr1_val,
				.descr_control=NULL,
				.descr_handle=0,
				.descr_read_callback=descr1_read_handler,
				.descr_write_callback=descr1_write_handler,
				.is_notify = false
		},
		{		//1 Hearth rate Measurement Characteristic
				.char_uuid.len = ESP_UUID_LEN_16, // RX
				.char_uuid.uuid.uuid16 =  GATTS_CHAR_UUID_HR,
				.char_perm = ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
				.char_property = ESP_GATT_CHAR_PROP_BIT_READ | ESP_GATT_CHAR_PROP_BIT_WRITE | ESP_GATT_CHAR_PROP_BIT_NOTIFY,
				.char_val = &char2_HR1_val,
				.char_control = NULL,
				.char_handle = 0,
				.char_read_callback=char2_read_handler,
				.char_write_callback=char2_write_handler,
				.descr_uuid.len = ESP_UUID_LEN_16,
				.descr_uuid.uuid.uuid16 = ESP_GATT_UUID_CHAR_CLIENT_CONFIG,
				.descr_perm=ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
				.descr_val = &gatts_demo_descr1_val,
				.descr_control=NULL,
				.descr_handle=0,
				.descr_read_callback=descr2_read_handler,
				.descr_write_callback=descr2_write_handler,
				.is_notify = false
		},
		{		//2 PLX Spot-check measurement
				.char_uuid.len = ESP_UUID_LEN_16,  // TX
				.char_uuid.uuid.uuid16 =   GATTS_CHAR_UUID_PLX,
				.char_perm = ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
				.char_property = ESP_GATT_CHAR_PROP_BIT_READ | ESP_GATT_CHAR_PROP_BIT_WRITE | ESP_GATT_CHAR_PROP_BIT_NOTIFY,
				.char_val = &char3_PLX1_val,
				.char_control=NULL,
				.char_handle =0,
				.char_read_callback=char3_read_handler,
				.char_write_callback=char3_write_handler,
				.descr_uuid.len = ESP_UUID_LEN_16,
				.descr_uuid.uuid.uuid16 = ESP_GATT_UUID_CHAR_CLIENT_CONFIG, // ESP_GATT_UUID_CHAR_DESCRIPTION,
				.descr_perm=ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
				.descr_val = &gatts_demo_descr1_val,
				.descr_control=NULL,
				.descr_handle=0,
				.descr_read_callback=descr3_read_handler,
				.descr_write_callback=descr3_write_handler,
				.is_notify = false
		},
		{		//3 Body Location Characteristic
				.char_uuid.len = ESP_UUID_LEN_16,  // TX
				.char_uuid.uuid.uuid16 =  GATTS_CHAR_UUID_BODY_LOCATION,
				.char_perm = ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
				.char_property = ESP_GATT_CHAR_PROP_BIT_READ | ESP_GATT_CHAR_PROP_BIT_WRITE ,
				.char_val = &char4_BL2_val,
				.char_control=NULL,
				.char_handle =0,
				.char_read_callback=char4_read_handler,
				.char_write_callback=char4_write_handler,
				.descr_uuid.len = 0,
				.descr_uuid.uuid.uuid16 = ESP_GATT_UUID_CHAR_CLIENT_CONFIG, // ESP_GATT_UUID_CHAR_DESCRIPTION,
				.descr_perm=ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
				.descr_val = &gatts_demo_descr1_val,
				.descr_control=NULL,
				.descr_handle=0,
				.descr_read_callback=descr4_read_handler,
				.descr_write_callback=descr4_write_handler,
				.is_notify = false
		},
		{		//4 Hearth rate Measurement Characteristic
				.char_uuid.len = ESP_UUID_LEN_16, // RX
				.char_uuid.uuid.uuid16 =  GATTS_CHAR_UUID_HR,
				.char_perm = ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
				.char_property = ESP_GATT_CHAR_PROP_BIT_READ | ESP_GATT_CHAR_PROP_BIT_WRITE | ESP_GATT_CHAR_PROP_BIT_NOTIFY,
				.char_val = &char5_HR2_val,
				.char_control = NULL,
				.char_handle = 0,
				.char_read_callback=char5_read_handler,
				.char_write_callback=char5_write_handler,
				.descr_uuid.len = ESP_UUID_LEN_16,
				.descr_uuid.uuid.uuid16 = ESP_GATT_UUID_CHAR_CLIENT_CONFIG,
				.descr_perm=ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
				.descr_val = &gatts_demo_descr1_val,
				.descr_control=NULL,
				.descr_handle=0,
				.descr_read_callback=descr5_read_handler,
				.descr_write_callback=descr5_write_handler,
				.is_notify = false
		},
		{		//5 PLX Spot-check measurement
				.char_uuid.len = ESP_UUID_LEN_16,  // TX
				.char_uuid.uuid.uuid16 =   GATTS_CHAR_UUID_PLX,
				.char_perm = ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
				.char_property = ESP_GATT_CHAR_PROP_BIT_READ | ESP_GATT_CHAR_PROP_BIT_WRITE | ESP_GATT_CHAR_PROP_BIT_NOTIFY,
				.char_val = &char6_PLX2_val,
				.char_control=NULL,
				.char_handle =0,
				.char_read_callback=char6_read_handler,
				.char_write_callback=char6_write_handler,
				.descr_uuid.len = ESP_UUID_LEN_16,
				.descr_uuid.uuid.uuid16 = ESP_GATT_UUID_CHAR_CLIENT_CONFIG, // ESP_GATT_UUID_CHAR_DESCRIPTION,
				.descr_perm=ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
				.descr_val = &gatts_demo_descr1_val,
				.descr_control=NULL,
				.descr_handle=0,
				.descr_read_callback=descr6_read_handler,
				.descr_write_callback=descr6_write_handler,
				.is_notify = false
		},
		{		//6 HR RAW data
				.char_uuid.len = ESP_UUID_LEN_16,  // TX
				.char_uuid.uuid.uuid16 =   GATTS_CHAR_UUID_RAW,
				.char_perm = ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
				.char_property = ESP_GATT_CHAR_PROP_BIT_NOTIFY,//
				.char_val = &char7_RAW1_val,
				.char_control=NULL,
				.char_handle =0,
				.char_read_callback=char6_read_handler,
				.char_write_callback=char6_write_handler,
				.descr_uuid.len = ESP_UUID_LEN_16,
				.descr_uuid.uuid.uuid16 = ESP_GATT_UUID_CHAR_CLIENT_CONFIG, // ESP_GATT_UUID_CHAR_DESCRIPTION,
				.descr_perm=ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
				.descr_val = &gatts_demo_descr1_val,
				.descr_control=NULL,
				.descr_handle=0,
				.descr_read_callback=descr6_read_handler,
				.descr_write_callback=descr6_write_handler,
				.is_notify = false
		},
		{		//7 HR RAW data
				.char_uuid.len = ESP_UUID_LEN_16,  // TX
				.char_uuid.uuid.uuid16 =   GATTS_CHAR_UUID_RAW,
				.char_perm = ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
				.char_property = ESP_GATT_CHAR_PROP_BIT_NOTIFY,//
				.char_val = &char8_RAW2_val,
				.char_control=NULL,
				.char_handle =0,
				.char_read_callback=char6_read_handler,
				.char_write_callback=char6_write_handler,
				.descr_uuid.len = ESP_UUID_LEN_16,
				.descr_uuid.uuid.uuid16 = ESP_GATT_UUID_CHAR_CLIENT_CONFIG, // ESP_GATT_UUID_CHAR_DESCRIPTION,
				.descr_perm=ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
				.descr_val = &gatts_demo_descr1_val,
				.descr_control=NULL,
				.descr_handle=0,
				.descr_read_callback=descr6_read_handler,
				.descr_write_callback=descr6_write_handler,
				.is_notify = false
		},
		{		//8 Battery level
				.char_uuid.len = ESP_UUID_LEN_16,  // TX
				.char_uuid.uuid.uuid16 =   GATTS_CHAR_UUID_BAT,
				.char_perm = ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
				.char_property = ESP_GATT_CHAR_PROP_BIT_NOTIFY,//
				.char_val = &char9_BAT_val,
				.char_control=NULL,
				.char_handle =0,
				.char_read_callback=char6_read_handler,
				.char_write_callback=char6_write_handler,
				.descr_uuid.len = ESP_UUID_LEN_16,
				.descr_uuid.uuid.uuid16 = ESP_GATT_UUID_CHAR_CLIENT_CONFIG, // ESP_GATT_UUID_CHAR_DESCRIPTION,
				.descr_perm=ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
				.descr_val = &gatts_demo_descr1_val,
				.descr_control=NULL,
				.descr_handle=0,
				.descr_read_callback=descr6_read_handler,
				.descr_write_callback=descr6_write_handler,
				.is_notify = false
		}
};

void notify_task_optimized( void* arg) {
	notify_task_running = true;
	do{
		n_notify=0;
		uint8_t ch=0;
		for (int profile=0;profile<PROFILE_TOTAL_NUM;profile++){	//for each profile
			for (int j=0;j<gl_profile_tab[profile].char_num_total;j++,ch++){ //for each char in this profile
				if(gl_char[ch].is_notify){	//is char notifying
					n_notify++;
					switch	(gl_profile_tab[profile].sensor_id){
					case -1:
						if(gl_char[ch].char_uuid.uuid.uuid16 == GATTS_CHAR_UUID_RAW){ //if raw characteristic
							if(!sensor_have_finger[j]){ //if not detecting finger, do not notify
								printf("No finger on sensor %d\n",j);
								//continue; //do not notify
								//notify if 1st time
							}else{
								esp_ble_gatts_send_indicate(gl_profile_tab[profile].gatts_if, 0, gl_profile_tab[profile].char_handle[j],
										gl_char[ch].char_val->attr_len,gl_char[ch].char_val->attr_value , false);
								//printf("Notified this: \n");
								//print_array(gl_char[ch].char_val->attr_value ,gl_char[ch].char_val->attr_len/2);
								//realloc(j==0 ? raw_ptr0:raw_ptr1,0);
								//free(j==0 ? raw_ptr0:raw_ptr1_IR);
							}
						}else if(gl_char[ch].char_uuid.uuid.uuid16 == GATTS_CHAR_UUID_BAT){
							//verify if batery level changed and notify
							esp_ble_gatts_send_indicate(gl_profile_tab[profile].gatts_if, 0, gl_profile_tab[profile].char_handle[j],
									gl_char[ch].char_val->attr_len,gl_char[ch].char_val->attr_value , false);
						}
						break;
					default:
						if(sensor_have_finger[gl_profile_tab[profile].sensor_id]){
							if(gl_char[ch].char_uuid.uuid.uuid16 == GATTS_CHAR_UUID_HR){ //if heart rate
								if((gl_char[ch].char_val->attr_value[0] & SENSOR_CONTACT_DETECTED) == SENSOR_CONTACT_NOT_DETECTED){//SENSOR NOT DETECTED
									gl_char[ch].char_val->attr_value[0] |=  0b00000110;//todo corrigir isto
									esp_ble_gatts_send_indicate(gl_profile_tab[profile].gatts_if, 0, gl_profile_tab[profile].char_handle[j],
											gl_char[ch].char_val->attr_len,gl_char[ch].char_val->attr_value , false);
								}
							}
							esp_ble_gatts_send_indicate(gl_profile_tab[profile].gatts_if, 0, gl_profile_tab[profile].char_handle[j],
									gl_char[ch].char_val->attr_len,gl_char[ch].char_val->attr_value , false);
						}else{//if no finger
							if(gl_char[ch].char_uuid.uuid.uuid16 == GATTS_CHAR_UUID_HR){
								if((gl_char[ch].char_val->attr_value[0] & SENSOR_CONTACT_DETECTED) == SENSOR_CONTACT_DETECTED){//SENSOR DETECTED
									gl_char[ch].char_val->attr_value[0] &= 0b11111101; //set as sensor not detected
									esp_ble_gatts_send_indicate(gl_profile_tab[profile].gatts_if, 0, gl_profile_tab[profile].char_handle[j],
											gl_char[ch].char_val->attr_len,gl_char[ch].char_val->attr_value , false);
								}
							}
						}
						break;
					}
				}
			}
		}

		printf("Have %d chars to notify\n",n_notify);
		//vTaskDelay(2500 / portTICK_RATE_MS); // delay 1s
		vTaskSuspend(notify_TaskHandle);
	}while(n_notify>0);//there is any char to notify
	notify_task_running = false;
	vTaskDelete(NULL);
}

static uint8_t notify_pos=0;

void char1_read_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param) {
	uint8_t char_num = 0;
	ESP_LOGI(GATTS_TAG, "char%d_read_handler %d\n",char_num ,param->read.handle);
	esp_gatt_rsp_t rsp;
	memset(&rsp, 0, sizeof(esp_gatt_rsp_t));
	rsp.attr_value.handle = param->read.handle;
	if (gl_char[char_num].char_val!=NULL) {
		ESP_LOGI(GATTS_TAG, "char%d_read_handler char_val %d\n",char_num,gl_char[char_num].char_val->attr_len);
		rsp.attr_value.len = gl_char[char_num].char_val->attr_len;
		for (uint32_t pos=0;pos<gl_char[char_num].char_val->attr_len&&pos<gl_char[char_num].char_val->attr_max_len;pos++) {
			rsp.attr_value.value[pos] = gl_char[char_num].char_val->attr_value[pos];
		}
	}
	ESP_LOGI(GATTS_TAG, "char%d_read_handler esp_gatt_rsp_t\n",char_num);
	esp_ble_gatts_send_response(gatts_if, param->read.conn_id, param->read.trans_id, ESP_GATT_OK, &rsp);
}
void char2_read_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param) {
	uint8_t char_num = 1;
	ESP_LOGI(GATTS_TAG, "char%d_read_handler %d\n",char_num ,param->read.handle);
	esp_gatt_rsp_t rsp;
	memset(&rsp, 0, sizeof(esp_gatt_rsp_t));
	rsp.attr_value.handle = param->read.handle;
	if (gl_char[char_num].char_val!=NULL) {
		ESP_LOGI(GATTS_TAG, "char%d_read_handler char_val %d\n",char_num,gl_char[char_num].char_val->attr_len);
		rsp.attr_value.len = gl_char[char_num].char_val->attr_len;
		for (uint32_t pos=0;pos<gl_char[char_num].char_val->attr_len&&pos<gl_char[char_num].char_val->attr_max_len;pos++) {
			rsp.attr_value.value[pos] = gl_char[char_num].char_val->attr_value[pos];
		}
	}
	ESP_LOGI(GATTS_TAG, "char%d_read_handler esp_gatt_rsp_t\n",char_num);
	esp_ble_gatts_send_response(gatts_if, param->read.conn_id, param->read.trans_id, ESP_GATT_OK, &rsp);
}
void char3_read_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param) {
	uint8_t char_num = 2;
	ESP_LOGI(GATTS_TAG, "char%d_read_handler %d\n",char_num ,param->read.handle);
	esp_gatt_rsp_t rsp;
	memset(&rsp, 0, sizeof(esp_gatt_rsp_t));
	rsp.attr_value.handle = param->read.handle;
	if (gl_char[char_num].char_val!=NULL) {
		ESP_LOGI(GATTS_TAG, "char%d_read_handler char_val %d\n",char_num,gl_char[char_num].char_val->attr_len);
		rsp.attr_value.len = gl_char[char_num].char_val->attr_len;
		for (uint32_t pos=0;pos<gl_char[char_num].char_val->attr_len&&pos<gl_char[char_num].char_val->attr_max_len;pos++) {
			rsp.attr_value.value[pos] = gl_char[char_num].char_val->attr_value[pos];
		}
	}
	ESP_LOGI(GATTS_TAG, "char%d_read_handler esp_gatt_rsp_t\n",char_num);
	esp_ble_gatts_send_response(gatts_if, param->read.conn_id, param->read.trans_id, ESP_GATT_OK, &rsp);
}
void char4_read_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param) {
	uint8_t char_num = 3;
	ESP_LOGI(GATTS_TAG, "char%d_read_handler %d\n",char_num ,param->read.handle);
	esp_gatt_rsp_t rsp;
	memset(&rsp, 0, sizeof(esp_gatt_rsp_t));
	rsp.attr_value.handle = param->read.handle;
	if (gl_char[char_num].char_val!=NULL) {
		ESP_LOGI(GATTS_TAG, "char%d_read_handler char_val %d\n",char_num,gl_char[char_num].char_val->attr_len);
		rsp.attr_value.len = gl_char[char_num].char_val->attr_len;
		for (uint32_t pos=0;pos<gl_char[char_num].char_val->attr_len&&pos<gl_char[char_num].char_val->attr_max_len;pos++) {
			rsp.attr_value.value[pos] = gl_char[char_num].char_val->attr_value[pos];
		}
	}
	ESP_LOGI(GATTS_TAG, "char%d_read_handler esp_gatt_rsp_t\n",char_num);
	esp_ble_gatts_send_response(gatts_if, param->read.conn_id, param->read.trans_id, ESP_GATT_OK, &rsp);
}
void char5_read_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param) {
	uint8_t char_num = 4;
	ESP_LOGI(GATTS_TAG, "char%d_read_handler %d\n",char_num ,param->read.handle);
	esp_gatt_rsp_t rsp;
	memset(&rsp, 0, sizeof(esp_gatt_rsp_t));
	rsp.attr_value.handle = param->read.handle;
	if (gl_char[char_num].char_val!=NULL) {
		ESP_LOGI(GATTS_TAG, "char%d_read_handler char_val %d\n",char_num,gl_char[char_num].char_val->attr_len);
		rsp.attr_value.len = gl_char[char_num].char_val->attr_len;
		for (uint32_t pos=0;pos<gl_char[char_num].char_val->attr_len&&pos<gl_char[char_num].char_val->attr_max_len;pos++) {
			rsp.attr_value.value[pos] = gl_char[char_num].char_val->attr_value[pos];
		}
	}
	ESP_LOGI(GATTS_TAG, "char%d_read_handler esp_gatt_rsp_t\n",char_num);
	esp_ble_gatts_send_response(gatts_if, param->read.conn_id, param->read.trans_id, ESP_GATT_OK, &rsp);
}
void char6_read_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param) {
	uint8_t char_num = 5;
	ESP_LOGI(GATTS_TAG, "char%d_read_handler %d\n",char_num ,param->read.handle);
	esp_gatt_rsp_t rsp;
	memset(&rsp, 0, sizeof(esp_gatt_rsp_t));
	rsp.attr_value.handle = param->read.handle;
	if (gl_char[char_num].char_val!=NULL) {
		ESP_LOGI(GATTS_TAG, "char%d_read_handler char_val %d\n",char_num,gl_char[char_num].char_val->attr_len);
		rsp.attr_value.len = gl_char[char_num].char_val->attr_len;
		for (uint32_t pos=0;pos<gl_char[char_num].char_val->attr_len&&pos<gl_char[char_num].char_val->attr_max_len;pos++) {
			rsp.attr_value.value[pos] = gl_char[char_num].char_val->attr_value[pos];
		}
	}
	ESP_LOGI(GATTS_TAG, "char%d_read_handler esp_gatt_rsp_t\n",char_num);
	esp_ble_gatts_send_response(gatts_if, param->read.conn_id, param->read.trans_id, ESP_GATT_OK, &rsp);
}

void descr1_read_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param) {
	uint8_t char_num = 0;
	ESP_LOGI(GATTS_TAG, "descr%d_read_handler %d\n",char_num, param->read.handle);
	esp_gatt_rsp_t rsp;
	memset(&rsp, 0, sizeof(esp_gatt_rsp_t));
	rsp.attr_value.handle = param->read.handle;
	if (gl_char[char_num].descr_val!=NULL) {
		ESP_LOGI(GATTS_TAG, "descr%d_read_handler descr_val %d\n",char_num,gl_char[char_num].descr_val->attr_len);
		rsp.attr_value.len = gl_char[char_num].descr_val->attr_len;
		for (uint32_t pos=0;pos<gl_char[char_num].descr_val->attr_len&&pos<gl_char[char_num].descr_val->attr_max_len;pos++) {
			rsp.attr_value.value[pos] = gl_char[char_num].descr_val->attr_value[pos];
		}
	}
	ESP_LOGI(GATTS_TAG, "descr%d_read_handler esp_gatt_rsp_t \n",char_num);
	esp_ble_gatts_send_response(gatts_if, param->read.conn_id, param->read.trans_id,
			ESP_GATT_OK, &rsp);
}
void descr2_read_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param) {
	uint8_t char_num = 1;
	ESP_LOGI(GATTS_TAG, "descr%d_read_handler %d\n",char_num, param->read.handle);
	esp_gatt_rsp_t rsp;
	memset(&rsp, 0, sizeof(esp_gatt_rsp_t));
	rsp.attr_value.handle = param->read.handle;
	if (gl_char[char_num].descr_val!=NULL) {
		ESP_LOGI(GATTS_TAG, "descr%d_read_handler descr_val %d\n",char_num,gl_char[char_num].descr_val->attr_len);
		rsp.attr_value.len = gl_char[char_num].descr_val->attr_len;
		for (uint32_t pos=0;pos<gl_char[char_num].descr_val->attr_len&&pos<gl_char[char_num].descr_val->attr_max_len;pos++) {
			rsp.attr_value.value[pos] = gl_char[char_num].descr_val->attr_value[pos];
		}
	}
	ESP_LOGI(GATTS_TAG, "descr%d_read_handler esp_gatt_rsp_t \n",char_num);
	esp_ble_gatts_send_response(gatts_if, param->read.conn_id, param->read.trans_id,
			ESP_GATT_OK, &rsp);
}
void descr3_read_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param) {
	uint8_t char_num = 2;
	ESP_LOGI(GATTS_TAG, "descr%d_read_handler %d\n",char_num, param->read.handle);
	esp_gatt_rsp_t rsp;
	memset(&rsp, 0, sizeof(esp_gatt_rsp_t));
	rsp.attr_value.handle = param->read.handle;
	if (gl_char[char_num].descr_val!=NULL) {
		ESP_LOGI(GATTS_TAG, "descr%d_read_handler descr_val %d\n",char_num,gl_char[char_num].descr_val->attr_len);
		rsp.attr_value.len = gl_char[char_num].descr_val->attr_len;
		for (uint32_t pos=0;pos<gl_char[char_num].descr_val->attr_len&&pos<gl_char[char_num].descr_val->attr_max_len;pos++) {
			rsp.attr_value.value[pos] = gl_char[char_num].descr_val->attr_value[pos];
		}
	}
	ESP_LOGI(GATTS_TAG, "descr%d_read_handler esp_gatt_rsp_t \n",char_num);
	esp_ble_gatts_send_response(gatts_if, param->read.conn_id, param->read.trans_id,
			ESP_GATT_OK, &rsp);
}
void descr4_read_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param) {
	uint8_t char_num = 3;
	ESP_LOGI(GATTS_TAG, "descr%d_read_handler %d\n",char_num, param->read.handle);
	esp_gatt_rsp_t rsp;
	memset(&rsp, 0, sizeof(esp_gatt_rsp_t));
	rsp.attr_value.handle = param->read.handle;
	if (gl_char[char_num].descr_val!=NULL) {
		ESP_LOGI(GATTS_TAG, "descr%d_read_handler descr_val %d\n",char_num,gl_char[char_num].descr_val->attr_len);
		rsp.attr_value.len = gl_char[char_num].descr_val->attr_len;
		for (uint32_t pos=0;pos<gl_char[char_num].descr_val->attr_len&&pos<gl_char[char_num].descr_val->attr_max_len;pos++) {
			rsp.attr_value.value[pos] = gl_char[char_num].descr_val->attr_value[pos];
		}
	}
	ESP_LOGI(GATTS_TAG, "descr%d_read_handler esp_gatt_rsp_t \n",char_num);
	esp_ble_gatts_send_response(gatts_if, param->read.conn_id, param->read.trans_id,
			ESP_GATT_OK, &rsp);
}
void descr5_read_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param) {
	uint8_t char_num = 4;
	ESP_LOGI(GATTS_TAG, "descr%d_read_handler %d\n",char_num, param->read.handle);
	esp_gatt_rsp_t rsp;
	memset(&rsp, 0, sizeof(esp_gatt_rsp_t));
	rsp.attr_value.handle = param->read.handle;
	if (gl_char[char_num].descr_val!=NULL) {
		ESP_LOGI(GATTS_TAG, "descr%d_read_handler descr_val %d\n",char_num,gl_char[char_num].descr_val->attr_len);
		rsp.attr_value.len = gl_char[char_num].descr_val->attr_len;
		for (uint32_t pos=0;pos<gl_char[char_num].descr_val->attr_len&&pos<gl_char[char_num].descr_val->attr_max_len;pos++) {
			rsp.attr_value.value[pos] = gl_char[char_num].descr_val->attr_value[pos];
		}
	}
	ESP_LOGI(GATTS_TAG, "descr%d_read_handler esp_gatt_rsp_t \n",char_num);
	esp_ble_gatts_send_response(gatts_if, param->read.conn_id, param->read.trans_id,
			ESP_GATT_OK, &rsp);
}
void descr6_read_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param) {
	uint8_t char_num = 5;
	ESP_LOGI(GATTS_TAG, "descr%d_read_handler %d\n",char_num, param->read.handle);
	esp_gatt_rsp_t rsp;
	memset(&rsp, 0, sizeof(esp_gatt_rsp_t));
	rsp.attr_value.handle = param->read.handle;
	if (gl_char[char_num].descr_val!=NULL) {
		ESP_LOGI(GATTS_TAG, "descr%d_read_handler descr_val %d\n",char_num,gl_char[char_num].descr_val->attr_len);
		rsp.attr_value.len = gl_char[char_num].descr_val->attr_len;
		for (uint32_t pos=0;pos<gl_char[char_num].descr_val->attr_len&&pos<gl_char[char_num].descr_val->attr_max_len;pos++) {
			rsp.attr_value.value[pos] = gl_char[char_num].descr_val->attr_value[pos];
		}
	}
	ESP_LOGI(GATTS_TAG, "descr%d_read_handler esp_gatt_rsp_t \n",char_num);
	esp_ble_gatts_send_response(gatts_if, param->read.conn_id, param->read.trans_id,
			ESP_GATT_OK, &rsp);
}

void char1_notify_handle(esp_gatt_if_t gatts_if, uint16_t conn_id) {
	uint8_t char_num = 0;
	ESP_LOGI(GATTS_TAG, "char%d_notify_handler \n",char_num);
	if (gl_char[char_num].is_notify==1) {
		notify_pos='0';
		for (uint32_t i=0;i<5;i++) {
			ESP_LOGI(GATTS_TAG, "char%d_notify_handle esp_ble_gatts_send_indicate\n",char_num);
			vTaskDelay(100 / portTICK_RATE_MS); // delay 1s
			esp_ble_gatts_send_indicate(gatts_if, conn_id, gl_char[char_num].char_handle,1,&notify_pos,false);
			notify_pos++;
		}
	}
}
void char2_notify_handle(esp_gatt_if_t gatts_if, uint16_t conn_id) {
	uint8_t char_num = 1;
	ESP_LOGI(GATTS_TAG, "char%d_notify_handler \n",char_num);
	if (gl_char[char_num].is_notify==1) {
		notify_pos='0';
		/*for (uint32_t i=0;i<10;i++) {
			ESP_LOGI(GATTS_TAG, "char%d_notify_handle esp_ble_gatts_send_indicate\n",char_num);
			vTaskDelay(1000 / portTICK_RATE_MS); // delay 1s
			esp_ble_gatts_send_indicate(gatts_if, conn_id, gl_char[char_num].char_handle,1,&notify_pos,false);
			notify_pos++;
		}*/
	}
}
void char3_notify_handle(esp_gatt_if_t gatts_if, uint16_t conn_id) {
	uint8_t char_num = 2;
	ESP_LOGI(GATTS_TAG, "char%d_notify_handler \n",char_num);
	if (gl_char[char_num].is_notify==1) {
		notify_pos='0';
		for (uint32_t i=0;i<10;i++) {
			ESP_LOGI(GATTS_TAG, "char%d_notify_handle esp_ble_gatts_send_indicate\n",char_num);
			vTaskDelay(1000 / portTICK_RATE_MS); // delay 1s
			esp_ble_gatts_send_indicate(gatts_if, conn_id, gl_char[char_num].char_handle,1,&notify_pos,false);
			notify_pos++;
		}
	}
}
void char4_notify_handle(esp_gatt_if_t gatts_if, uint16_t conn_id) {
	uint8_t char_num = 3;
	ESP_LOGI(GATTS_TAG, "char%d_notify_handler \n",char_num);
	if (gl_char[char_num].is_notify==1) {
		notify_pos='0';
		for (uint32_t i=0;i<10;i++) {
			ESP_LOGI(GATTS_TAG, "char%d_notify_handle esp_ble_gatts_send_indicate\n",char_num);
			vTaskDelay(1000 / portTICK_RATE_MS); // delay 1s
			esp_ble_gatts_send_indicate(gatts_if, conn_id, gl_char[char_num].char_handle,1,&notify_pos,false);
			notify_pos++;
		}
	}
}
void char5_notify_handle(esp_gatt_if_t gatts_if, uint16_t conn_id) {
	uint8_t char_num = 4;
	ESP_LOGI(GATTS_TAG, "char%d_notify_handler \n",char_num);
	if (gl_char[char_num].is_notify==1) {
		notify_pos='0';
		for (uint32_t i=0;i<10;i++) {
			ESP_LOGI(GATTS_TAG, "char%d_notify_handle esp_ble_gatts_send_indicate\n",char_num);
			vTaskDelay(1000 / portTICK_RATE_MS); // delay 1s
			esp_ble_gatts_send_indicate(gatts_if, conn_id, gl_char[char_num].char_handle,1,&notify_pos,false);
			notify_pos++;
		}
	}
}
void char6_notify_handle(esp_gatt_if_t gatts_if, uint16_t conn_id) {
	uint8_t char_num = 5;
	ESP_LOGI(GATTS_TAG, "char%d_notify_handler \n",char_num);
	if (gl_char[char_num].is_notify==1) {
		notify_pos='0';
		for (uint32_t i=0;i<10;i++) {
			ESP_LOGI(GATTS_TAG, "char%d_notify_handle esp_ble_gatts_send_indicate\n",char_num);
			vTaskDelay(1000 / portTICK_RATE_MS); // delay 1s
			esp_ble_gatts_send_indicate(gatts_if, conn_id, gl_char[char_num].char_handle,1,&notify_pos,false);
			notify_pos++;
		}
	}
}

void char1_write_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param) {
	uint8_t char_num = 0;
	ESP_LOGI(GATTS_TAG, "char%d_write_handler %d\n", char_num, param->write.handle);
	if (gl_char[char_num].char_val!=NULL) {
		ESP_LOGI(GATTS_TAG, "char%d_write_handler char_val %d\n",char_num,param->write.len);
		gl_char[char_num].char_val->attr_len = param->write.len;
		for (uint32_t pos=0;pos<param->write.len;pos++) {
			gl_char[char_num].char_val->attr_value[pos]=param->write.value[pos];
		}
		ESP_LOGI(GATTS_TAG, "char%d_write_handler %.*s",char_num ,gl_char[char_num].char_val->attr_len, (char*)gl_char[char_num].char_val->attr_value);
	}
	ESP_LOGI(GATTS_TAG, "char%d_write_handler esp_gatt_rsp_t\n",char_num);
	esp_ble_gatts_send_response(gatts_if, param->write.conn_id, param->write.trans_id, ESP_GATT_OK, NULL);
	if (strncmp((const char *)gl_char[char_num].char_val->attr_value,"LED ON",6)==0) {
	} else if (strncmp((const char *)gl_char[char_num].char_val->attr_value,"LED OFF",7)==0) {
	} else if (strncmp((const char *)gl_char[char_num].char_val->attr_value,"LED SWITCH",10)==0) {
	} else {
		char1_notify_handle(gatts_if, param->write.conn_id);
	}
}
void char2_write_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param) {
	uint8_t char_num = 1;
	ESP_LOGI(GATTS_TAG, "char%d_write_handler %d\n", char_num, param->write.handle);
	if (gl_char[char_num].char_val!=NULL) {
		ESP_LOGI(GATTS_TAG, "char%d_write_handler char_val %d\n",char_num,param->write.len);
		gl_char[char_num].char_val->attr_len = param->write.len;
		for (uint32_t pos=0;pos<param->write.len;pos++) {
			gl_char[char_num].char_val->attr_value[pos]=param->write.value[pos];
		}
		ESP_LOGI(GATTS_TAG, "char%d_write_handler %.*s", char_num,gl_char[char_num].char_val->attr_len, (char*)gl_char[char_num].char_val->attr_value);
	}
	ESP_LOGI(GATTS_TAG, "char%d_write_handler esp_gatt_rsp_t\n",char_num);
	esp_ble_gatts_send_response(gatts_if, param->write.conn_id, param->write.trans_id, ESP_GATT_OK, NULL);
	if (strncmp((const char *)gl_char[char_num].char_val->attr_value,"LED ON",6)==0) {
	} else if (strncmp((const char *)gl_char[char_num].char_val->attr_value,"LED OFF",7)==0) {
	} else if (strncmp((const char *)gl_char[char_num].char_val->attr_value,"LED SWITCH",10)==0) {
	} else {
		char2_notify_handle(gatts_if, param->write.conn_id);
	}
}
void char3_write_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param) {
	uint8_t char_num = 2;
	ESP_LOGI(GATTS_TAG, "char%d_write_handler %d\n", char_num, param->write.handle);
	if (gl_char[char_num].char_val!=NULL) {
		ESP_LOGI(GATTS_TAG, "char%d_write_handler char_val %d\n",char_num,param->write.len);
		gl_char[char_num].char_val->attr_len = param->write.len;
		for (uint32_t pos=0;pos<param->write.len;pos++) {
			gl_char[char_num].char_val->attr_value[pos]=param->write.value[pos];
		}
		ESP_LOGI(GATTS_TAG, "char%d_write_handler %.*s",char_num, gl_char[char_num].char_val->attr_len, (char*)gl_char[char_num].char_val->attr_value);
	}
	ESP_LOGI(GATTS_TAG, "char%d_write_handler esp_gatt_rsp_t\n",char_num);
	esp_ble_gatts_send_response(gatts_if, param->write.conn_id, param->write.trans_id, ESP_GATT_OK, NULL);
	if (strncmp((const char *)gl_char[char_num].char_val->attr_value,"LED ON",6)==0) {
	} else if (strncmp((const char *)gl_char[char_num].char_val->attr_value,"LED OFF",7)==0) {
	} else if (strncmp((const char *)gl_char[char_num].char_val->attr_value,"LED SWITCH",10)==0) {
	} else {
		char3_notify_handle(gatts_if, param->write.conn_id);
	}
}
void char4_write_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param) {
	uint8_t char_num = 3;
	ESP_LOGI(GATTS_TAG, "char%d_write_handler %d\n", char_num, param->write.handle);
	if (gl_char[char_num].char_val!=NULL) {
		ESP_LOGI(GATTS_TAG, "char%d_write_handler char_val %d\n",char_num,param->write.len);
		gl_char[char_num].char_val->attr_len = param->write.len;
		for (uint32_t pos=0;pos<param->write.len;pos++) {
			gl_char[char_num].char_val->attr_value[pos]=param->write.value[pos];
		}
		ESP_LOGI(GATTS_TAG, "char%d_write_handler %.*s", char_num,gl_char[char_num].char_val->attr_len, (char*)gl_char[char_num].char_val->attr_value);
	}
	ESP_LOGI(GATTS_TAG, "char%d_write_handler esp_gatt_rsp_t\n",char_num);
	esp_ble_gatts_send_response(gatts_if, param->write.conn_id, param->write.trans_id, ESP_GATT_OK, NULL);
	if (strncmp((const char *)gl_char[char_num].char_val->attr_value,"LED ON",6)==0) {
	} else if (strncmp((const char *)gl_char[char_num].char_val->attr_value,"LED OFF",7)==0) {
	} else if (strncmp((const char *)gl_char[char_num].char_val->attr_value,"LED SWITCH",10)==0) {
	} else {
		char4_notify_handle(gatts_if, param->write.conn_id);
	}
}
void char5_write_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param) {
	uint8_t char_num = 4;
	ESP_LOGI(GATTS_TAG, "char%d_write_handler %d\n", char_num, param->write.handle);
	if (gl_char[char_num].char_val!=NULL) {
		ESP_LOGI(GATTS_TAG, "char%d_write_handler char_val %d\n",char_num,param->write.len);
		gl_char[char_num].char_val->attr_len = param->write.len;
		for (uint32_t pos=0;pos<param->write.len;pos++) {
			gl_char[char_num].char_val->attr_value[pos]=param->write.value[pos];
		}
		ESP_LOGI(GATTS_TAG, "char%d_write_handler %.*s",char_num ,gl_char[char_num].char_val->attr_len, (char*)gl_char[char_num].char_val->attr_value);
	}
	ESP_LOGI(GATTS_TAG, "char%d_write_handler esp_gatt_rsp_t\n",char_num);
	esp_ble_gatts_send_response(gatts_if, param->write.conn_id, param->write.trans_id, ESP_GATT_OK, NULL);
	if (strncmp((const char *)gl_char[char_num].char_val->attr_value,"LED ON",6)==0) {
	} else if (strncmp((const char *)gl_char[char_num].char_val->attr_value,"LED OFF",7)==0) {
	} else if (strncmp((const char *)gl_char[char_num].char_val->attr_value,"LED SWITCH",10)==0) {
	} else {
		char5_notify_handle(gatts_if, param->write.conn_id);
	}
}
void char6_write_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param) {
	uint8_t char_num = 5;
	ESP_LOGI(GATTS_TAG, "char%d_write_handler %d\n", char_num, param->write.handle);
	if (gl_char[char_num].char_val!=NULL) {
		ESP_LOGI(GATTS_TAG, "char%d_write_handler char_val %d\n",char_num,param->write.len);
		gl_char[char_num].char_val->attr_len = param->write.len;
		for (uint32_t pos=0;pos<param->write.len;pos++) {
			gl_char[char_num].char_val->attr_value[pos]=param->write.value[pos];
		}
		ESP_LOGI(GATTS_TAG, "char%d_write_handler %.*s",char_num ,gl_char[char_num].char_val->attr_len, (char*)gl_char[char_num].char_val->attr_value);
	}
	ESP_LOGI(GATTS_TAG, "char%d_write_handler esp_gatt_rsp_t\n",char_num);
	esp_ble_gatts_send_response(gatts_if, param->write.conn_id, param->write.trans_id, ESP_GATT_OK, NULL);
	if (strncmp((const char *)gl_char[char_num].char_val->attr_value,"LED ON",6)==0) {
	} else if (strncmp((const char *)gl_char[char_num].char_val->attr_value,"LED OFF",7)==0) {
	} else if (strncmp((const char *)gl_char[char_num].char_val->attr_value,"LED SWITCH",10)==0) {
	} else {
		char6_notify_handle(gatts_if, param->write.conn_id);
	}
}

void descr1_write_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param) {
	uint8_t char_num = 0;
	ESP_LOGI(GATTS_TAG, "descr%d_write_handler %d\n",char_num,param->write.handle);

	if (gl_char[char_num].descr_val!=NULL) {
		ESP_LOGI(GATTS_TAG, "descr%d_write_handler descr_val %d\n",char_num,param->write.len);
		gl_char[char_num].descr_val->attr_len = param->write.len;
		for (uint32_t pos=0;pos<param->write.len;pos++) {
			gl_char[char_num].descr_val->attr_value[pos]=param->write.value[pos];
		}
	}
	ESP_LOGI(GATTS_TAG, "descr%d_write_handler esp_gatt_rsp_t\n",char_num);
	esp_ble_gatts_send_response(gatts_if, param->write.conn_id, param->write.trans_id, ESP_GATT_OK, NULL);
}
void descr2_write_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param) {
	uint8_t char_num = 1;
	ESP_LOGI(GATTS_TAG, "descr%d_write_handler %d\n",char_num,param->write.handle);

	if (gl_char[char_num].descr_val!=NULL) {
		ESP_LOGI(GATTS_TAG, "descr%d_write_handler descr_val %d\n",char_num,param->write.len);
		gl_char[char_num].descr_val->attr_len = param->write.len;
		for (uint32_t pos=0;pos<param->write.len;pos++) {
			gl_char[char_num].descr_val->attr_value[pos]=param->write.value[pos];
		}
	}
	ESP_LOGI(GATTS_TAG, "descr%d_write_handler esp_gatt_rsp_t\n",char_num);
	esp_ble_gatts_send_response(gatts_if, param->write.conn_id, param->write.trans_id, ESP_GATT_OK, NULL);
}
void descr3_write_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param) {
	uint8_t char_num = 2;
	ESP_LOGI(GATTS_TAG, "descr%d_write_handler %d\n",char_num,param->write.handle);

	if (gl_char[char_num].descr_val!=NULL) {
		ESP_LOGI(GATTS_TAG, "descr%d_write_handler descr_val %d\n",char_num,param->write.len);
		gl_char[char_num].descr_val->attr_len = param->write.len;
		for (uint32_t pos=0;pos<param->write.len;pos++) {
			gl_char[char_num].descr_val->attr_value[pos]=param->write.value[pos];
		}
	}
	ESP_LOGI(GATTS_TAG, "descr%d_write_handler esp_gatt_rsp_t\n",char_num);
	esp_ble_gatts_send_response(gatts_if, param->write.conn_id, param->write.trans_id, ESP_GATT_OK, NULL);
}
void descr4_write_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param) {
	uint8_t char_num = 3;
	ESP_LOGI(GATTS_TAG, "descr%d_write_handler %d\n",char_num,param->write.handle);

	if (gl_char[char_num].descr_val!=NULL) {
		ESP_LOGI(GATTS_TAG, "descr%d_write_handler descr_val %d\n",char_num,param->write.len);
		gl_char[char_num].descr_val->attr_len = param->write.len;
		for (uint32_t pos=0;pos<param->write.len;pos++) {
			gl_char[char_num].descr_val->attr_value[pos]=param->write.value[pos];
		}
	}
	ESP_LOGI(GATTS_TAG, "descr%d_write_handler esp_gatt_rsp_t\n",char_num);
	esp_ble_gatts_send_response(gatts_if, param->write.conn_id, param->write.trans_id, ESP_GATT_OK, NULL);
}
void descr5_write_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param) {
	uint8_t char_num = 4;
	ESP_LOGI(GATTS_TAG, "descr%d_write_handler %d\n",char_num,param->write.handle);

	if (gl_char[char_num].descr_val!=NULL) {
		ESP_LOGI(GATTS_TAG, "descr%d_write_handler descr_val %d\n",char_num,param->write.len);
		gl_char[char_num].descr_val->attr_len = param->write.len;
		for (uint32_t pos=0;pos<param->write.len;pos++) {
			gl_char[char_num].descr_val->attr_value[pos]=param->write.value[pos];
		}
	}
	ESP_LOGI(GATTS_TAG, "descr%d_write_handler esp_gatt_rsp_t\n",char_num);
	esp_ble_gatts_send_response(gatts_if, param->write.conn_id, param->write.trans_id, ESP_GATT_OK, NULL);
}
void descr6_write_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param) {
	uint8_t char_num = 5;
	ESP_LOGI(GATTS_TAG, "descr%d_write_handler %d\n",char_num,param->write.handle);

	if (gl_char[char_num].descr_val!=NULL) {
		ESP_LOGI(GATTS_TAG, "descr%d_write_handler descr_val %d\n",char_num,param->write.len);
		gl_char[char_num].descr_val->attr_len = param->write.len;
		for (uint32_t pos=0;pos<param->write.len;pos++) {
			gl_char[char_num].descr_val->attr_value[pos]=param->write.value[pos];
		}
	}
	ESP_LOGI(GATTS_TAG, "descr%d_write_handler esp_gatt_rsp_t\n",char_num);
	esp_ble_gatts_send_response(gatts_if, param->write.conn_id, param->write.trans_id, ESP_GATT_OK, NULL);
}

/* One gatt-based profile one app_id and one gatts_if, this array will store the gatts_if returned by ESP_GATTS_REG_EVT */



typedef struct {
	uint8_t                 *prepare_buf;
	int                     prepare_len;
} prepare_type_env_t;

static prepare_type_env_t a_prepare_write_env,b_prepare_write_env,c_prepare_write_env,d_prepare_write_env;

void example_write_event_env(esp_gatt_if_t gatts_if, prepare_type_env_t *prepare_write_env, esp_ble_gatts_cb_param_t *param);
void example_exec_write_event_env(prepare_type_env_t *prepare_write_env, esp_ble_gatts_cb_param_t *param);

static void gap_event_handler(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param)
{
	switch (event) {
#ifdef CONFIG_SET_RAW_ADV_DATA
	case ESP_GAP_BLE_ADV_DATA_RAW_SET_COMPLETE_EVT:
		adv_config_done &= (~adv_config_flag);
		if (adv_config_done==0){
			esp_ble_gap_start_advertising(&adv_params);
		}
		break;
	case ESP_GAP_BLE_SCAN_RSP_DATA_RAW_SET_COMPLETE_EVT:
		adv_config_done &= (~scan_rsp_config_flag);
		if (adv_config_done==0){
			esp_ble_gap_start_advertising(&adv_params);
		}
		break;
#else
	case ESP_GAP_BLE_ADV_DATA_SET_COMPLETE_EVT:
		adv_config_done &= (~adv_config_flag);
		if (adv_config_done == 0){
			esp_ble_gap_start_advertising(&adv_params);
		}
		break;
	case ESP_GAP_BLE_SCAN_RSP_DATA_SET_COMPLETE_EVT:
		adv_config_done &= (~scan_rsp_config_flag);
		if (adv_config_done == 0){
			esp_ble_gap_start_advertising(&adv_params);
		}
		break;
#endif
	case ESP_GAP_BLE_ADV_START_COMPLETE_EVT:
		//advertising start complete event to indicate advertising start successfully or failed
		if (param->adv_start_cmpl.status != ESP_BT_STATUS_SUCCESS) {
			ESP_LOGE(GATTS_TAG, "Advertising start failed\n");
		}
		break;
	case ESP_GAP_BLE_ADV_STOP_COMPLETE_EVT:
		if (param->adv_stop_cmpl.status != ESP_BT_STATUS_SUCCESS) {
			ESP_LOGE(GATTS_TAG, "Advertising stop failed\n");
		}
		else {
			ESP_LOGI(GATTS_TAG, "Stop adv successfully\n");
		}
		break;
	case ESP_GAP_BLE_UPDATE_CONN_PARAMS_EVT:
		ESP_LOGI(GATTS_TAG, "update connection params status = %d, min_int = %d, max_int = %d,conn_int = %d,latency = %d, timeout = %d",
				param->update_conn_params.status,
				param->update_conn_params.min_int,
				param->update_conn_params.max_int,
				param->update_conn_params.conn_int,
				param->update_conn_params.latency,
				param->update_conn_params.timeout);
		break;
	default:
		break;
	}
}

void example_write_event_env(esp_gatt_if_t gatts_if, prepare_type_env_t *prepare_write_env, esp_ble_gatts_cb_param_t *param){
	esp_gatt_status_t status = ESP_GATT_OK;
	if (param->write.need_rsp){
		if (param->write.is_prep){
			if (prepare_write_env->prepare_buf == NULL) {
				prepare_write_env->prepare_buf = (uint8_t *)malloc(PREPARE_BUF_MAX_SIZE*sizeof(uint8_t));
				prepare_write_env->prepare_len = 0;
				if (prepare_write_env->prepare_buf == NULL) {
					ESP_LOGE(GATTS_TAG, "Gatt_server prep no mem\n");
					status = ESP_GATT_NO_RESOURCES;
				}
			} else {
				if(param->write.offset > PREPARE_BUF_MAX_SIZE) {
					status = ESP_GATT_INVALID_OFFSET;
				} else if ((param->write.offset + param->write.len) > PREPARE_BUF_MAX_SIZE) {
					status = ESP_GATT_INVALID_ATTR_LEN;
				}
			}

			esp_gatt_rsp_t *gatt_rsp = (esp_gatt_rsp_t *)malloc(sizeof(esp_gatt_rsp_t));
			gatt_rsp->attr_value.len = param->write.len;
			gatt_rsp->attr_value.handle = param->write.handle;
			gatt_rsp->attr_value.offset = param->write.offset;
			gatt_rsp->attr_value.auth_req = ESP_GATT_AUTH_REQ_NONE;
			memcpy(gatt_rsp->attr_value.value, param->write.value, param->write.len);
			esp_err_t response_err = esp_ble_gatts_send_response(gatts_if, param->write.conn_id, param->write.trans_id, status, gatt_rsp);
			if (response_err != ESP_OK){
				ESP_LOGE(GATTS_TAG, "Send response error\n");
			}
			free(gatt_rsp);
			if (status != ESP_GATT_OK){
				return;
			}
			memcpy(prepare_write_env->prepare_buf + param->write.offset,
					param->write.value,
					param->write.len);
			prepare_write_env->prepare_len += param->write.len;

		}else{
			esp_ble_gatts_send_response(gatts_if, param->write.conn_id, param->write.trans_id, status, NULL);
		}
	}
}

void example_exec_write_event_env(prepare_type_env_t *prepare_write_env, esp_ble_gatts_cb_param_t *param){
	if (param->exec_write.exec_write_flag == ESP_GATT_PREP_WRITE_EXEC){
		esp_log_buffer_hex(GATTS_TAG, prepare_write_env->prepare_buf, prepare_write_env->prepare_len);
	}else{
		ESP_LOGI(GATTS_TAG,"ESP_GATT_PREP_WRITE_CANCEL");
	}
	if (prepare_write_env->prepare_buf) {
		free(prepare_write_env->prepare_buf);
		prepare_write_env->prepare_buf = NULL;
	}
	prepare_write_env->prepare_len = 0;
}

void gatts_add_char(uint8_t profile) {

	ESP_LOGI(GATTS_TAG, "gatts_add_char %d in profile %d\n", gl_profile_tab[profile].char_num, profile);

	for (uint32_t pos=0;pos<GATTS_CHAR_NUM_TOTAL;pos++) {
		if (gl_char[pos].char_handle==0) {
			ble_add_char_pos=pos;
			//ESP_LOGI(GATTS_TAG, "ADD pos %d handle %d service %d\n", pos,gl_char[pos].char_handle,gl_profile_tab[profile].service_handle);
			esp_ble_gatts_add_char(gl_profile_tab[profile].service_handle, &gl_char[pos].char_uuid,gl_char[pos].char_perm,gl_char[pos].char_property,gl_char[pos].char_val, gl_char[pos].char_control);
			gl_profile_tab[profile].char_num++;
			break;
		}
	}
}

void gatts_check_add_char(esp_bt_uuid_t char_uuid, uint16_t attr_handle,uint8_t profile) {
	ESP_LOGI(GATTS_TAG, "gatts_check_add_char %d\n", attr_handle);
	if (attr_handle != 0) {
		if (char_uuid.len == ESP_UUID_LEN_16) {
			ESP_LOGI(GATTS_TAG, "Char UUID16: %x", char_uuid.uuid.uuid16);
		} else if (char_uuid.len == ESP_UUID_LEN_32) {
			ESP_LOGI(GATTS_TAG, "Char UUID32: %x", char_uuid.uuid.uuid32);
		} else if (char_uuid.len == ESP_UUID_LEN_128) {
			ESP_LOGI(GATTS_TAG, "Char UUID128: %x,%x,%x,%x,%x,%x,%x,%x,%x,%x,%x,%x,%x,%x,%x,%x", char_uuid.uuid.uuid128[0],
					char_uuid.uuid.uuid128[1], char_uuid.uuid.uuid128[2], char_uuid.uuid.uuid128[3],
					char_uuid.uuid.uuid128[4], char_uuid.uuid.uuid128[5], char_uuid.uuid.uuid128[6],
					char_uuid.uuid.uuid128[7], char_uuid.uuid.uuid128[8], char_uuid.uuid.uuid128[9],
					char_uuid.uuid.uuid128[10], char_uuid.uuid.uuid128[11], char_uuid.uuid.uuid128[12],
					char_uuid.uuid.uuid128[13], char_uuid.uuid.uuid128[14], char_uuid.uuid.uuid128[15]);
		} else {
			ESP_LOGE(GATTS_TAG, "Char UNKNOWN LEN %d\n", char_uuid.len);
		}

		ESP_LOGI(GATTS_TAG, "FOUND Char pos %d handle %d\n", ble_add_char_pos,attr_handle);
		gl_char[ble_add_char_pos].char_handle=attr_handle;


		// is there a descriptor to add ?
		if (gl_char[ble_add_char_pos].descr_uuid.len!=0 && gl_char[ble_add_char_pos].descr_handle==0) {
			ESP_LOGI(GATTS_TAG, "ADD Descr pos %d handle %d service %d\n", ble_add_char_pos,gl_char[ble_add_char_pos].descr_handle,gl_profile_tab[profile].service_handle);
			esp_ble_gatts_add_char_descr(gl_profile_tab[profile].service_handle, &gl_char[ble_add_char_pos].descr_uuid,
					gl_char[ble_add_char_pos].descr_perm, gl_char[ble_add_char_pos].descr_val, gl_char[ble_add_char_pos].descr_control);
		} else {
			gatts_add_char(profile);
		}
	}
}

void gatts_check_callback(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param) {
	uint16_t handle=0;
	uint8_t read=1;

	switch (event) {
	case ESP_GATTS_READ_EVT: {
		read=1;
		handle=param->read.handle;
		break;
	}
	case ESP_GATTS_WRITE_EVT: {
		read=0;
		handle=param->write.handle;
		break;
	}
	default:
		break;
	}

	ESP_LOGI(GATTS_TAG, "gatts_check_callback read %d num %d handle %d\n", read, GATTS_CHAR_NUM_TOTAL, handle);
	for (uint32_t pos=0;pos<GATTS_CHAR_NUM_TOTAL;pos++) {
		//ESP_LOGI(GATTS_TAG, "gatts_check_callback (%d) in pos %d \n\t\tLooking for handle %d\n", gl_char[pos].char_handle,pos, handle);
		if (gl_char[pos].char_handle==handle) {
			if (read==1) {
				if (gl_char[pos].char_read_callback!=NULL) {
					gl_char[pos].char_read_callback(event, gatts_if, param);
				}
			} else {
				if (gl_char[pos].char_write_callback!=NULL) {
					gl_char[pos].char_write_callback(event, gatts_if, param);
				}
			}
			break;
		}
		//ESP_LOGI(GATTS_TAG, "descr_handle (%d) in pos %d \n\t\tLooking for handle %d\n", gl_char[pos].descr_handle ,pos, handle);
		if (gl_char[pos].char_handle+1==handle) {
			if (read==1) {
				if (gl_char[pos].descr_read_callback!=NULL) {
					gl_char[pos].descr_read_callback(event, gatts_if, param);
				}
			} else {
				if (gl_char[pos].descr_write_callback!=NULL) {
					gl_char[pos].descr_write_callback(event, gatts_if, param);
				}
			}
			break;
		}
	}
}

static void gatts_profile_a_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param) {
	uint8_t profile = PROFILE_HR1_APP_ID;
	switch (event) {
	case ESP_GATTS_REG_EVT:

		ESP_LOGI(GATTS_TAG, "REGISTER_APP_EVT, status %d, app_id %d\n", param->reg.status, param->reg.app_id);
		gl_profile_tab[profile].service_id.is_primary = true;
		gl_profile_tab[profile].service_id.id.inst_id = 0x00;
		gl_profile_tab[profile].service_id.id.uuid.len = ESP_UUID_LEN_16;
		gl_profile_tab[profile].service_id.id.uuid.uuid.uuid16 = GATTS_SERVICE_UUID_HEART_RATE;

		esp_ble_gatts_create_service(gatts_if, &gl_profile_tab[profile].service_id, GATTS_NUM_HANDLE_HR);

		esp_err_t set_dev_name_ret = esp_ble_gap_set_device_name(TEST_DEVICE_NAME);
		if (set_dev_name_ret){
			ESP_LOGE(GATTS_TAG, "set device name failed, error code = %x", set_dev_name_ret);
		}

		//config adv data
		esp_err_t ret = esp_ble_gap_config_adv_data(&adv_data);
		if (ret){
			ESP_LOGE(GATTS_TAG, "config adv data failed, error code = %x", ret);
		}
		adv_config_done |= adv_config_flag;
		//config scan response data
		ret = esp_ble_gap_config_adv_data(&scan_rsp_data);
		if (ret){
			ESP_LOGE(GATTS_TAG, "config scan response data failed, error code = %x", ret);
		}
		adv_config_done |= scan_rsp_config_flag;

		break;

	case ESP_GATTS_READ_EVT: {
		ESP_LOGI(GATTS_TAG, "GATT_READ_EVT, conn_id %d, trans_id %d, handle %d\n", param->read.conn_id, param->read.trans_id, param->read.handle);
		/*esp_gatt_rsp_t rsp;
        memset(&rsp, 0, sizeof(esp_gatt_rsp_t));
        rsp.attr_value.len = 4;
        rsp.attr_value.value[0] = 0x11;
        rsp.attr_value.value[1] = 0x11;
        rsp.attr_value.value[2] = 0xbe;
        rsp.attr_value.value[3] = 0xef;
        esp_ble_gatts_send_response(gatts_if, param->read.conn_id, param->read.trans_id,
                                    ESP_GATT_OK, &rsp);*/
		gatts_check_callback(event, gatts_if, param);
		break;
	}
	case ESP_GATTS_WRITE_EVT: {
		ESP_LOGI(GATTS_TAG, "GATT_WRITE_EVT, conn_id %d, trans_id %d, handle %d", param->write.conn_id, param->write.trans_id, param->write.handle);
		ESP_LOGI(GATTS_TAG, "GATT_WRITE_EVT, value len %d, value %08x\n", param->write.len, *(uint32_t *)param->write.value);
		if (!param->write.is_prep){
			esp_log_buffer_hex(GATTS_TAG, param->write.value, param->write.len);

			if (gl_profile_tab[profile].descr_handle == param->write.handle && param->write.len == 2){
				uint16_t descr_value = param->write.value[1]<<8 | param->write.value[0];
				if (descr_value == 0x0001){
					if (ESP_GATT_CHAR_PROP_BIT_NOTIFY){
						ESP_LOGI(GATTS_TAG, "notify enable");
						gl_char[1].is_notify = true;	//notify flag
#ifdef EN_NOTIFY
						if (!notify_task_running){
							xTaskCreate(notify_task_optimized, "notify_task_optimized", 1024 * 4, (void*) 0, 10, &notify_TaskHandle);
						}
#endif
						esp_ble_gatts_send_indicate(gatts_if, param->write.conn_id, gl_profile_tab[profile].char_handle[0],
								sizeof(body_location1_str), body_location1_str, false); //false for notify*/
					}
				}else if (descr_value == 0x0002){
					if (ESP_GATT_CHAR_PROP_BIT_INDICATE){
						ESP_LOGI(GATTS_TAG, "indicate enable");
						uint8_t indicate_data[15];
						for (int i = 0; i < sizeof(indicate_data); ++i)
						{
							indicate_data[i] = i%0xff;
						}
						//the size of indicate_data[] need less than MTU size
						esp_ble_gatts_send_indicate(gatts_if, param->write.conn_id, gl_profile_tab[profile].char_handle[0],
								sizeof(indicate_data), indicate_data, true);	//true for indicate ()
					}
				}
				else if (descr_value == 0x0000){
					ESP_LOGI(GATTS_TAG, "notify/indicate disable ");
					gl_char[1].is_notify = false;	//notify flag
					//vTaskDelete(notify_TaskHandle);
				}else{
					ESP_LOGE(GATTS_TAG, "unknown descr value");
					esp_log_buffer_hex(GATTS_TAG, param->write.value, param->write.len);
				}
			}
		}
		//example_write_event_env(gatts_if, &a_prepare_write_env, param);
		gatts_check_callback(event, gatts_if, param);
		break;
	}
	case ESP_GATTS_EXEC_WRITE_EVT:
		ESP_LOGI(GATTS_TAG,"ESP_GATTS_EXEC_WRITE_EVT");
		esp_ble_gatts_send_response(gatts_if, param->write.conn_id, param->write.trans_id, ESP_GATT_OK, NULL);
		example_exec_write_event_env(&a_prepare_write_env, param);
		break;
	case ESP_GATTS_MTU_EVT:
		ESP_LOGI(GATTS_TAG, "ESP_GATTS_MTU_EVT, MTU %d", param->mtu.mtu);
		break;
	case ESP_GATTS_UNREG_EVT:
		break;
	case ESP_GATTS_CREATE_EVT:
		ESP_LOGI(GATTS_TAG, "CREATE_SERVICE_EVT, status %d,  service_handle %d\n", param->create.status, param->create.service_handle);
		gl_profile_tab[profile].service_handle = param->create.service_handle;
		gl_profile_tab[profile].char_uuid.len = ESP_UUID_LEN_16;
		gl_profile_tab[profile].char_uuid.uuid.uuid16 = gl_char[1].char_uuid.uuid.uuid16;

		esp_ble_gatts_start_service(gl_profile_tab[profile].service_handle);
		gatts_add_char(profile);
		break;
	case ESP_GATTS_ADD_INCL_SRVC_EVT:
		break;
	case ESP_GATTS_ADD_CHAR_EVT: {

		ESP_LOGI(GATTS_TAG, "ADD_CHAR_EVT, status 0x%X,  attr_handle %d, service_handle %d\n",
				param->add_char.status, param->add_char.attr_handle, param->add_char.service_handle);

		if (param->add_char.status==ESP_GATT_OK) {
			gl_profile_tab[profile].char_handle[0] = param->add_char.attr_handle;
			gatts_check_add_char(param->add_char.char_uuid,param->add_char.attr_handle,profile);
		}

		if (gl_profile_tab[profile].char_num < gl_profile_tab[profile].char_num_total) { //there are more chars to add
			gatts_add_char(profile);
		}else{//organize the char handlers
			for (int i = gl_profile_tab[profile].char_num_total-1; i >=0; i--) {

				uint8_t base_handler = gl_profile_tab[profile].char_handle[0];
				gl_profile_tab[profile].char_handle[i]= base_handler-((gl_profile_tab[profile].char_num_total-i-1)*3);
				//printf("profile: %d\ti=%d\tchar_handler[i]=%d\n",profile,i,gl_profile_tab[profile].char_handle[i]);
			}
		}
		break;
	}
	case ESP_GATTS_ADD_CHAR_DESCR_EVT:
		gl_profile_tab[profile].descr_handle = param->add_char_descr.attr_handle;
		ESP_LOGI(GATTS_TAG, "ADD_DESCR_EVT, status %d, attr_handle %d, service_handle %d\n",
				param->add_char_descr.status, param->add_char_descr.attr_handle, param->add_char_descr.service_handle);
		break;
	case ESP_GATTS_DELETE_EVT:
		break;
	case ESP_GATTS_START_EVT:
		ESP_LOGI(GATTS_TAG, "SERVICE_START_EVT, status %d, service_handle %d\n",
				param->start.status, param->start.service_handle);
		break;
	case ESP_GATTS_STOP_EVT:
		break;
	case ESP_GATTS_CONNECT_EVT: {
#ifdef EN_MAX30102_READING_TASK
#ifdef EN_SENSOR0
		max30102_init(I2C_NUM_0);	//start sensors
#endif
#ifdef EN_SENSOR1
		max30102_init(I2C_NUM_1);
#endif
#endif
		esp_ble_conn_update_params_t conn_params = {0};
		memcpy(conn_params.bda, param->connect.remote_bda, sizeof(esp_bd_addr_t));
		/* For the IOS system, please reference the apple official documents about the ble connection parameters restrictions. */
		conn_params.latency = 0;
		conn_params.max_int = 0x20;    // max_int = 0x20*1.25ms = 40ms
		conn_params.min_int = 0x10;    // min_int = 0x10*1.25ms = 20ms
		conn_params.timeout = 3000;    // timeout = 400*10ms = 4000ms
		ESP_LOGI(GATTS_TAG, "ESP_GATTS_CONNECT_EVT, conn_id %d, remote %02x:%02x:%02x:%02x:%02x:%02x:",
				param->connect.conn_id,
				param->connect.remote_bda[0], param->connect.remote_bda[1], param->connect.remote_bda[2],
				param->connect.remote_bda[3], param->connect.remote_bda[4], param->connect.remote_bda[5]);
		gl_profile_tab[profile].conn_id = param->connect.conn_id;
		//start sent the update connection parameters to the peer device.
		esp_ble_gap_update_conn_params(&conn_params);
		//esp_ble_gap_stop_advertising();//todo stop advertising or not?
		break;
	}
	case ESP_GATTS_DISCONNECT_EVT:
		ESP_LOGI(GATTS_TAG, "ESP_GATTS_DISCONNECT_EVT");
		for (int i = 0; i < GATTS_CHAR_NUM_TOTAL; i++) {//disable all notify
			gl_char[i].is_notify = false;
		}
		esp_ble_gap_start_advertising(&adv_params);
#ifdef EN_SENSOR0
		max30102_shutdown(I2C_NUM_0);	//shutdown sensor
#endif
#ifdef EN_SENSOR1
		max30102_shutdown(I2C_NUM_1);	//shutdown sensor
#endif
		break;
	case ESP_GATTS_CONF_EVT:
		ESP_LOGI(GATTS_TAG, "ESP_GATTS_CONF_EVT, status %d", param->conf.status);
		if (param->conf.status != ESP_GATT_OK){
			esp_log_buffer_hex(GATTS_TAG, param->conf.value, param->conf.len);
		}
		break;
	case ESP_GATTS_OPEN_EVT:
		ESP_LOGI(GATTS_TAG, "ESP_GATTS_OPEN_EVT");
		break;
	case ESP_GATTS_CANCEL_OPEN_EVT:
		ESP_LOGI(GATTS_TAG, "ESP_GATTS_CANCEL_OPEN_EVT");
		break;
	case ESP_GATTS_CLOSE_EVT:
		ESP_LOGI(GATTS_TAG, "ESP_GATTS_CLOSE_EVT");
		break;
	case ESP_GATTS_LISTEN_EVT:
		ESP_LOGI(GATTS_TAG, "ESP_GATTS_LISTEN_EVT");
		break;
	case ESP_GATTS_CONGEST_EVT:
		ESP_LOGI(GATTS_TAG, "ESP_GATTS_CONGEST_EVT");
		break;
	default:
		break;
	}
}
static void gatts_profile_b_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param) {
	uint8_t profile = PROFILE_PLX1_APP_ID;
	switch (event) {
	case ESP_GATTS_REG_EVT:

		ESP_LOGI(GATTS_TAG, "REGISTER_APP_EVT, status %d, app_id %d\n", param->reg.status, param->reg.app_id);
		gl_profile_tab[profile].service_id.is_primary = true;
		gl_profile_tab[profile].service_id.id.inst_id = 0x00;
		gl_profile_tab[profile].service_id.id.uuid.len = ESP_UUID_LEN_16;
		gl_profile_tab[profile].service_id.id.uuid.uuid.uuid16 = GATTS_SERVICE_UUID_PULSE_OXIMETER;

		esp_ble_gatts_create_service(gatts_if, &gl_profile_tab[profile].service_id, GATTS_NUM_HANDLE_PLX);
		break;

	case ESP_GATTS_READ_EVT: {
		ESP_LOGI(GATTS_TAG, "GATT_READ_EVT, conn_id %d, trans_id %d, handle %d\n", param->read.conn_id, param->read.trans_id, param->read.handle);
		/*esp_gatt_rsp_t rsp;
        memset(&rsp, 0, sizeof(esp_gatt_rsp_t));
        rsp.attr_value.len = 4;
        rsp.attr_value.value[0] = 0x11;
        rsp.attr_value.value[1] = 0x11;
        rsp.attr_value.value[2] = 0xbe;
        rsp.attr_value.value[3] = 0xef;
        esp_ble_gatts_send_response(gatts_if, param->read.conn_id, param->read.trans_id,
                                    ESP_GATT_OK, &rsp);*/
		gatts_check_callback(event, gatts_if, param);
		break;
	}
	case ESP_GATTS_WRITE_EVT: {
		ESP_LOGI(GATTS_TAG, "GATT_WRITE_EVT, conn_id %d, trans_id %d, handle %d", param->write.conn_id, param->write.trans_id, param->write.handle);
		ESP_LOGI(GATTS_TAG, "GATT_WRITE_EVT, value len %d, value %08x\n", param->write.len, *(uint32_t *)param->write.value);
		if (!param->write.is_prep){
			ESP_LOGI(GATTS_TAG, "GATT_WRITE_EVT, value len %d, value :", param->write.len);
			esp_log_buffer_hex(GATTS_TAG, param->write.value, param->write.len);
			if (gl_profile_tab[profile].descr_handle == param->write.handle && param->write.len == 2){
				uint16_t descr_value = param->write.value[1]<<8 | param->write.value[0];
				if (descr_value == 0x0001){
					if (ESP_GATT_CHAR_PROP_BIT_NOTIFY){
						ESP_LOGI(GATTS_TAG, "notify enable");
						gl_char[2].is_notify = true;
#ifdef EN_NOTIFY
						if (!notify_task_running){
							xTaskCreate(notify_task_optimized, "notify_task_optimized", 1024 * 4, (void*) 0, 10, &notify_TaskHandle);
						}
#endif
					}
				}else if (descr_value == 0x0002){
					if (ESP_GATT_CHAR_PROP_BIT_INDICATE){
						ESP_LOGI(GATTS_TAG, "indicate enable");
						uint8_t indicate_data[15];
						for (int i = 0; i < sizeof(indicate_data); ++i)
						{
							indicate_data[i] = i%0xff;
						}
						//the size of indicate_data[] need less than MTU size
						esp_ble_gatts_send_indicate(gatts_if, param->write.conn_id, gl_profile_tab[profile].char_handle[0],
								sizeof(indicate_data), indicate_data, true);
					}
				}
				else if (descr_value == 0x0000){
					ESP_LOGI(GATTS_TAG, "notify/indicate disable ");
					gl_char[2].is_notify = false;
				}else{
					ESP_LOGE(GATTS_TAG, "unknown descr value");
					esp_log_buffer_hex(GATTS_TAG, param->write.value, param->write.len);
				}

			}
		}
		//example_write_event_env(gatts_if, &b_prepare_write_env, param);
		gatts_check_callback(event, gatts_if, param);
		break;
	}
	case ESP_GATTS_EXEC_WRITE_EVT:
		ESP_LOGI(GATTS_TAG,"ESP_GATTS_EXEC_WRITE_EVT");
		esp_ble_gatts_send_response(gatts_if, param->write.conn_id, param->write.trans_id, ESP_GATT_OK, NULL);
		example_exec_write_event_env(&b_prepare_write_env, param);
		break;
	case ESP_GATTS_MTU_EVT:
		ESP_LOGI(GATTS_TAG, "ESP_GATTS_MTU_EVT, MTU %d", param->mtu.mtu);
		break;
	case ESP_GATTS_UNREG_EVT:
		break;
	case ESP_GATTS_CREATE_EVT:
		ESP_LOGI(GATTS_TAG, "CREATE_SERVICE_EVT, status %d,  service_handle %d\n", param->create.status, param->create.service_handle);
		gl_profile_tab[profile].service_handle = param->create.service_handle;
		gl_profile_tab[profile].char_uuid.len = ESP_UUID_LEN_16;
		gl_profile_tab[profile].char_uuid.uuid.uuid16 = gl_char[2].char_uuid.uuid.uuid16;

		esp_ble_gatts_start_service(gl_profile_tab[profile].service_handle);
		gatts_add_char(profile);

		break;
	case ESP_GATTS_ADD_INCL_SRVC_EVT:
		break;
	case ESP_GATTS_ADD_CHAR_EVT: {

		ESP_LOGI(GATTS_TAG, "ADD_CHAR_EVT, status 0x%X,  attr_handle %d, service_handle %d\n",
				param->add_char.status, param->add_char.attr_handle, param->add_char.service_handle);


		if (param->add_char.status==ESP_GATT_OK) {
			gl_profile_tab[profile].char_handle[0] = param->add_char.attr_handle;
			gatts_check_add_char(param->add_char.char_uuid,param->add_char.attr_handle,profile);
		}
		if (gl_profile_tab[profile].char_num < gl_profile_tab[profile].char_num_total) {
			//gatts_add_char(profile);
		}
		break;
	}
	case ESP_GATTS_ADD_CHAR_DESCR_EVT:
		gl_profile_tab[profile].descr_handle = param->add_char_descr.attr_handle;
		ESP_LOGI(GATTS_TAG, "ADD_DESCR_EVT, status %d, attr_handle %d, service_handle %d\n",
				param->add_char_descr.status, param->add_char_descr.attr_handle, param->add_char_descr.service_handle);
		break;
	case ESP_GATTS_DELETE_EVT:
		break;
	case ESP_GATTS_START_EVT:
		ESP_LOGI(GATTS_TAG, "SERVICE_START_EVT, status %d, service_handle %d\n",
				param->start.status, param->start.service_handle);
		break;
	case ESP_GATTS_STOP_EVT:
		break;
	case ESP_GATTS_CONNECT_EVT: {
		break;
	}
	case ESP_GATTS_DISCONNECT_EVT:
		ESP_LOGI(GATTS_TAG, "ESP_GATTS_DISCONNECT_EVT");
		break;
	case ESP_GATTS_CONF_EVT:
		ESP_LOGI(GATTS_TAG, "ESP_GATTS_CONF_EVT, status %d",  param->conf.status);
		if (param->conf.status != ESP_GATT_OK){
			esp_log_buffer_hex(GATTS_TAG, param->conf.value, param->conf.len);
		}
		break;
	case ESP_GATTS_OPEN_EVT:
		ESP_LOGI(GATTS_TAG, "ESP_GATTS_OPEN_EVT");
		break;
	case ESP_GATTS_CANCEL_OPEN_EVT:
		ESP_LOGI(GATTS_TAG, "ESP_GATTS_CANCEL_OPEN_EVT");
		break;
	case ESP_GATTS_CLOSE_EVT:
		ESP_LOGI(GATTS_TAG, "ESP_GATTS_CLOSE_EVT");
		break;
	case ESP_GATTS_LISTEN_EVT:
		ESP_LOGI(GATTS_TAG, "ESP_GATTS_LISTEN_EVT");
		break;
	case ESP_GATTS_CONGEST_EVT:
		ESP_LOGI(GATTS_TAG, "ESP_GATTS_CONGEST_EVT");
		break;
	default:
		break;
	}
}
static void gatts_profile_c_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param) {
	uint8_t profile = PROFILE_HR2_APP_ID;
	switch (event) {
	case ESP_GATTS_REG_EVT:

		ESP_LOGI(GATTS_TAG, "REGISTER_APP_EVT, status %d, app_id %d\n", param->reg.status, param->reg.app_id);
		gl_profile_tab[profile].service_id.is_primary = true;
		gl_profile_tab[profile].service_id.id.inst_id = 0x00;
		gl_profile_tab[profile].service_id.id.uuid.len = ESP_UUID_LEN_16;
		gl_profile_tab[profile].service_id.id.uuid.uuid.uuid16 = GATTS_SERVICE_UUID_HEART_RATE;

		esp_ble_gatts_create_service(gatts_if, &gl_profile_tab[profile].service_id, GATTS_NUM_HANDLE_HR);

		break;

	case ESP_GATTS_READ_EVT: {
		ESP_LOGI(GATTS_TAG, "GATT_READ_EVT, conn_id %d, trans_id %d, handle %d\n", param->read.conn_id, param->read.trans_id, param->read.handle);
		/*esp_gatt_rsp_t rsp;
        memset(&rsp, 0, sizeof(esp_gatt_rsp_t));
        rsp.attr_value.len = 4;
        rsp.attr_value.value[0] = 0x11;
        rsp.attr_value.value[1] = 0x11;
        rsp.attr_value.value[2] = 0xbe;
        rsp.attr_value.value[3] = 0xef;
        esp_ble_gatts_send_response(gatts_if, param->read.conn_id, param->read.trans_id,
                                    ESP_GATT_OK, &rsp);*/
		gatts_check_callback(event, gatts_if, param);
		break;
	}
	case ESP_GATTS_WRITE_EVT: {
		ESP_LOGI(GATTS_TAG, "GATT_WRITE_EVT, conn_id %d, trans_id %d, handle %d", param->write.conn_id, param->write.trans_id, param->write.handle);
		ESP_LOGI(GATTS_TAG, "GATT_WRITE_EVT, value len %d, value %08x\n", param->write.len, *(uint32_t *)param->write.value);
		if (!param->write.is_prep){
			ESP_LOGI(GATTS_TAG, "GATT_WRITE_EVT, value len %d, value :", param->write.len);
			esp_log_buffer_hex(GATTS_TAG, param->write.value, param->write.len);
			if (gl_profile_tab[profile].descr_handle == param->write.handle && param->write.len == 2){
				uint16_t descr_value = param->write.value[1]<<8 | param->write.value[0];
				if (descr_value == 0x0001){
					if (ESP_GATT_CHAR_PROP_BIT_NOTIFY){
						ESP_LOGI(GATTS_TAG, "notify enable");
						gl_char[4].is_notify = true;
#ifdef EN_NOTIFY
						if (!notify_task_running){
							xTaskCreate(notify_task_optimized, "notify_task_optimized", 1024 * 4, (void*) 0, 10, &notify_TaskHandle);
						}
#endif
					}
				}else if (descr_value == 0x0002){
					if (ESP_GATT_CHAR_PROP_BIT_INDICATE){
						ESP_LOGI(GATTS_TAG, "indicate enable");
						uint8_t indicate_data[15];
						for (int i = 0; i < sizeof(indicate_data); ++i)
						{
							indicate_data[i] = i%0xff;
						}
						//the size of indicate_data[] need less than MTU size
						esp_ble_gatts_send_indicate(gatts_if, param->write.conn_id, gl_profile_tab[profile].char_handle[0],
								sizeof(indicate_data), indicate_data, true);
					}
				}
				else if (descr_value == 0x0000){
					ESP_LOGI(GATTS_TAG, "notify/indicate disable ");
					gl_char[4].is_notify = false;
				}else{
					ESP_LOGE(GATTS_TAG, "unknown descr value");
					esp_log_buffer_hex(GATTS_TAG, param->write.value, param->write.len);
				}

			}
		}
		//example_write_event_env(gatts_if, &c_prepare_write_env, param);
		gatts_check_callback(event, gatts_if, param);
		break;
	}
	case ESP_GATTS_EXEC_WRITE_EVT:
		ESP_LOGI(GATTS_TAG,"ESP_GATTS_EXEC_WRITE_EVT");
		esp_ble_gatts_send_response(gatts_if, param->write.conn_id, param->write.trans_id, ESP_GATT_OK, NULL);
		example_exec_write_event_env(&c_prepare_write_env, param);
		break;
	case ESP_GATTS_MTU_EVT:
		ESP_LOGI(GATTS_TAG, "ESP_GATTS_MTU_EVT, MTU %d", param->mtu.mtu);
		break;
	case ESP_GATTS_UNREG_EVT:
		break;
	case ESP_GATTS_CREATE_EVT:
		ESP_LOGI(GATTS_TAG, "CREATE_SERVICE_EVT, status %d,  service_handle %d\n", param->create.status, param->create.service_handle);
		gl_profile_tab[profile].service_handle = param->create.service_handle;
		gl_profile_tab[profile].char_uuid.len = ESP_UUID_LEN_16;
		gl_profile_tab[profile].char_uuid.uuid.uuid16 = gl_char[5].char_uuid.uuid.uuid16;

		esp_ble_gatts_start_service(gl_profile_tab[profile].service_handle);
		gatts_add_char(profile);

		break;
	case ESP_GATTS_ADD_INCL_SRVC_EVT:
		break;
	case ESP_GATTS_ADD_CHAR_EVT: {
		ESP_LOGI(GATTS_TAG, "ADD_CHAR_EVT, status 0x%X,  attr_handle %d, service_handle %d\n",
				param->add_char.status, param->add_char.attr_handle, param->add_char.service_handle);


		if (param->add_char.status==ESP_GATT_OK) {
			gatts_check_add_char(param->add_char.char_uuid,param->add_char.attr_handle,profile);
			gl_profile_tab[profile].char_handle[0] = param->add_char.attr_handle;
		}
		if (gl_profile_tab[profile].char_num < gl_profile_tab[profile].char_num_total) {
			gatts_add_char(profile);
		}else{//organize the char handlers
			for (int i = gl_profile_tab[profile].char_num_total-1; i >=0; i--) {
				uint8_t base_handler = gl_profile_tab[profile].char_handle[0];
				gl_profile_tab[profile].char_handle[i]= base_handler-((gl_profile_tab[profile].char_num_total-i-1)*3);//ex: 67,64,61...
				//printf("profile: %d\ti=%d\tchar_handler[i]=%d\n",profile,i,gl_profile_tab[profile].char_handle[i]);
			}
		}
		break;
	}
	case ESP_GATTS_ADD_CHAR_DESCR_EVT:
		gl_profile_tab[profile].descr_handle = param->add_char_descr.attr_handle;
		ESP_LOGI(GATTS_TAG, "ADD_DESCR_EVT, status %d, attr_handle %d, service_handle %d\n",
				param->add_char_descr.status, param->add_char_descr.attr_handle, param->add_char_descr.service_handle);
		break;
	case ESP_GATTS_DELETE_EVT:
		break;
	case ESP_GATTS_START_EVT:
		ESP_LOGI(GATTS_TAG, "SERVICE_START_EVT, status %d, service_handle %d\n",
				param->start.status, param->start.service_handle);
		break;
	case ESP_GATTS_STOP_EVT:
		break;
	case ESP_GATTS_CONNECT_EVT: {
		break;
	}
	case ESP_GATTS_DISCONNECT_EVT:
		ESP_LOGI(GATTS_TAG, "ESP_GATTS_DISCONNECT_EVT");
		break;
	case ESP_GATTS_CONF_EVT:
		ESP_LOGI(GATTS_TAG, "ESP_GATTS_CONF_EVT, status %d",  param->conf.status);
		if (param->conf.status != ESP_GATT_OK){
			esp_log_buffer_hex(GATTS_TAG, param->conf.value, param->conf.len);
		}
		break;
	case ESP_GATTS_OPEN_EVT:
		ESP_LOGI(GATTS_TAG, "ESP_GATTS_OPEN_EVT");
		break;
	case ESP_GATTS_CANCEL_OPEN_EVT:
		ESP_LOGI(GATTS_TAG, "ESP_GATTS_CANCEL_OPEN_EVT");
		break;
	case ESP_GATTS_CLOSE_EVT:
		ESP_LOGI(GATTS_TAG, "ESP_GATTS_CLOSE_EVT");
		break;
	case ESP_GATTS_LISTEN_EVT:
		ESP_LOGI(GATTS_TAG, "ESP_GATTS_LISTEN_EVT");
		break;
	case ESP_GATTS_CONGEST_EVT:
		ESP_LOGI(GATTS_TAG, "ESP_GATTS_CONGEST_EVT");
		break;
	default:
		break;
	}
}
static void gatts_profile_d_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param) {
	uint8_t profile = PROFILE_PLX2_APP_ID;
	switch (event) {
	case ESP_GATTS_REG_EVT:

		ESP_LOGI(GATTS_TAG, "REGISTER_APP_EVT, status %d, app_id %d\n", param->reg.status, param->reg.app_id);
		gl_profile_tab[profile].service_id.is_primary = true;
		gl_profile_tab[profile].service_id.id.inst_id = 0x00;
		gl_profile_tab[profile].service_id.id.uuid.len = ESP_UUID_LEN_16;
		gl_profile_tab[profile].service_id.id.uuid.uuid.uuid16 = GATTS_SERVICE_UUID_PULSE_OXIMETER;

		esp_ble_gatts_create_service(gatts_if, &gl_profile_tab[profile].service_id, GATTS_NUM_HANDLE_PLX);
		break;

	case ESP_GATTS_READ_EVT: {
		ESP_LOGI(GATTS_TAG, "GATT_READ_EVT, conn_id %d, trans_id %d, handle %d\n", param->read.conn_id, param->read.trans_id, param->read.handle);
		/*esp_gatt_rsp_t rsp;
        memset(&rsp, 0, sizeof(esp_gatt_rsp_t));
        rsp.attr_value.len = 4;
        rsp.attr_value.value[0] = 0x11;
        rsp.attr_value.value[1] = 0x11;
        rsp.attr_value.value[2] = 0xbe;
        rsp.attr_value.value[3] = 0xef;
        esp_ble_gatts_send_response(gatts_if, param->read.conn_id, param->read.trans_id,
                                    ESP_GATT_OK, &rsp);*/
		gatts_check_callback(event, gatts_if, param);
		break;
	}
	case ESP_GATTS_WRITE_EVT: {
		ESP_LOGI(GATTS_TAG, "GATT_WRITE_EVT, conn_id %d, trans_id %d, handle %d", param->write.conn_id, param->write.trans_id, param->write.handle);
		ESP_LOGI(GATTS_TAG, "GATT_WRITE_EVT, value len %d, value %08x\n", param->write.len, *(uint32_t *)param->write.value);
		if (!param->write.is_prep){
			ESP_LOGI(GATTS_TAG, "GATT_WRITE_EVT, value len %d, value :", param->write.len);
			esp_log_buffer_hex(GATTS_TAG, param->write.value, param->write.len);
			if (gl_profile_tab[profile].descr_handle == param->write.handle && param->write.len == 2){
				uint16_t descr_value = param->write.value[1]<<8 | param->write.value[0];
				if (descr_value == 0x0001){
					if (ESP_GATT_CHAR_PROP_BIT_NOTIFY){
						ESP_LOGI(GATTS_TAG, "notify enable");
						gl_char[5].is_notify = true;
#ifdef EN_NOTIFY
						if (!notify_task_running){
							xTaskCreate(notify_task_optimized, "notify_task_optimized", 1024 * 4, (void*) 0, 10, &notify_TaskHandle);
						}
#endif
					}
				}else if (descr_value == 0x0002){
					if (ESP_GATT_CHAR_PROP_BIT_INDICATE){
						ESP_LOGI(GATTS_TAG, "indicate enable");
						uint8_t indicate_data[15];
						for (int i = 0; i < sizeof(indicate_data); ++i)
						{
							indicate_data[i] = i%0xff;
						}

						//the size of indicate_data[] need less than MTU size
						esp_ble_gatts_send_indicate(gatts_if, param->write.conn_id, gl_profile_tab[profile].char_handle[0],
								sizeof(indicate_data), indicate_data, true);
					}
				}
				else if (descr_value == 0x0000){
					ESP_LOGI(GATTS_TAG, "notify/indicate disable ");
					gl_char[5].is_notify = false;
				}else{
					ESP_LOGE(GATTS_TAG, "unknown descr value");
					esp_log_buffer_hex(GATTS_TAG, param->write.value, param->write.len);
				}

			}
		}
		//example_write_event_env(gatts_if, &d_prepare_write_env, param);
		gatts_check_callback(event, gatts_if, param);
		break;
	}
	case ESP_GATTS_EXEC_WRITE_EVT:
		ESP_LOGI(GATTS_TAG,"ESP_GATTS_EXEC_WRITE_EVT");
		esp_ble_gatts_send_response(gatts_if, param->write.conn_id, param->write.trans_id, ESP_GATT_OK, NULL);
		example_exec_write_event_env(&d_prepare_write_env, param);
		break;
	case ESP_GATTS_MTU_EVT:
		ESP_LOGI(GATTS_TAG, "ESP_GATTS_MTU_EVT, MTU %d", param->mtu.mtu);
		break;
	case ESP_GATTS_UNREG_EVT:
		break;
	case ESP_GATTS_CREATE_EVT:
		ESP_LOGI(GATTS_TAG, "CREATE_SERVICE_EVT, status %d,  service_handle %d\n", param->create.status, param->create.service_handle);
		gl_profile_tab[profile].service_handle = param->create.service_handle;
		gl_profile_tab[profile].char_uuid.len = ESP_UUID_LEN_16;
		gl_profile_tab[profile].char_uuid.uuid.uuid16 = gl_char[5].char_uuid.uuid.uuid16;

		esp_ble_gatts_start_service(gl_profile_tab[profile].service_handle);
		gatts_add_char(profile);

		break;
	case ESP_GATTS_ADD_INCL_SRVC_EVT:
		break;
	case ESP_GATTS_ADD_CHAR_EVT: {

		ESP_LOGI(GATTS_TAG, "ADD_CHAR_EVT, status 0x%X,  attr_handle %d, service_handle %d\n",
				param->add_char.status, param->add_char.attr_handle, param->add_char.service_handle);

		if (param->add_char.status==ESP_GATT_OK) {
			gl_profile_tab[profile].char_handle[0] = param->add_char.attr_handle;
			gatts_check_add_char(param->add_char.char_uuid,param->add_char.attr_handle,profile);
		}
		if (gl_profile_tab[profile].char_num < gl_profile_tab[profile].char_num_total) {
			gatts_add_char(profile);
		}
		break;
	}
	case ESP_GATTS_ADD_CHAR_DESCR_EVT:
		gl_profile_tab[profile].descr_handle = param->add_char_descr.attr_handle;
		ESP_LOGI(GATTS_TAG, "ADD_DESCR_EVT, status %d, attr_handle %d, service_handle %d\n",
				param->add_char_descr.status, param->add_char_descr.attr_handle, param->add_char_descr.service_handle);
		break;
	case ESP_GATTS_DELETE_EVT:
		break;
	case ESP_GATTS_START_EVT:
		ESP_LOGI(GATTS_TAG, "SERVICE_START_EVT, status %d, service_handle %d\n",
				param->start.status, param->start.service_handle);
		break;
	case ESP_GATTS_STOP_EVT:
		break;
	case ESP_GATTS_CONNECT_EVT: {
		break;
	}
	case ESP_GATTS_DISCONNECT_EVT:
		ESP_LOGI(GATTS_TAG, "ESP_GATTS_DISCONNECT_EVT");
		break;
	case ESP_GATTS_CONF_EVT:
		ESP_LOGI(GATTS_TAG, "ESP_GATTS_CONF_EVT, status %d", param->conf.status);
		if (param->conf.status != ESP_GATT_OK){
			esp_log_buffer_hex(GATTS_TAG, param->conf.value, param->conf.len);
		}
		break;
	case ESP_GATTS_OPEN_EVT:
		ESP_LOGI(GATTS_TAG, "ESP_GATTS_OPEN_EVT");
		break;
	case ESP_GATTS_CANCEL_OPEN_EVT:
		ESP_LOGI(GATTS_TAG, "ESP_GATTS_CANCEL_OPEN_EVT");
		break;
	case ESP_GATTS_CLOSE_EVT:
		ESP_LOGI(GATTS_TAG, "ESP_GATTS_CLOSE_EVT");
		break;
	case ESP_GATTS_LISTEN_EVT:
		ESP_LOGI(GATTS_TAG, "ESP_GATTS_LISTEN_EVT");
		break;
	case ESP_GATTS_CONGEST_EVT:
		ESP_LOGI(GATTS_TAG, "ESP_GATTS_CONGEST_EVT");
		break;
	default:
		break;
	}
}
static void gatts_profile_e_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param) {
	uint8_t profile = PROFILE_RAW_APP_ID;
	switch (event) {
	case ESP_GATTS_REG_EVT:

		ESP_LOGI(GATTS_TAG, "REGISTER_APP_EVT, status %d, app_id %d\n", param->reg.status, param->reg.app_id);
		gl_profile_tab[profile].service_id.is_primary = true;
		gl_profile_tab[profile].service_id.id.inst_id = 0x00;
		gl_profile_tab[profile].service_id.id.uuid.len = ESP_UUID_LEN_16;
		gl_profile_tab[profile].service_id.id.uuid.uuid.uuid16 = GATTS_SERVICE_UUID_RAW_DATA_SERVICE;

		esp_ble_gatts_create_service(gatts_if, &gl_profile_tab[profile].service_id, GATTS_NUM_HANDLE_RAW);
		break;

	case ESP_GATTS_READ_EVT: {
		ESP_LOGI(GATTS_TAG, "GATT_READ_EVT, conn_id %d, trans_id %d, handle %d\n", param->read.conn_id, param->read.trans_id, param->read.handle);
		/*esp_gatt_rsp_t rsp;
        memset(&rsp, 0, sizeof(esp_gatt_rsp_t));
        rsp.attr_value.len = 4;
        rsp.attr_value.value[0] = 0x11;
        rsp.attr_value.value[1] = 0x11;
        rsp.attr_value.value[2] = 0xbe;
        rsp.attr_value.value[3] = 0xef;
        esp_ble_gatts_send_response(gatts_if, param->read.conn_id, param->read.trans_id,
                                    ESP_GATT_OK, &rsp);*/
		gatts_check_callback(event, gatts_if, param);
		break;
	}
	case ESP_GATTS_WRITE_EVT: {
		ESP_LOGI(GATTS_TAG, "GATT_WRITE_EVT, conn_id %d, trans_id %d, handle %d", param->write.conn_id, param->write.trans_id, param->write.handle);
		ESP_LOGI(GATTS_TAG, "GATT_WRITE_EVT, value len %d, value %08x\n", param->write.len, *(uint32_t *)param->write.value);
		if (!param->write.is_prep){
			ESP_LOGI(GATTS_TAG, "GATT_WRITE_EVT, value len %d, value :", param->write.len);
			esp_log_buffer_hex(GATTS_TAG, param->write.value, param->write.len);
			if ((gl_profile_tab[profile].descr_handle == param->write.handle
					|| gl_profile_tab[profile].descr_handle == param->write.handle+3)
					&& param->write.len == 2){
				uint16_t descr_value = param->write.value[1]<<8 | param->write.value[0];
				uint8_t raw1 = !(gl_profile_tab[profile].descr_handle == param->write.handle+3);
				if (descr_value == 0x0001){
					if (ESP_GATT_CHAR_PROP_BIT_NOTIFY){
						if (raw1){
							gl_char[7].is_notify = true;
						}else{
							gl_char[6].is_notify = true;
						}
#ifdef EN_NOTIFY
if (!notify_task_running){
	xTaskCreate(notify_task_optimized, "notify_task_optimized", 1024 * 4, (void*) 0, 10, &notify_TaskHandle);
}
#endif
					}
				}else if (descr_value == 0x0002){
					if (ESP_GATT_CHAR_PROP_BIT_INDICATE){
						ESP_LOGI(GATTS_TAG, "indicate enable");
						uint8_t indicate_data[15];
						for (int i = 0; i < sizeof(indicate_data); ++i)
						{
							indicate_data[i] = i%0xff;
						}

						//the size of indicate_data[] need less than MTU size
						esp_ble_gatts_send_indicate(gatts_if, param->write.conn_id, gl_profile_tab[profile].char_handle[0],
								sizeof(indicate_data), indicate_data, true);
					}
				}
				else if (descr_value == 0x0000){
					ESP_LOGI(GATTS_TAG, "notify/indicate disable ");
					if (raw1){
						gl_char[7].is_notify = false;
					}else {
						gl_char[6].is_notify = false;
					}
				}else{
					ESP_LOGE(GATTS_TAG, "unknown descr value");
					esp_log_buffer_hex(GATTS_TAG, param->write.value, param->write.len);
				}

			}
		}
		//example_write_event_env(gatts_if, &d_prepare_write_env, param);
		gatts_check_callback(event, gatts_if, param);
		break;
	}
	case ESP_GATTS_EXEC_WRITE_EVT:
		ESP_LOGI(GATTS_TAG,"ESP_GATTS_EXEC_WRITE_EVT");
		esp_ble_gatts_send_response(gatts_if, param->write.conn_id, param->write.trans_id, ESP_GATT_OK, NULL);
		example_exec_write_event_env(&d_prepare_write_env, param);
		break;
	case ESP_GATTS_MTU_EVT:
		ESP_LOGI(GATTS_TAG, "ESP_GATTS_MTU_EVT, MTU %d", param->mtu.mtu);
		break;
	case ESP_GATTS_UNREG_EVT:
		break;
	case ESP_GATTS_CREATE_EVT:
		ESP_LOGI(GATTS_TAG, "CREATE_SERVICE_EVT, status %d,  service_handle %d\n", param->create.status, param->create.service_handle);
		gl_profile_tab[profile].service_handle = param->create.service_handle;
		gl_profile_tab[profile].char_uuid.len = ESP_UUID_LEN_16;
		gl_profile_tab[profile].char_uuid.uuid.uuid16 = gl_char[5].char_uuid.uuid.uuid16;

		esp_ble_gatts_start_service(gl_profile_tab[profile].service_handle);
		gatts_add_char(profile);

		break;
	case ESP_GATTS_ADD_INCL_SRVC_EVT:
		break;
	case ESP_GATTS_ADD_CHAR_EVT: {

		ESP_LOGI(GATTS_TAG, "ADD_CHAR_EVT, status 0x%X,  attr_handle %d, service_handle %d\n",
				param->add_char.status, param->add_char.attr_handle, param->add_char.service_handle);

		if (param->add_char.status==ESP_GATT_OK) {
			gl_profile_tab[profile].char_handle[0] = param->add_char.attr_handle;
			gatts_check_add_char(param->add_char.char_uuid,param->add_char.attr_handle,profile);
		}
		if (gl_profile_tab[profile].char_num < gl_profile_tab[profile].char_num_total) { //there are more chars to add
			gatts_add_char(profile);
		}else{//organize the char handlers
			for (int i = gl_profile_tab[profile].char_num_total-1; i >=0; i--) {

				uint8_t base_handler = gl_profile_tab[profile].char_handle[0];
				gl_profile_tab[profile].char_handle[i]= base_handler-((gl_profile_tab[profile].char_num_total-i-1)*3);
				//printf("profile: %d\ti=%d\tchar_handler[i]=%d\n",profile,i,gl_profile_tab[profile].char_handle[i]);
			}
		}
		break;
	}
	case ESP_GATTS_ADD_CHAR_DESCR_EVT:
		gl_profile_tab[profile].descr_handle = param->add_char_descr.attr_handle;
		ESP_LOGI(GATTS_TAG, "ADD_DESCR_EVT, status %d, attr_handle %d, service_handle %d\n",
				param->add_char_descr.status, param->add_char_descr.attr_handle, param->add_char_descr.service_handle);
		break;
	case ESP_GATTS_DELETE_EVT:
		break;
	case ESP_GATTS_START_EVT:
		ESP_LOGI(GATTS_TAG, "SERVICE_START_EVT, status %d, service_handle %d\n",
				param->start.status, param->start.service_handle);
		break;
	case ESP_GATTS_STOP_EVT:
		break;
	case ESP_GATTS_CONNECT_EVT: {
		break;
	}
	case ESP_GATTS_DISCONNECT_EVT:
		ESP_LOGI(GATTS_TAG, "ESP_GATTS_DISCONNECT_EVT");
		break;
	case ESP_GATTS_CONF_EVT:
		ESP_LOGI(GATTS_TAG, "ESP_GATTS_CONF_EVT, status %d", param->conf.status);
		if (param->conf.status != ESP_GATT_OK){
			esp_log_buffer_hex(GATTS_TAG, param->conf.value, param->conf.len);
		}
		break;
	case ESP_GATTS_OPEN_EVT:
		ESP_LOGI(GATTS_TAG, "ESP_GATTS_OPEN_EVT");
		break;
	case ESP_GATTS_CANCEL_OPEN_EVT:
		ESP_LOGI(GATTS_TAG, "ESP_GATTS_CANCEL_OPEN_EVT");
		break;
	case ESP_GATTS_CLOSE_EVT:
		ESP_LOGI(GATTS_TAG, "ESP_GATTS_CLOSE_EVT");
		break;
	case ESP_GATTS_LISTEN_EVT:
		ESP_LOGI(GATTS_TAG, "ESP_GATTS_LISTEN_EVT");
		break;
	case ESP_GATTS_CONGEST_EVT:
		ESP_LOGI(GATTS_TAG, "ESP_GATTS_CONGEST_EVT");
		break;
	default:
		break;
	}
}
static void gatts_profile_f_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param) {
	uint8_t profile = PROFILE_BATT_APP_ID;
	switch (event) {
	case ESP_GATTS_REG_EVT:

		ESP_LOGI(GATTS_TAG, "REGISTER_APP_EVT, status %d, app_id %d\n", param->reg.status, param->reg.app_id);
		gl_profile_tab[profile].service_id.is_primary = true;
		gl_profile_tab[profile].service_id.id.inst_id = 0x00;
		gl_profile_tab[profile].service_id.id.uuid.len = ESP_UUID_LEN_16;
		gl_profile_tab[profile].service_id.id.uuid.uuid.uuid16 = GATTS_SERVICE_UUID_BATTERY_SERVICE;

		esp_ble_gatts_create_service(gatts_if, &gl_profile_tab[profile].service_id, GATTS_NUM_HANDLE_BAT);
		break;

	case ESP_GATTS_READ_EVT: {
		ESP_LOGI(GATTS_TAG, "GATT_READ_EVT, conn_id %d, trans_id %d, handle %d\n", param->read.conn_id, param->read.trans_id, param->read.handle);
		/*esp_gatt_rsp_t rsp;
        memset(&rsp, 0, sizeof(esp_gatt_rsp_t));
        rsp.attr_value.len = 4;
        rsp.attr_value.value[0] = 0x11;
        rsp.attr_value.value[1] = 0x11;
        rsp.attr_value.value[2] = 0xbe;
        rsp.attr_value.value[3] = 0xef;
        esp_ble_gatts_send_response(gatts_if, param->read.conn_id, param->read.trans_id,
                                    ESP_GATT_OK, &rsp);*/
		gatts_check_callback(event, gatts_if, param);
		break;
	}
	case ESP_GATTS_WRITE_EVT: {
		ESP_LOGI(GATTS_TAG, "GATT_WRITE_EVT, conn_id %d, trans_id %d, handle %d", param->write.conn_id, param->write.trans_id, param->write.handle);
		ESP_LOGI(GATTS_TAG, "GATT_WRITE_EVT, value len %d, value %08x\n", param->write.len, *(uint32_t *)param->write.value);
		if (!param->write.is_prep){
			ESP_LOGI(GATTS_TAG, "GATT_WRITE_EVT, value len %d, value :", param->write.len);
			esp_log_buffer_hex(GATTS_TAG, param->write.value, param->write.len);
			if (gl_profile_tab[profile].descr_handle == param->write.handle && param->write.len == 2){
				uint16_t descr_value = param->write.value[1]<<8 | param->write.value[0];
				if (descr_value == 0x0001){
					if (ESP_GATT_CHAR_PROP_BIT_NOTIFY){
						ESP_LOGI(GATTS_TAG, "notify enable");
						gl_char[8].is_notify = true;
#ifdef EN_NOTIFY
						if (!notify_task_running){
							xTaskCreate(notify_task_optimized, "notify_task_optimized", 1024 * 4, (void*) 0, 10, &notify_TaskHandle);
						}
#else
						ESP_LOGI(GATTS_TAG, "notify not defined ");
#endif
					}
				}else if (descr_value == 0x0002){
					if (ESP_GATT_CHAR_PROP_BIT_INDICATE){
						ESP_LOGI(GATTS_TAG, "indicate enable");
						uint8_t indicate_data[15];
						for (int i = 0; i < sizeof(indicate_data); ++i)
						{
							indicate_data[i] = i%0xff;
						}

						//the size of indicate_data[] need less than MTU size
						esp_ble_gatts_send_indicate(gatts_if, param->write.conn_id, gl_profile_tab[profile].char_handle[0],
								sizeof(indicate_data), indicate_data, true);
					}
				}
				else if (descr_value == 0x0000){
					ESP_LOGI(GATTS_TAG, "notify/indicate disable ");
					gl_char[8].is_notify = false;
				}else{
					ESP_LOGE(GATTS_TAG, "unknown descr value");
					esp_log_buffer_hex(GATTS_TAG, param->write.value, param->write.len);
				}

			}
		}
		//example_write_event_env(gatts_if, &d_prepare_write_env, param);
		gatts_check_callback(event, gatts_if, param);
		break;
	}
	case ESP_GATTS_EXEC_WRITE_EVT:
		ESP_LOGI(GATTS_TAG,"ESP_GATTS_EXEC_WRITE_EVT");
		esp_ble_gatts_send_response(gatts_if, param->write.conn_id, param->write.trans_id, ESP_GATT_OK, NULL);
		example_exec_write_event_env(&d_prepare_write_env, param);
		break;
	case ESP_GATTS_MTU_EVT:
		ESP_LOGI(GATTS_TAG, "ESP_GATTS_MTU_EVT, MTU %d", param->mtu.mtu);
		break;
	case ESP_GATTS_UNREG_EVT:
		break;
	case ESP_GATTS_CREATE_EVT:
		ESP_LOGI(GATTS_TAG, "CREATE_SERVICE_EVT, status %d,  service_handle %d\n", param->create.status, param->create.service_handle);
		gl_profile_tab[profile].service_handle = param->create.service_handle;
		gl_profile_tab[profile].char_uuid.len = ESP_UUID_LEN_16;
		gl_profile_tab[profile].char_uuid.uuid.uuid16 = gl_char[5].char_uuid.uuid.uuid16;

		esp_ble_gatts_start_service(gl_profile_tab[profile].service_handle);
		gatts_add_char(profile);

		break;
	case ESP_GATTS_ADD_INCL_SRVC_EVT:
		break;
	case ESP_GATTS_ADD_CHAR_EVT: {

		ESP_LOGI(GATTS_TAG, "ADD_CHAR_EVT, status 0x%X,  attr_handle %d, service_handle %d\n",
				param->add_char.status, param->add_char.attr_handle, param->add_char.service_handle);

		if (param->add_char.status==ESP_GATT_OK) {
			gl_profile_tab[profile].char_handle[0] = param->add_char.attr_handle;
			gatts_check_add_char(param->add_char.char_uuid,param->add_char.attr_handle,profile);
		}
		if (gl_profile_tab[profile].char_num < gl_profile_tab[profile].char_num_total) {
			gatts_add_char(profile);
		}
		break;
	}
	case ESP_GATTS_ADD_CHAR_DESCR_EVT:
		gl_profile_tab[profile].descr_handle = param->add_char_descr.attr_handle;
		ESP_LOGI(GATTS_TAG, "ADD_DESCR_EVT, status %d, attr_handle %d, service_handle %d\n",
				param->add_char_descr.status, param->add_char_descr.attr_handle, param->add_char_descr.service_handle);
		break;
	case ESP_GATTS_DELETE_EVT:
		break;
	case ESP_GATTS_START_EVT:
		ESP_LOGI(GATTS_TAG, "SERVICE_START_EVT, status %d, service_handle %d\n",
				param->start.status, param->start.service_handle);
		break;
	case ESP_GATTS_STOP_EVT:
		break;
	case ESP_GATTS_CONNECT_EVT: {
		break;
	}
	case ESP_GATTS_DISCONNECT_EVT:
		ESP_LOGI(GATTS_TAG, "ESP_GATTS_DISCONNECT_EVT");
		break;
	case ESP_GATTS_CONF_EVT:
		ESP_LOGI(GATTS_TAG, "ESP_GATTS_CONF_EVT, status %d", param->conf.status);
		if (param->conf.status != ESP_GATT_OK){
			esp_log_buffer_hex(GATTS_TAG, param->conf.value, param->conf.len);
		}
		break;
	case ESP_GATTS_OPEN_EVT:
		ESP_LOGI(GATTS_TAG, "ESP_GATTS_OPEN_EVT");
		break;
	case ESP_GATTS_CANCEL_OPEN_EVT:
		ESP_LOGI(GATTS_TAG, "ESP_GATTS_CANCEL_OPEN_EVT");
		break;
	case ESP_GATTS_CLOSE_EVT:
		ESP_LOGI(GATTS_TAG, "ESP_GATTS_CLOSE_EVT");
		break;
	case ESP_GATTS_LISTEN_EVT:
		ESP_LOGI(GATTS_TAG, "ESP_GATTS_LISTEN_EVT");
		break;
	case ESP_GATTS_CONGEST_EVT:
		ESP_LOGI(GATTS_TAG, "ESP_GATTS_CONGEST_EVT");
		break;
	default:
		break;
	}
}

static void gatts_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param)
{
	/* If event is register event, store the gatts_if for each profile */
	if (event == ESP_GATTS_REG_EVT) {
		if (param->reg.status == ESP_GATT_OK) {
			gl_profile_tab[param->reg.app_id].gatts_if = gatts_if;
		} else {
			ESP_LOGI(GATTS_TAG, "Reg app failed, app_id %04x, status %d\n",
					param->reg.app_id,
					param->reg.status);
			return;
		}
	}

	/* If the gatts_if equal to profile A, call profile A cb handler,
	 * so here call each profile's callback */
	do {
		for (int idx = 0; idx < PROFILE_TOTAL_NUM; idx++) {
			if (gatts_if == ESP_GATT_IF_NONE || /* ESP_GATT_IF_NONE, not specify a certain gatt_if, need to call every profile cb function */
					gatts_if == gl_profile_tab[idx].gatts_if) {
				if (gl_profile_tab[idx].gatts_cb) {
					gl_profile_tab[idx].gatts_cb(event, gatts_if, param);
				}
			}
		}
	} while (0);
}

void bt_main(){
	esp_err_t ret;

	// Initialize NVS.
	ret = nvs_flash_init();
	if (ret == ESP_ERR_NVS_NO_FREE_PAGES) {
		ESP_ERROR_CHECK(nvs_flash_erase());
		ret = nvs_flash_init();
	}
	ESP_ERROR_CHECK( ret );
	ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT));

	esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
	ret = esp_bt_controller_init(&bt_cfg);
	if (ret) {
		ESP_LOGE(GATTS_TAG, "%s initialize controller failed: %s\n", __func__, esp_err_to_name(ret));
		return;
	}
	ret = esp_bt_controller_enable(ESP_BT_MODE_BLE);
	if (ret) {
		ESP_LOGE(GATTS_TAG, "%s enable controller failed: %s\n", __func__, esp_err_to_name(ret));
		return;
	}

	ret = esp_bluedroid_init();
	if (ret) {
		ESP_LOGE(GATTS_TAG, "%s init bluetooth failed: %s\n", __func__, esp_err_to_name(ret));
		return;
	}
	ret = esp_bluedroid_enable();
	if (ret) {
		ESP_LOGE(GATTS_TAG, "%s enable bluetooth failed: %s\n", __func__, esp_err_to_name(ret));
		return;
	}

	ret = esp_ble_gatts_register_callback(gatts_event_handler);
	if (ret){
		ESP_LOGE(GATTS_TAG, "gatts register error, error code = %x", ret);
		return;
	}

	ret = esp_ble_gap_register_callback(gap_event_handler);
	if (ret){
		ESP_LOGE(GATTS_TAG, "gap register error, error code = %x", ret);
		return;
	}

	for (int profile = 0; profile < PROFILE_TOTAL_NUM; profile++) {	//register all profiles
		ret = esp_ble_gatts_app_register(profile);
		if (ret){
			ESP_LOGE(GATTS_TAG, "gatts app register error, error code = %x", ret);
			return;
		}
	}

	esp_err_t local_mtu_ret = esp_ble_gatt_set_local_mtu(FIFO_A_FULL *PACKS_TO_SEND + 3);//Define MTU size 60 bytes + 1 byte op_code + 2 byte handle
	if (local_mtu_ret){
		ESP_LOGE(GATTS_TAG, "set local  MTU failed, error code = %x", local_mtu_ret);
	}

	//esp_bt_sleep_enable();
	esp_ble_tx_power_set(9, 0);	//https://dl.espressif.com/doc/esp-idf/latest/api-reference/bluetooth/controller_vhci.html#_CPPv220esp_ble_tx_power_set20esp_ble_power_type_t17esp_power_level_t
	return;
}

uint8_t get_n_notify(){
	n_notify = 0;
	for (int i = 0; i < GATTS_CHAR_NUM_TOTAL; ++i) {
		if(gl_char[i].is_notify){
			n_notify++;
		}
	}
	return n_notify;
}

void print_array(uint8_t *array,uint16_t size){
	for (int  i= 0;  i< size; i++) {
		printf("array[%d]=0x%x\n",i,array[i]);
	}
}



















