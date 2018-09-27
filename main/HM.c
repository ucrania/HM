

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <stdbool.h>

//#include "general_properties.h"
//#include "ble_properties.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "driver/gpio.h"
#include "driver/rtc_io.h"
#include "driver/adc.h"
#include "driver/i2c.h"
#include "esp_adc_cal.h"
#include "esp_intr_alloc.h"
#include "esp_system.h"

#if defined(CONFIG_BT_ENABLED) && defined(CONFIG_BLUEDROID_ENABLED)
#include "esp_bt.h"
#include "esp_gap_ble_api.h"
#include "esp_gap_bt_api.h"
#include "esp_gatts_api.h"
#include "esp_bt_main.h"
#include "esp_gatt_common_api.h"
#endif

#include "esp_log.h"
#include "nvs_flash.h"

#define REG_INTR_STATUS_1 0x00
#define REG_INTR_STATUS_2 0x01
#define REG_INTR_ENABLE_1 0x02
#define REG_INTR_ENABLE_2 0x03
#define REG_FIFO_WR_PTR 0x04
#define REG_OVF_COUNTER 0x05
#define REG_FIFO_RD_PTR 0x06
#define REG_FIFO_DATA 0x07
#define REG_FIFO_CONFIG 0x08
#define REG_MODE_CONFIG 0x09
#define REG_SPO2_CONFIG 0x0A
#define REG_LED1_PA 0x0C
#define REG_LED2_PA 0x0D
#define REG_PILOT_PA 0x10
#define REG_MULTI_LED_CTRL1 0x11
#define REG_MULTI_LED_CTRL2 0x12
#define REG_TEMP_INTR 0x1F
#define REG_TEMP_FRAC 0x20
#define REG_TEMP_CONFIG 0x21
#define REG_PROX_INT_THRESH 0x30
#define REG_REV_ID 0xFE
#define REG_PART_ID 0xFF

#define FIFO_A_FULL_EN		0x80
#define PRG_RDY_EN			0x40
#define ALC_OVF_EN			0x20
#define PROX_INT_EN			0x10



#define FIFO_A_FULL_EN		0x80
#define PRG_RDY_EN		0x40
#define ALC_OVF_EN		0x20
#define PROX_INT_EN		0x10

#define BLINK_GPIO 5
#define PRINT    1

#define I2C_SCL_IO_0           	26               /*!<gpio number for i2c slave clock  */
#define I2C_SDA_IO_0           	25               /*!<gpio number for i2c slave data */
#define I2C_NUM_0			    I2C_NUM_0        /*!<I2C port number for slave dev */

#define I2C_SCL_IO_1         	19               /*!< gpio number for I2C master clock */
#define I2C_SDA_IO_1          	18               /*!< gpio number for I2C master data  */
#define I2C_NUM_1            	I2C_NUM_1        /*!< I2C port number for master dev */
#define I2C_TX_BUF_DISABLE  	0                /*!< I2C master do not need buffer */
#define I2C_RX_BUF_DISABLE  	0                /*!< I2C master do not need buffer */
#define I2C_FREQ_HZ         	400000           /*!< I2C master clock frequency */

#define WRITE_BIT               I2C_MASTER_WRITE /*!< I2C master write */
#define READ_BIT                I2C_MASTER_READ  /*!< I2C master read */
#define ACK_CHECK_EN            0x1              /*!< I2C master will check ack from slave*/
#define ACK_CHECK_DIS           0x0              /*!< I2C master will not check ack from slave */
#define ACK_VAL                 0x0              /*!< I2C ack value */
#define NACK_VAL                0x1              /*!< I2C nack value */

#define MAXREFDES117_ADDR       0x57             /*!< address for MAXREFDES117 sensor */

#define SPO2_RES 				16				//SPO2 ADC resolution 18,17,16,15 bits
#define SPO2_SAMPLE_RATE		50				//Options: 50,100,200,400,800,1000,1600,3600 default:100

#define SMP_AVE 				2				//Options: 1,2,4,8,16,32	default:4
#define FIFO_A_FULL 			30				//Options: 17 - 32			default:17
#define FIFO_ROLLOVER_EN 		1				//Override data in fifo after it is full

#define LED1_CURRENT 			7				//Red led current 0-50mA 0.2mA resolution
#define LED2_CURRENT 			LED1_CURRENT	//IR  led current 0-50mA 0.2mA resolution

#define INT_PIN_0     			34				//RTC GPIO used for interruptions
#define INT_PIN_1     			35				//RTC GPIO used for interruptions
#define GPIO_INPUT_PIN_SEL  ((1ULL<<INT_PIN_0) | (1ULL<<INT_PIN_1))
#define ESP_INTR_FLAG_DEFAULT 0

#define SENSOR_1 "sensor_one"
#define SENSOR_2 "sensor_two"
#define TRESHOLD_ON 40000
#define PLOT

struct SensorData {
	int LED_1;	//RED
	int	LED_2;	//IR
};

void i2c_task_0(void* arg);
void i2c_task_1(void* arg);
void blink_task(void* arg);
void notify_task(void* arg);
void isr_task_manager(void* arg);
void check_ret(int ret, uint8_t sensor_data_h);
esp_err_t max30102_read_reg (uint8_t uch_addr,i2c_port_t i2c_num, uint8_t* data);
esp_err_t max30102_write_reg(uint8_t uch_addr,i2c_port_t i2c_num, uint8_t puch_data);
esp_err_t max30102_read_fifo(i2c_port_t i2c_num, struct SensorData sensorData[]);
static void max30102_init(i2c_port_t i2c_num);
static void max30102_reset(i2c_port_t i2c_num);
static void max30102_shutdown(i2c_port_t i2c_num);
void check_fifo(int ret,uint8_t sensor_data_h, uint8_t sensor_data_m, uint8_t sensor_data_l);
uint8_t get_SPO2_CONF_REG();
uint8_t get_FIFO_CONF_REG();
uint8_t get_LED1_PA();
uint8_t get_LED2_PA();
void intr_init();
double process_data(struct SensorData sensorData[],double *mean1, double *mean2);
void struct_rms(struct SensorData sensorData[],double *rms_l1, double *rms_l2);
void struct_mean(struct SensorData sensorData[],double *mean1, double *mean2);

void bt_main();

static xQueueHandle gpio_evt_queue = NULL;


static void i2c_init(i2c_port_t i2c_num){
	if (i2c_num==I2C_NUM_1){
		i2c_config_t conf;
		conf.mode = I2C_MODE_MASTER;
		conf.sda_io_num = I2C_SDA_IO_1;
		conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
		conf.scl_io_num = I2C_SCL_IO_1;
		conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
		conf.master.clk_speed = I2C_FREQ_HZ;
		i2c_param_config(i2c_num, &conf);
		i2c_driver_install(i2c_num, conf.mode,
				I2C_RX_BUF_DISABLE,
				I2C_TX_BUF_DISABLE, 0);
	} else {
		i2c_config_t conf;
		conf.mode = I2C_MODE_MASTER;
		conf.sda_io_num = I2C_SDA_IO_0;
		conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
		conf.scl_io_num = I2C_SCL_IO_0;
		conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
		conf.master.clk_speed = I2C_FREQ_HZ;
		i2c_param_config(i2c_num, &conf);
		i2c_driver_install(i2c_num, conf.mode,
				I2C_RX_BUF_DISABLE,
				I2C_TX_BUF_DISABLE, 0);
	}
}
static void max30102_shutdown(i2c_port_t i2c_num){
	uint8_t data_h=0x00;
	max30102_read_reg(REG_MODE_CONFIG,i2c_num, &data_h);
	max30102_write_reg(REG_MODE_CONFIG,i2c_num,0x80 + data_h);	//shutdown and keep the same mode
}
static void max30102_reset(i2c_port_t i2c_num){
	uint8_t data_h=0x00;
	max30102_read_reg(REG_INTR_STATUS_1,I2C_NUM_1,&data_h);//clear interrupt pin
	int ret = max30102_read_reg(REG_MODE_CONFIG, i2c_num, &data_h);
	max30102_write_reg(REG_MODE_CONFIG,i2c_num,0x40 + data_h);	//reset and keep the same mode
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
	xTaskCreate(isr_task_manager, "isr_task_manager", 1024 * 2, (void* ) arg, 10, NULL);

	//xQueueSendFromISR(gpio_evt_queue, &gpio_num, NULL);
}
//#define MAX30102

void app_main()
{
#ifdef MAX30102
		printf("Start app_main\n");
		intr_init();
		i2c_init(I2C_NUM_1);
		max30102_reset(I2C_NUM_1);
		max30102_init(I2C_NUM_1);
#endif
		bt_main();
#ifndef CONFIG_BT_ENABLED
		esp_light_sleep_start();
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
	struct SensorData sensorData[FIFO_A_FULL],processed_data[FIFO_A_FULL];
	max30102_read_fifo(I2C_NUM_1, sensorData);
	//memcpy(processed_data,sensorData,sizeof(sensorData));

	double SPO2 = process_data(sensorData,&mean1,&mean2);

	//the size of notify_data[] need less than MTU size
	//esp_ble_gatts_send_indicate(gatts_if, param->write.conn_id, gl_profile_tab[PROFILE_A_APP_ID].char_handle,
			//sizeof(sensorData), sensorData, false);//TODO baata

	//printf("Mean:\t%f,\t%f\n",mean1,mean2);
	//fprintf(stdout,"\tSPO2: %02f%%\n\n",SPO2);
	if(mean1<TRESHOLD_ON && mean2<TRESHOLD_ON){	//IF NO FINGER
		max30102_write_reg(REG_INTR_ENABLE_1,I2C_NUM_1, PROX_INT_EN);	//Enable proximity interruption
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
	//printf("********** ISR_TASK_MANAGER **********\n\n");
	uint8_t data=0x00;
	i2c_port_t port;
	if (arg == INT_PIN_0){
		port = I2C_NUM_1;
	}else{
		port = I2C_NUM_0;
	}
	max30102_read_reg(REG_INTR_STATUS_1,port,&data);

	bool fifo_a_full_int = data>>7 & 0x01;
	bool prox_int = data>>4 & 0x01;
	bool alc_ovf = data>>5 & 0x01;
#ifndef PLOT
	printf("\tINT Reason: 0x%02x\t",data);
	fifo_a_full_int ? printf("\tFIFO A FULL\n") : NULL;
	prox_int 		? printf("\tPROX INT\n") : NULL;
#endif
	if (prox_int){
		max30102_write_reg(REG_INTR_ENABLE_1,port, FIFO_A_FULL_EN);	//0b1000 0000
	}


	//xTaskCreate(i2c_task_0, "i2c_test_task_0", 1024 * 4, (void* ) 0, 10, NULL);
	xTaskCreate(i2c_task_1, "i2c_task_1", 1024 * 4, (void* ) 0, 10, NULL);	//4kB stack size
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
    printf("writing to 0x%02x\n",uch_addr);
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

esp_err_t max30102_read_fifo(i2c_port_t i2c_num, struct SensorData sensorData[FIFO_A_FULL/2])
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

	for(int i=0; i < FIFO_A_FULL/2; i++){//TODO modificar para ler metade das cenas
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
	uint8_t data=0x00;
	for(int i=0; i < FIFO_A_FULL/2; i++){
		sensorData[i].LED_1 = (((LED_1[i][0] &  0b00000011) <<16) + (LED_1[i][1] <<8) + LED_1[i][2])>>(18-SPO2_RES)<<(18-SPO2_RES);	//TODO rever se faz bem as contas
		sensorData[i].LED_2 = (((LED_2[i][0] &  0b00000011) <<16) + (LED_2[i][1] <<8) + LED_2[i][2])>>(18-SPO2_RES)<<(18-SPO2_RES);	//
		//printf("\t%x %x %x\n",LED_1[i][0]&0x03,LED_1[i][1],LED_1[i][2]);
		fprintf(stdout,"%d, %d\n",sensorData[i].LED_1 ,sensorData[i].LED_2);
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
		printf("TASK[%d]  MASTER READ SENSOR( MAX30102 )\n", 0);
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
		printf("READING FIFO ( MAX30102 )\n");
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
	 gpio_isr_handler_add(INT_PIN_0, gpio_isr_handler, (void*) INT_PIN_1);	//hook isr handler for specific gpio pin
	 gpio_isr_handler_add(INT_PIN_1, gpio_isr_handler, (void*) INT_PIN_0);	//hook isr handler for specific gpio pin


	 esp_sleep_enable_ext1_wakeup(GPIO_INPUT_PIN_SEL,ESP_EXT1_WAKEUP_ALL_LOW);
}

double process_data(struct SensorData sensorData[],double *mean1, double *mean2){
	double rms1, rms2, dc1, dc2, R,SPO2;
	struct_rms(sensorData,&rms1,&rms2);
	struct_mean(sensorData,&dc1,&dc2);
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

void struct_rms(struct SensorData sensorData[],double *rms_1, double *rms_2){
	double dataLength = FIFO_A_FULL/2;
	double sum_led1=0 , sum_led2 =0;
	for (int i = 0; i < dataLength; ++i) {
		sum_led1 +=(sensorData[i].LED_1)^2;
		sum_led2 +=(sensorData[i].LED_2)^2;
	}

	*rms_1= sqrt((1/dataLength)*sum_led1);
	*rms_2= sqrt((1/dataLength)*sum_led2);
	return;
}

void struct_mean(struct SensorData sensorData[],double *mean1, double *mean2){
	double dataLength = FIFO_A_FULL/2;
	double sum1=0 ,sum2=0 ;
	for (int i = 0; i < dataLength; ++i) {
		sum1 += sensorData[i].LED_1;
		sum2 += sensorData[i].LED_2;
	}
	*mean1 = sum1/dataLength;
	*mean2 = sum2/dataLength;
}

/*********************BLE PART**************************************************************************************/
static struct notify_struct {
		esp_gatt_if_t gatts_if;
		esp_ble_gatts_cb_param_t param;
		uint16_t char_handle;
}notify_task_data;
static TaskHandle_t xHandle;


static void gatts_profile_a_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param);
static void gatts_profile_b_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param);
static void gatts_profile_c_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param);
static void gatts_profile_d_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param);

void char1_read_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param);
void char1_write_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param);
void char2_read_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param);
void char2_write_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param);
void char3_read_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param);
void char3_write_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param);
void char4_read_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param);
void char4_write_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param);
void char5_read_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param);
void char5_write_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param);
void char6_read_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param);
void char6_write_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param);

void descr1_read_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param);
void descr1_write_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param);
void descr2_read_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param);
void descr2_write_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param);
void descr3_read_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param);
void descr3_write_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param);
void descr4_read_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param);
void descr4_write_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param);
void descr5_read_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param);
void descr5_write_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param);
void descr6_read_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param);
void descr6_write_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param);

#define GATTS_TAG 					"GATT_Server"
#define TEST_DEVICE_NAME            "HM_BLE_Koval" /*"HM_BLE_Calil"*/

#define GATTS_SERVICE_UUID_HEART_RATE   0x180D 	//Heart Rate Service uuid
#define GATTS_CHAR_UUID_A      0x2A37	//Heart Rate Measurement characteristic uuid
#define GATTS_CHAR2_UUID_A      0x2A38	//Body sensor location   characteristic uuid
#define GATTS_DESCR_UUID_A     0x2901	//
#define GATTS_NUM_HANDLE_A     7

#define GATTS_SERVICE_UUID_PULSE_OXIMETER   0x1822 	//Pulse Oximeter Service uuid
#define GATTS_CHAR_UUID_B       0x2A5E	//PLX Spot-Check Measurement characteristic uuid
#define GATTS_DESCR_UUID_B     0x2901	//
#define GATTS_NUM_HANDLE_B     4

#define TEST_MANUFACTURER_DATA_LEN  17

#define GATTS_DEMO_CHAR_VAL_LEN_MAX 0x40

#define PREPARE_BUF_MAX_SIZE 1024

//****Body Location defines****//
#define OTHER		0
#define CHEST		1
#define WRIST		2
#define FINGER		3
#define HAND		4
#define EAR_LOBE	5
#define FOOT		6

//****Heart rate Measurment Service FLAG defines****//
#define HEART_RATE_8BIT 						0x00	//
#define HEART_RATE_16BIT 						0x01	//Heart rate value with 16bit
#define SENSOR_CONTACT_NOT_SUPPORTED 			0x02	//
#define SENSOR_CONTACT_NOT_DETECTED 			0x04	//
#define SENSOR_CONTACT_DETECTED		 			0x06	//
#define ENERGY_EXPENDED_NOT_PRESENT 			0x00	//
#define ENERGY_EXPENDED_PRESENT 				0x08	//in Kilojoule
#define RR_INTERVAL_NOT_PRESENT 				0x00	//
#define RR_INTERVAL_PRESENT 					0x016	//16bit for RR-interval 1/1024 Resolution

#define CHAR1_FLAGS			HEART_RATE_8BIT | SENSOR_CONTACT_NOT_SUPPORTED | ENERGY_EXPENDED_NOT_PRESENT | RR_INTERVAL_NOT_PRESENT
#define CHAR4_FLAGS			HEART_RATE_8BIT | SENSOR_CONTACT_NOT_SUPPORTED | ENERGY_EXPENDED_NOT_PRESENT | RR_INTERVAL_NOT_PRESENT

//*****Pulse oximeter FLAG defines *****//
#define SPO2PS_FAST_PRESENT						0x01	//
#define SPO2PS_SLOW_PRESENT						0x02	//
#define MEASURMENT_STATUS_PRESENT 				0x04	//
#define DEVICE_AND_SENSOR_STATUS_PRESENT		0x08	//
#define PULSE_AMPLITUDE_INDEX_PRESENT			0x10	//

#define CHAR3_FLAGS			0x0
#define CHAR6_FLAGS			0x0

uint8_t char1_str[] = {CHAR1_FLAGS,80};//,13551%255,13551>>8};	//Heart Rate
uint8_t char2_str[] = {FINGER}; 				//Body Location:
uint8_t char3_str[] = {CHAR3_FLAGS,99,0,11,0};					//Pulse Measurement , value: 99 , 11-> data for testing
uint8_t char4_str[] = {CHAR4_FLAGS,70};//,18500%255,18500>>8};	//Heart Rate
uint8_t char5_str[] = {WRIST}; 					//Body Location:
uint8_t char6_str[] = {CHAR6_FLAGS,98,0,10,0};					//Pulse Measurement

uint8_t descr1_str[] = {0,0};
uint8_t descr2_str[] = {0,0};
uint8_t descr3_str[] = {0,0};
uint8_t descr4_str[] = {0,0};
uint8_t descr5_str[] = {0,0};
uint8_t descr6_str[] = {0,0};



esp_attr_value_t gatts_demo_char1_val = {
	.attr_max_len = 22,
	.attr_len		= sizeof(char1_str),
	.attr_value     = char1_str,
};

esp_attr_value_t gatts_demo_char2_val = {
	.attr_max_len = 22,
	.attr_len		= sizeof(char2_str),
	.attr_value     = char2_str,
};

esp_attr_value_t gatts_demo_char3_val = {
	.attr_max_len = 22,
	.attr_len		= sizeof(char3_str),
	.attr_value     = char3_str,
};

esp_attr_value_t gatts_demo_char4_val = {
	.attr_max_len = 22,
	.attr_len		= sizeof(char4_str),
	.attr_value     = char4_str,
};

esp_attr_value_t gatts_demo_char5_val = {
	.attr_max_len = 22,
	.attr_len		= sizeof(char5_str),
	.attr_value     = char5_str,
};

esp_attr_value_t gatts_demo_char6_val = {
	.attr_max_len = 22,
	.attr_len		= sizeof(char6_str),
	.attr_value     = char6_str,
};

//***************DESCRIPTORS**********************//
esp_attr_value_t gatts_demo_descr1_val = {
	.attr_max_len = 22,
	.attr_len		= sizeof(descr1_str),
	.attr_value     = descr1_str,
};

esp_attr_value_t gatts_demo_descr2_val = {
	.attr_max_len = 22,
	.attr_len		= sizeof(descr2_str),
	.attr_value     = descr2_str,
};

esp_attr_value_t gatts_demo_descr3_val = {
	.attr_max_len = 22,
	.attr_len		= sizeof(descr3_str),
	.attr_value     = descr3_str,
};

esp_attr_value_t gatts_demo_descr4_val = {
	.attr_max_len = 22,
	.attr_len		= sizeof(descr4_str),
	.attr_value     = descr4_str,
};

esp_attr_value_t gatts_demo_descr5_val = {
	.attr_max_len = 22,
	.attr_len		= sizeof(descr5_str),
	.attr_value     = descr5_str,
};

esp_attr_value_t gatts_demo_descr6_val = {
	.attr_max_len = 22,
	.attr_len		= sizeof(descr6_str),
	.attr_value     = descr6_str,
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
    .min_interval = 0x05,	//0x20
    .max_interval = 0x10, 	//0x40
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

#define PROFILE_NUM 4		//TOTAL Profile number
#define PROFILE_A_APP_ID 0
#define PROFILE_B_APP_ID 1
#define PROFILE_C_APP_ID 2
#define PROFILE_D_APP_ID 3

#define GATTS_CHAR_NUM_A		2	//HR 	CHAR
#define GATTS_CHAR_NUM_B		1	//PULSE CHAR
#define GATTS_CHAR_NUM			(GATTS_CHAR_NUM_A + GATTS_CHAR_NUM_B)*2 	//TOTAL CHAR x2 sensors

static uint32_t ble_add_char_pos;



#define EN_NOTIFY_0
#define EN_NOTIFY_1
#define EN_NOTIFY_2
#define EN_NOTIFY_3

uint8_t char1_test_str[] = {0xE,1,123,0,13551>>8,13551&0x0F};	//Heart Rate
void notify_task( void* arg) {
#ifndef PLOT
	printf("\tNotify TASK!\n");
#endif
	uint8_t descr_aux[6];
printf("Profile %d\n", (uint8_t)arg);
	switch ((int)arg) {
	case PROFILE_A_APP_ID:
		memcpy(descr_aux,char1_test_str,sizeof(char1_test_str));
		break;
	case PROFILE_B_APP_ID:
		memcpy(descr_aux,char2_str,sizeof(char2_str));
		break;
	case PROFILE_C_APP_ID:
		memcpy(descr_aux,char3_str,sizeof(char3_str));
		break;
	case PROFILE_D_APP_ID:
		memcpy(descr_aux,char4_str,sizeof(char4_str));
		break;
	default:
		break;
	}


	for (int i = (int)arg*10; i < (int)arg*10 + 11; i++) { //infinite loop
		descr_aux[1] = i % 255;
		printf("\tSent: %d\n",descr_aux[1]);
		printf("\tConn_id: %d\n",notify_task_data.param.write.conn_id);
		printf("\tchar_handle: %d\n",notify_task_data.char_handle);
		esp_ble_gatts_send_indicate(notify_task_data.gatts_if, notify_task_data.param.write.conn_id, notify_task_data.char_handle,
				sizeof(descr_aux),descr_aux , true);
		vTaskDelay(1000 / portTICK_RATE_MS); // delay 1s
	}
	//esp_ble_gatts_send_indicate(gatts_if, param->write.conn_id, gl_profile_tab[profile].char_handle,sizeof(char1_str), char1_str, false);
	vTaskDelete(NULL);
}

struct gatts_profile_inst {
    esp_gatts_cb_t gatts_cb;
    uint16_t gatts_if;
    uint16_t app_id;
    uint16_t conn_id;
    uint16_t service_handle;
    uint16_t char_num;
    uint16_t char_num_total;
    esp_gatt_srvc_id_t service_id;
    uint16_t char_handle;
    esp_bt_uuid_t char_uuid;
    esp_gatt_perm_t perm;
    esp_gatt_char_prop_t property;
    uint16_t descr_handle;
    esp_bt_uuid_t descr_uuid;
};

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
};

static struct gatts_char_inst gl_char[] = {
		{		//Hearth rate Measurement Characteristic
				.char_uuid.len = ESP_UUID_LEN_16, // RX
				.char_uuid.uuid.uuid16 =  0x2A37,
				.char_perm = ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
				.char_property = ESP_GATT_CHAR_PROP_BIT_READ | ESP_GATT_CHAR_PROP_BIT_WRITE | ESP_GATT_CHAR_PROP_BIT_NOTIFY,
				.char_val = &gatts_demo_char1_val,
				.char_control = NULL,
				.char_handle = 0,
				.char_read_callback=char1_read_handler,
				.char_write_callback=char1_write_handler,
				.descr_uuid.len = ESP_UUID_LEN_16,
				.descr_uuid.uuid.uuid16 = ESP_GATT_UUID_CHAR_CLIENT_CONFIG,
				.descr_perm=ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
				.descr_val = &gatts_demo_descr1_val,
				.descr_control=NULL,
				.descr_handle=0,
				.descr_read_callback=descr1_read_handler,
				.descr_write_callback=descr1_write_handler
		},
		{		//Body Location Characteristic
				.char_uuid.len = ESP_UUID_LEN_16,  // TX
				.char_uuid.uuid.uuid16 =  0x2A38,
				.char_perm = ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
				.char_property = ESP_GATT_CHAR_PROP_BIT_READ | ESP_GATT_CHAR_PROP_BIT_WRITE,
				.char_val = &gatts_demo_char2_val,
				.char_control=NULL,
				.char_handle=0,
				.char_read_callback=char2_read_handler,
				.char_write_callback=char2_write_handler,
				.descr_uuid.len = 0,
				.descr_uuid.uuid.uuid16 = ESP_GATT_UUID_CHAR_CLIENT_CONFIG, // ESP_GATT_UUID_CHAR_DESCRIPTION,  0x2902
				.descr_perm=ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
				.descr_val = &gatts_demo_descr2_val,
				.descr_control=NULL,
				.descr_handle=0,
				.descr_read_callback=descr2_read_handler,
				.descr_write_callback=descr2_write_handler
		},
		{		//PLX Spot-check measurement
				.char_uuid.len = ESP_UUID_LEN_16,  // TX
				.char_uuid.uuid.uuid16 =   0x2A37,
				.char_perm = ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
				.char_property = ESP_GATT_CHAR_PROP_BIT_READ | ESP_GATT_CHAR_PROP_BIT_WRITE | ESP_GATT_CHAR_PROP_BIT_NOTIFY,
				.char_val = &gatts_demo_char3_val,
				.char_control=NULL,
				.char_handle=0,
				.char_read_callback=char3_read_handler,
				.char_write_callback=char3_write_handler,
				.descr_uuid.len = ESP_UUID_LEN_16,
				.descr_uuid.uuid.uuid16 = ESP_GATT_UUID_CHAR_CLIENT_CONFIG, // ESP_GATT_UUID_CHAR_DESCRIPTION,
				.descr_perm=ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
				.descr_val = &gatts_demo_descr3_val,
				.descr_control=NULL,
				.descr_handle=0,
				.descr_read_callback=descr3_read_handler,
				.descr_write_callback=descr3_write_handler
		},
		{		//Hearth rate Measurement Characteristic
				.char_uuid.len = ESP_UUID_LEN_16, // RX
				.char_uuid.uuid.uuid16 =  0x2A37,
				.char_perm = ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
				.char_property = ESP_GATT_CHAR_PROP_BIT_READ | ESP_GATT_CHAR_PROP_BIT_WRITE | ESP_GATT_CHAR_PROP_BIT_NOTIFY,
				.char_val = &gatts_demo_char4_val,
				.char_control = NULL,
				.char_handle = 0,
				.char_read_callback=char4_read_handler,
				.char_write_callback=char4_write_handler,
				.descr_uuid.len = ESP_UUID_LEN_16,
				.descr_uuid.uuid.uuid16 = ESP_GATT_UUID_CHAR_CLIENT_CONFIG,
				.descr_perm=ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
				.descr_val = &gatts_demo_descr4_val,
				.descr_control=NULL,
				.descr_handle=0,
				.descr_read_callback=descr4_read_handler,
				.descr_write_callback=descr4_write_handler
		},
		{		//Body Location Characteristic
				.char_uuid.len = ESP_UUID_LEN_16,  // TX
				.char_uuid.uuid.uuid16 =  0x2A38,
				.char_perm = ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
				.char_property = ESP_GATT_CHAR_PROP_BIT_READ | ESP_GATT_CHAR_PROP_BIT_WRITE ,
				.char_val = &gatts_demo_char5_val,
				.char_control=NULL,
				.char_handle=0,
				.char_read_callback=char5_read_handler,
				.char_write_callback=char5_write_handler,
				.descr_uuid.len = 0,
				.descr_uuid.uuid.uuid16 = ESP_GATT_UUID_CHAR_CLIENT_CONFIG, // ESP_GATT_UUID_CHAR_DESCRIPTION,
				.descr_perm=ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
				.descr_val = &gatts_demo_descr5_val,
				.descr_control=NULL,
				.descr_handle=0,
				.descr_read_callback=descr5_read_handler,
				.descr_write_callback=descr5_write_handler
		},
		{		//PLX Spot-check measurement
				.char_uuid.len = ESP_UUID_LEN_16,  // TX
				.char_uuid.uuid.uuid16 =   0x2A5F,
				.char_perm = ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
				.char_property = ESP_GATT_CHAR_PROP_BIT_READ | ESP_GATT_CHAR_PROP_BIT_WRITE | ESP_GATT_CHAR_PROP_BIT_NOTIFY,
				.char_val = &gatts_demo_char6_val,
				.char_control=NULL,
				.char_handle=0,
				.char_read_callback=char6_read_handler,
				.char_write_callback=char6_write_handler,
				.descr_uuid.len = ESP_UUID_LEN_16,
				.descr_uuid.uuid.uuid16 = ESP_GATT_UUID_CHAR_CLIENT_CONFIG, // ESP_GATT_UUID_CHAR_DESCRIPTION,
				.descr_perm=ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
				.descr_val = &gatts_demo_descr6_val,
				.descr_control=NULL,
				.descr_handle=0,
				.descr_read_callback=descr6_read_handler,
				.descr_write_callback=descr6_write_handler
		}
};

static uint16_t notify_conn_id = 0;
static esp_gatt_if_t notify_gatts_if = NULL;
static uint8_t notify_pos=0;
static uint8_t is_notify=0;

void char1_read_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param) {
	uint8_t char_num = 0;
	ESP_LOGI(GATTS_TAG, "char%d_read_handler %d\n",char_num ,param->read.handle);
	esp_gatt_rsp_t rsp;
	memset(&rsp, 0, sizeof(esp_gatt_rsp_t));
	rsp.attr_value.handle = param->read.handle;
	if (gl_char[char_num].char_val!=NULL) {
		ESP_LOGI(GATTS_TAG, "char1_read_handler char_val %d\n",gl_char[char_num].char_val->attr_len);
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
		ESP_LOGI(GATTS_TAG, "char1_read_handler char_val %d\n",gl_char[char_num].char_val->attr_len);
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
		ESP_LOGI(GATTS_TAG, "char1_read_handler char_val %d\n",gl_char[char_num].char_val->attr_len);
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
		ESP_LOGI(GATTS_TAG, "char1_read_handler char_val %d\n",gl_char[char_num].char_val->attr_len);
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
		ESP_LOGI(GATTS_TAG, "char1_read_handler char_val %d\n",gl_char[char_num].char_val->attr_len);
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
		ESP_LOGI(GATTS_TAG, "char1_read_handler char_val %d\n",gl_char[char_num].char_val->attr_len);
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
	if (is_notify==1) {
		notify_pos='0';
		for (uint32_t i=0;i<10;i++) {
			ESP_LOGI(GATTS_TAG, "char%d_notify_handle esp_ble_gatts_send_indicate\n",char_num);
			vTaskDelay(1000 / portTICK_RATE_MS); // delay 1s
			esp_ble_gatts_send_indicate(gatts_if, conn_id, gl_char[char_num].char_handle,1,&notify_pos,false);
			notify_pos++;
		}
	}
}
void char2_notify_handle(esp_gatt_if_t gatts_if, uint16_t conn_id) {
	uint8_t char_num = 1;
	ESP_LOGI(GATTS_TAG, "char%d_notify_handler \n",char_num);
	if (is_notify==1) {
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
	if (is_notify==1) {
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
	if (is_notify==1) {
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
	if (is_notify==1) {
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
	if (is_notify==1) {
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
	notify_gatts_if = gatts_if;
	notify_conn_id = param->write.conn_id;
    esp_ble_gatts_send_response(gatts_if, param->write.conn_id, param->write.trans_id, ESP_GATT_OK, NULL);
    if (strncmp((const char *)gl_char[char_num].char_val->attr_value,"LED ON",6)==0) {
    } else if (strncmp((const char *)gl_char[char_num].char_val->attr_value,"LED OFF",7)==0) {
    } else if (strncmp((const char *)gl_char[char_num].char_val->attr_value,"LED SWITCH",10)==0) {
    } else {
    	//char1_notify_handle(gatts_if, param->write.conn_id);
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
	notify_gatts_if = gatts_if;
	notify_conn_id = param->write.conn_id;
    esp_ble_gatts_send_response(gatts_if, param->write.conn_id, param->write.trans_id, ESP_GATT_OK, NULL);
    if (strncmp((const char *)gl_char[char_num].char_val->attr_value,"LED ON",6)==0) {
    } else if (strncmp((const char *)gl_char[char_num].char_val->attr_value,"LED OFF",7)==0) {
    } else if (strncmp((const char *)gl_char[char_num].char_val->attr_value,"LED SWITCH",10)==0) {
    } else {
    	//char2_notify_handle(gatts_if, param->write.conn_id);
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
	notify_gatts_if = gatts_if;
	notify_conn_id = param->write.conn_id;
    esp_ble_gatts_send_response(gatts_if, param->write.conn_id, param->write.trans_id, ESP_GATT_OK, NULL);
    if (strncmp((const char *)gl_char[char_num].char_val->attr_value,"LED ON",6)==0) {
    } else if (strncmp((const char *)gl_char[char_num].char_val->attr_value,"LED OFF",7)==0) {
    } else if (strncmp((const char *)gl_char[char_num].char_val->attr_value,"LED SWITCH",10)==0) {
    } else {
    	//char3_notify_handle(gatts_if, param->write.conn_id);
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
	notify_gatts_if = gatts_if;
	notify_conn_id = param->write.conn_id;
    esp_ble_gatts_send_response(gatts_if, param->write.conn_id, param->write.trans_id, ESP_GATT_OK, NULL);
    if (strncmp((const char *)gl_char[char_num].char_val->attr_value,"LED ON",6)==0) {
    } else if (strncmp((const char *)gl_char[char_num].char_val->attr_value,"LED OFF",7)==0) {
    } else if (strncmp((const char *)gl_char[char_num].char_val->attr_value,"LED SWITCH",10)==0) {
    } else {
    	char1_notify_handle(gatts_if, param->write.conn_id);
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
	notify_gatts_if = gatts_if;
	notify_conn_id = param->write.conn_id;
    esp_ble_gatts_send_response(gatts_if, param->write.conn_id, param->write.trans_id, ESP_GATT_OK, NULL);
    if (strncmp((const char *)gl_char[char_num].char_val->attr_value,"LED ON",6)==0) {
    } else if (strncmp((const char *)gl_char[char_num].char_val->attr_value,"LED OFF",7)==0) {
    } else if (strncmp((const char *)gl_char[char_num].char_val->attr_value,"LED SWITCH",10)==0) {
    } else {
    	char1_notify_handle(gatts_if, param->write.conn_id);
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
	notify_gatts_if = gatts_if;
	notify_conn_id = param->write.conn_id;
    esp_ble_gatts_send_response(gatts_if, param->write.conn_id, param->write.trans_id, ESP_GATT_OK, NULL);
    if (strncmp((const char *)gl_char[char_num].char_val->attr_value,"LED ON",6)==0) {
    } else if (strncmp((const char *)gl_char[char_num].char_val->attr_value,"LED OFF",7)==0) {
    } else if (strncmp((const char *)gl_char[char_num].char_val->attr_value,"LED SWITCH",10)==0) {
    } else {
    	//char1_notify_handle(gatts_if, param->write.conn_id);
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

static struct gatts_profile_inst gl_profile_tab[] = {
		[PROFILE_A_APP_ID] = {
				.char_num =0,
				.char_num_total = GATTS_CHAR_NUM_A,
				.gatts_cb = gatts_profile_a_event_handler,
				.gatts_if = ESP_GATT_IF_NONE,       /* Not get the gatt_if, so initial is ESP_GATT_IF_NONE */
		},
		[PROFILE_B_APP_ID] = {
				.char_num =0,
				.char_num_total = GATTS_CHAR_NUM_B,
				.gatts_cb = gatts_profile_b_event_handler,
				.gatts_if = ESP_GATT_IF_NONE,       /* Not get the gatt_if, so initial is ESP_GATT_IF_NONE */
		},
		[PROFILE_C_APP_ID] = {
				.char_num =0,
				.char_num_total = GATTS_CHAR_NUM_A,
				.gatts_cb = gatts_profile_c_event_handler,
				.gatts_if = ESP_GATT_IF_NONE,       /* Not get the gatt_if, so initial is ESP_GATT_IF_NONE */
		},
		[PROFILE_D_APP_ID] = {
				.char_num =0,
				.char_num_total = GATTS_CHAR_NUM_B,
				.gatts_cb = gatts_profile_d_event_handler,
				.gatts_if = ESP_GATT_IF_NONE,       /* Not get the gatt_if, so initial is ESP_GATT_IF_NONE */
		},
};

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

	for (uint32_t pos=0;pos<GATTS_CHAR_NUM;pos++) {
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

	ESP_LOGI(GATTS_TAG, "gatts_check_callback read %d num %d handle %d\n", read, GATTS_CHAR_NUM, handle);
	for (uint32_t pos=0;pos<GATTS_CHAR_NUM;pos++) {
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
		ESP_LOGI(GATTS_TAG, "descr_handle (%d) in pos %d \n\t\tLooking for handle %d\n", gl_char[pos].descr_handle ,pos, handle);
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
    uint8_t profile = PROFILE_A_APP_ID;
	switch (event) {
    case ESP_GATTS_REG_EVT:

        ESP_LOGI(GATTS_TAG, "REGISTER_APP_EVT, status %d, app_id %d\n", param->reg.status, param->reg.app_id);
        gl_profile_tab[profile].service_id.is_primary = true;
        gl_profile_tab[profile].service_id.id.inst_id = 0x00;
        gl_profile_tab[profile].service_id.id.uuid.len = ESP_UUID_LEN_16;
        gl_profile_tab[profile].service_id.id.uuid.uuid.uuid16 = GATTS_SERVICE_UUID_HEART_RATE;

        esp_ble_gatts_create_service(gatts_if, &gl_profile_tab[profile].service_id, GATTS_NUM_HANDLE_A);

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

						notify_task_data.char_handle = gl_profile_tab[profile].char_handle;
						notify_task_data.gatts_if = gatts_if;
						notify_task_data.param = *param;

						esp_ble_gatts_send_indicate(gatts_if, param->write.conn_id, gl_profile_tab[profile].char_handle,sizeof(char1_str), char1_str, false);
#ifdef EN_NOTIFY_0
						xTaskCreate(notify_task, "notify_task", 1024 * 2, (void*) profile, 10, xHandle);
#endif
						//the size of notify_data[] need less than MTU size
    					/*esp_ble_gatts_send_indicate(gatts_if, param->write.conn_id, gl_profile_tab[profile].char_handle,
    							sizeof(char1_str), char1_str, false);
    					esp_ble_gatts_send_indicate(gatts_if, param->write.conn_id, gl_profile_tab[profile].char_handle,
    							sizeof(char1_str), char1_str, false); //false for notify*/
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
						esp_ble_gatts_send_indicate(gatts_if, param->write.conn_id, gl_profile_tab[profile].char_handle,
								sizeof(indicate_data), indicate_data, true);	//true for indicate ()
    				}
    			}
    			else if (descr_value == 0x0000){
    				ESP_LOGI(GATTS_TAG, "notify/indicate disable ");
    				//vTaskDelete(xHandle);
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
        gl_profile_tab[profile].char_uuid.uuid.uuid16 = gl_char[0].char_uuid.uuid.uuid16;

        esp_ble_gatts_start_service(gl_profile_tab[profile].service_handle);
        gatts_add_char(profile);
        break;
    case ESP_GATTS_ADD_INCL_SRVC_EVT:
        break;
    case ESP_GATTS_ADD_CHAR_EVT: {

        ESP_LOGI(GATTS_TAG, "ADD_CHAR_EVT, status 0x%X,  attr_handle %d, service_handle %d\n",
                param->add_char.status, param->add_char.attr_handle, param->add_char.service_handle);

        if (param->add_char.status==ESP_GATT_OK) {
        	gl_profile_tab[profile].char_handle = param->add_char.attr_handle;
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
        esp_ble_gap_start_advertising(&adv_params);
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
	uint8_t profile = PROFILE_B_APP_ID;
	switch (event) {
    case ESP_GATTS_REG_EVT:

        ESP_LOGI(GATTS_TAG, "REGISTER_APP_EVT, status %d, app_id %d\n", param->reg.status, param->reg.app_id);
        gl_profile_tab[profile].service_id.is_primary = true;
        gl_profile_tab[profile].service_id.id.inst_id = 0x00;
        gl_profile_tab[profile].service_id.id.uuid.len = ESP_UUID_LEN_16;
        gl_profile_tab[profile].service_id.id.uuid.uuid.uuid16 = GATTS_SERVICE_UUID_PULSE_OXIMETER;

        esp_ble_gatts_create_service(gatts_if, &gl_profile_tab[profile].service_id, GATTS_NUM_HANDLE_B);
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

                        notify_task_data.char_handle = gl_profile_tab[profile].char_handle;
                        notify_task_data.gatts_if = gatts_if;
                        notify_task_data.param = *param;
                        //the size of notify_data[] need less than MTU size
                        //esp_ble_gatts_send_indicate(gatts_if, param->write.conn_id, gl_profile_tab[profile].char_handle, sizeof(notify_data), notify_data, false);
#ifdef EN_NOTIFY_1
						xTaskCreate(notify_task, "notify_task", 1024 * 2, (void*) profile, 10, xHandle);
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
                        esp_ble_gatts_send_indicate(gatts_if, param->write.conn_id, gl_profile_tab[profile].char_handle,
                                                sizeof(indicate_data), indicate_data, true);
                    }
                }
                else if (descr_value == 0x0000){
                    ESP_LOGI(GATTS_TAG, "notify/indicate disable ");
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
        	gl_profile_tab[profile].char_handle = param->add_char.attr_handle;
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
	uint8_t profile = PROFILE_C_APP_ID;
	switch (event) {
    case ESP_GATTS_REG_EVT:

        ESP_LOGI(GATTS_TAG, "REGISTER_APP_EVT, status %d, app_id %d\n", param->reg.status, param->reg.app_id);
        gl_profile_tab[profile].service_id.is_primary = true;
        gl_profile_tab[profile].service_id.id.inst_id = 0x00;
        gl_profile_tab[profile].service_id.id.uuid.len = ESP_UUID_LEN_16;
        gl_profile_tab[profile].service_id.id.uuid.uuid.uuid16 = GATTS_SERVICE_UUID_HEART_RATE;

        esp_ble_gatts_create_service(gatts_if, &gl_profile_tab[profile].service_id, GATTS_NUM_HANDLE_A);

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

                        notify_task_data.char_handle = gl_profile_tab[profile].char_handle;
                        notify_task_data.gatts_if = gatts_if;
                        notify_task_data.param = *param;
                        //the size of notify_data[] need less than MTU size
                        //esp_ble_gatts_send_indicate(gatts_if, param->write.conn_id, gl_profile_tab[profile].char_handle,sizeof(notify_data), notify_data, false);
#ifdef EN_NOTIFY_2
						xTaskCreate(notify_task, "notify_task", 1024 * 2, (void*) profile, 10, xHandle);
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
                        esp_ble_gatts_send_indicate(gatts_if, param->write.conn_id, gl_profile_tab[profile].char_handle,
                                                sizeof(indicate_data), indicate_data, true);
                    }
                }
                else if (descr_value == 0x0000){
                    ESP_LOGI(GATTS_TAG, "notify/indicate disable ");
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
        gl_profile_tab[profile].char_uuid.uuid.uuid16 = gl_char[0].char_uuid.uuid.uuid16;

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
        	gl_profile_tab[profile].char_handle = param->add_char.attr_handle;
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
	uint8_t profile = PROFILE_D_APP_ID;
	switch (event) {
    case ESP_GATTS_REG_EVT:

        ESP_LOGI(GATTS_TAG, "REGISTER_APP_EVT, status %d, app_id %d\n", param->reg.status, param->reg.app_id);
        gl_profile_tab[profile].service_id.is_primary = true;
        gl_profile_tab[profile].service_id.id.inst_id = 0x00;
        gl_profile_tab[profile].service_id.id.uuid.len = ESP_UUID_LEN_16;
        gl_profile_tab[profile].service_id.id.uuid.uuid.uuid16 = GATTS_SERVICE_UUID_PULSE_OXIMETER;

        esp_ble_gatts_create_service(gatts_if, &gl_profile_tab[profile].service_id, GATTS_NUM_HANDLE_B);
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

                        notify_task_data.char_handle = gl_profile_tab[profile].char_handle;
                        notify_task_data.gatts_if = gatts_if;
                        notify_task_data.param = *param;

                        //the size of notify_data[] need less than MTU size
                        //esp_ble_gatts_send_indicate(gatts_if, param->write.conn_id, gl_profile_tab[profile].char_handle, sizeof(notify_data), notify_data, false);
#ifdef EN_NOTIFY_3
						xTaskCreate(notify_task, "notify_task", 1024 * 2, (void*) profile, 10, xHandle);
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
                        esp_ble_gatts_send_indicate(gatts_if, param->write.conn_id, gl_profile_tab[profile].char_handle,
                                                sizeof(indicate_data), indicate_data, true);
                    }
                }
                else if (descr_value == 0x0000){
                    ESP_LOGI(GATTS_TAG, "notify/indicate disable ");
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
            gl_profile_tab[profile].char_handle = param->add_char.attr_handle;
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
        int idx;
        for (idx = 0; idx < PROFILE_NUM; idx++) {
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
	ret = esp_ble_gatts_app_register(PROFILE_A_APP_ID);
	if (ret){
		ESP_LOGE(GATTS_TAG, "gatts app register error, error code = %x", ret);
		return;
	}
    ret = esp_ble_gatts_app_register(PROFILE_B_APP_ID);
    if (ret){
        ESP_LOGE(GATTS_TAG, "gatts app register error, error code = %x", ret);
        return;
    }
    ret = esp_ble_gatts_app_register(PROFILE_C_APP_ID);
	if (ret){
		ESP_LOGE(GATTS_TAG, "gatts app register error, error code = %x", ret);
		return;
	}
    ret = esp_ble_gatts_app_register(PROFILE_D_APP_ID);
    if (ret){
        ESP_LOGE(GATTS_TAG, "gatts app register error, error code = %x", ret);
        return;
    }
	esp_err_t local_mtu_ret = esp_ble_gatt_set_local_mtu(500);
	if (local_mtu_ret){
		ESP_LOGE(GATTS_TAG, "set local  MTU failed, error code = %x", local_mtu_ret);
	}

	//esp_bt_sleep_enable();
	esp_ble_tx_power_set(9, 7);	//https://dl.espressif.com/doc/esp-idf/latest/api-reference/bluetooth/controller_vhci.html#_CPPv220esp_ble_tx_power_set20esp_ble_power_type_t17esp_power_level_t
	return;
}

//---------------- algorithm_by_RF -----------
/*
void rf_heart_rate_and_oxygen_saturation(uint32_t *pun_ir_buffer, int32_t n_ir_buffer_length, uint32_t *pun_red_buffer, float *pn_spo2, int8_t *pch_spo2_valid,
                int32_t *pn_heart_rate, int8_t *pch_hr_valid, float *ratio, float *correl, String i2c_sensor)

* \brief        Calculate the heart rate and SpO2 level, Robert Fraczkiewicz version
* \par          Details
*               By detecting peaks of PPG cycle and corresponding AC/DC of red/infra-red signal, the xy_ratio for the SPO2 is computed.
*
* \param[in]    *pun_ir_buffer            - IR sensor data buffer
* \param[in]    n_ir_buffer_length        - IR sensor data buffer length
* \param[in]    *pun_red_buffer           - Red sensor data buffer
* \param[out]   *pn_spo2                  - Calculated SpO2 value
* \param[out]   *pch_spo2_valid           - 1 if the calculated SpO2 value is valid
* \param[out]   *pn_heart_rate            - Calculated heart rate value
* \param[out]   *pch_hr_valid             - 1 if the calculated heart rate value is valid
*
* \retval       None

{
  int32_t k;
  static int32_t n_last_peak_interval=25; //FS; // Initialize it to 25, which corresponds to heart rate of 60 bps, RF
  float f_ir_mean,f_red_mean,f_ir_sumsq,f_red_sumsq;
  float f_y_ac, f_x_ac, xy_ratio;
  float beta_ir, beta_red, x;
  float an_x[BUFFER_SIZE], *ptr_x; //ir
  float an_y[BUFFER_SIZE], *ptr_y; //red

  // calculates DC mean and subtracts DC from ir and red
  f_ir_mean=0.0;
  f_red_mean=0.0;
  for (k=0; k<n_ir_buffer_length; ++k) {
    f_ir_mean += pun_ir_buffer[k];
    f_red_mean += pun_red_buffer[k];
  }
  f_ir_mean=f_ir_mean/n_ir_buffer_length ;
  f_red_mean=f_red_mean/n_ir_buffer_length ;

  // remove DC
  for (k=0,ptr_x=an_x,ptr_y=an_y; k<n_ir_buffer_length; ++k,++ptr_x,++ptr_y) {
    *ptr_x = pun_ir_buffer[k] - f_ir_mean;
    *ptr_y = pun_red_buffer[k] - f_red_mean;
  }

  // RF, remove linear trend (baseline leveling)
  beta_ir = rf_linear_regression_beta(an_x, mean_X, sum_X2);
  beta_red = rf_linear_regression_beta(an_y, mean_X, sum_X2);
  for(k=0,x=-mean_X,ptr_x=an_x,ptr_y=an_y; k<n_ir_buffer_length; ++k,++x,++ptr_x,++ptr_y) {
    *ptr_x -= beta_ir*x;
    *ptr_y -= beta_red*x;
  }

  // For SpO2 calculate RMS of both AC signals. In addition, pulse detector needs raw sum of squares for IR
  f_y_ac=rf_rms(an_y,n_ir_buffer_length,&f_red_sumsq);
  f_x_ac=rf_rms(an_x,n_ir_buffer_length,&f_ir_sumsq);

  // Calculate Pearson correlation between red and IR
  *correl=rf_Pcorrelation(an_x, an_y, n_ir_buffer_length)/sqrt(f_red_sumsq*f_ir_sumsq);
  if(*correl>=min_pearson_correlation) {
    // RF, If correlation os good, then find average periodicity of the IR signal. If aperiodic, return periodicity of 0
    rf_signal_periodicity(an_x, BUFFER_SIZE, &n_last_peak_interval, LOWEST_PERIOD, HIGHEST_PERIOD, min_autocorrelation_ratio, f_ir_sumsq, ratio, i2c_sensor); // adicionei parametro do i2c_sensor para verificar qual esta a ser usado
  } else n_last_peak_interval=0;
  if(n_last_peak_interval!=0) {
    *pn_heart_rate = (int32_t)(FS60/n_last_peak_interval);
    *pch_hr_valid  = 1;
  } else {
    n_last_peak_interval=25; //FS;
    *pn_heart_rate = -999; // unable to calculate because signal looks aperiodic
    *pch_hr_valid  = 0;
    *pn_spo2 =  -999 ; // do not use SPO2 from this corrupt signal
    *pch_spo2_valid  = 0;
    return;
  }

  // After trend removal, the mean represents DC level
  xy_ratio= (f_y_ac*f_ir_mean)/(f_x_ac*f_red_mean);  //formula is (f_y_ac*f_x_dc) / (f_x_ac*f_y_dc) ;
  if(xy_ratio>0.02 && xy_ratio<1.84) { // Check boundaries of applicability
    *pn_spo2 = (-45.060*xy_ratio + 30.354)*xy_ratio + 94.845;
    *pch_spo2_valid = 1;
  } else {
    *pn_spo2 =  -999 ; // do not use SPO2 since signal an_ratio is out of range
    *pch_spo2_valid  = 0;
  }
}

float rf_linear_regression_beta(float *pn_x, float xmean, float sum_x2)

{
  float x,beta,*pn_ptr;
  beta=0.0;
  for(x=-xmean,pn_ptr=pn_x;x<=xmean;++x,++pn_ptr)
    beta+=x*(*pn_ptr);
  return beta/sum_x2;
}

float rf_autocorrelation(float *pn_x, int32_t n_size, int32_t n_lag)
{
  int16_t i, n_temp=n_size-n_lag;
  float sum=0.0,*pn_ptr;
  if(n_temp<=0) return sum;
  for (i=0,pn_ptr=pn_x; i<n_temp; ++i,++pn_ptr) {
    sum += (*pn_ptr)*(*(pn_ptr+n_lag));
  }
  return sum/n_temp;
}

#define NUM_PEAKS 101
int32_t buffer_distance_peak_sensor_1[NUM_PEAKS];	// Sensor 1 distance peak data
int32_t buffer_distance_peak_sensor_2[NUM_PEAKS];	// Sensor 2 distance peak date
int8_t numExtI = 0;	// i used to calc peak
int i = 1;
void rf_signal_periodicity(float *pn_x, int32_t n_size, int32_t *p_last_periodicity, int32_t n_min_distance, int32_t n_max_distance, float min_aut_ratio, float aut_lag0, float *ratio, String i2c_sensor)

* \brief        Signal periodicity
* \par          Details
*               Finds periodicity of the IR signal which can be used to calculate heart rate.
*               Makes use of the autocorrelation function. If peak autocorrelation is less
*               than min_aut_ratio fraction of the autocorrelation at lag=0, then the input
*               signal is insufficiently periodic and probably indicates motion artifacts.
*               Robert Fraczkiewicz, 01/07/2018
* \retval       Average distance between peaks

{
  int32_t n_lag;
  float aut,aut_left,aut_right,aut_save;
  bool left_limit_reached=false;
  // Start from the last periodicity computing the corresponding autocorrelation
  n_lag=*p_last_periodicity;
  aut_save=aut=rf_autocorrelation(pn_x, n_size, n_lag);
  // Is autocorrelation one lag to the left greater?
  aut_left=aut;
  do {
    aut=aut_left;
    n_lag--;
    aut_left=rf_autocorrelation(pn_x, n_size, n_lag);
  } while(aut_left>aut && n_lag>n_min_distance);
  // Restore lag of the highest aut
  if(n_lag==n_min_distance) {
    left_limit_reached=true;
    n_lag=*p_last_periodicity;
    aut=aut_save;
  } else n_lag++;
  if(n_lag==*p_last_periodicity) {
    // Trip to the left made no progress. Walk to the right.
    aut_right=aut;
    do {
      aut=aut_right;
      n_lag++;
      aut_right=rf_autocorrelation(pn_x, n_size, n_lag);
    } while(aut_right>aut && n_lag<n_max_distance);
    // Restore lag of the highest aut
    if(n_lag==n_max_distance) n_lag=0; // Indicates failure
    else n_lag--;
    if(n_lag==*p_last_periodicity && left_limit_reached) n_lag=0; // Indicates failure
  }
  *ratio=aut/aut_lag0;
  if(*ratio < min_aut_ratio) n_lag=0; // Indicates failure
  *p_last_periodicity=n_lag;


  if (i > 10) {
    if (i2c_sensor == SENSOR_1) {
    	buffer_distance_peak_sensor_1[i] = *p_last_periodicity;
  //    Serial.print("distancePeakSensorOne ");Serial.println(distancePeakSensorOne[i]);
      numExtI = i;
    }  else if (i2c_sensor == SENSOR_2) {
    	buffer_distance_peak_sensor_2[i] = *p_last_periodicity;
  //    Serial.print("distancePeakSensorTwo ");Serial.println(distancePeakSensorTwo[i]);
      numExtI = i;
    }
  }

  i++;
  if (i == NUM_PEAKS) {
    if (i2c_sensor == SENSOR_1) {
      memset(buffer_distance_peak_sensor_1, 0, sizeof(buffer_distance_peak_sensor_1));
    } else if (i2c_sensor == SENSOR_2) {
      memset(buffer_distance_peak_sensor_2, 0, sizeof(buffer_distance_peak_sensor_2));
    }
  }

}


float rf_rms(float *pn_x, int32_t n_size, float *sumsq)

{
  int16_t i;
  float r,*pn_ptr;
  (*sumsq)=0.0;
  for (i=0,pn_ptr=pn_x; i<n_size; ++i,++pn_ptr) {
    r=(*pn_ptr);
    (*sumsq) += r*r;
  }
  (*sumsq)/=n_size; // This corresponds to autocorrelation at lag=0
  return sqrt(*sumsq);
}

float rf_Pcorrelation(float *pn_x, float *pn_y, int32_t n_size)

{
  int16_t i;
  float r,*x_ptr,*y_ptr;
  r=0.0;
  for (i=0,x_ptr=pn_x,y_ptr=pn_y; i<n_size; ++i,++x_ptr,++y_ptr) {
    r+=(*x_ptr)*(*y_ptr);
  }
  r/=n_size;
  return r;
}

*/


















