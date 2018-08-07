//test

#include <stdio.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/adc.h"
#include "esp_adc_cal.h"

#include "esp_intr_alloc.h"
//#include "max30102.h"
#include "driver/i2c.h"
//#include "max30105.h"
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


#define BLINK_GPIO 5
#define PRINT    1

#define I2C_SCL_IO_0           	26               /*!<gpio number for i2c slave clock  */
#define I2C_SDA_IO_0           	25               /*!<gpio number for i2c slave data */
#define I2C_NUM_0			    I2C_NUM_0_        /*!<I2C port number for slave dev */

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

#define SPO2_RES 				18				//SPO2 ADC resolution 18,17,16,15 bits
#define SPO2_SAMPLE_RATE		200				//Options: 50,100,200,400,800,1000,1600,3600 default:100

#define SMP_AVE 				32				//Options: 1,2,4,8,16,32	default:4
#define FIFO_A_FULL 			30				//Options: 17 - 32			default:17

#define INT_PIN_0     4
#define INT_PIN_1     5
#define GPIO_INPUT_PIN_SEL  ((1ULL<<INT_PIN_0) | (1ULL<<INT_PIN_1))
#define ESP_INTR_FLAG_DEFAULT 0

#define MY_LER_FIFO

struct SensorData {
	int LED_1;	//RED
	int	LED_2;	//IR
};

void i2c_task_0(void* arg);
void i2c_task_1(void* arg);
void blink_task(void* arg);
void check_ret(int ret, uint8_t sensor_data_h, uint8_t sensor_data_l);
esp_err_t maxim_max30102_read_reg (uint8_t uch_addr,i2c_port_t i2c_num, uint8_t* data_h, uint8_t* data_l);
esp_err_t maxim_max30102_write_reg(uint8_t uch_addr,i2c_port_t i2c_num, uint8_t puch_data);
esp_err_t maxim_max30102_read_fifo(i2c_port_t i2c_num, struct SensorData sensorData[]);
void check_fifo(int ret,uint8_t sensor_data_h, uint8_t sensor_data_m, uint8_t sensor_data_l);
uint8_t get_SPO2_CONF_REG();
uint8_t get_FIFO_CONF_REG();
void intr_init();

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
	uint8_t data_h=0x00, data_l=0x00;
	maxim_max30102_read_reg(REG_MODE_CONFIG,i2c_num, &data_h, &data_l);
	maxim_max30102_write_reg(REG_MODE_CONFIG,i2c_num,0x80 + data_h);	//shutdown and keep the same mode
}
static void max30102_reset(i2c_port_t i2c_num){
	uint8_t data_h=0x00, data_l=0x00;
	int ret = maxim_max30102_read_reg(REG_MODE_CONFIG, i2c_num, &data_h, &data_l);
	//printf("data_h: %02x\n data_l: %02x\nwriting %02x\n",data_h,data_l,0x40 + data_h);
	maxim_max30102_write_reg(REG_MODE_CONFIG,i2c_num,0x40 + data_h);	//reset and keep the same mode
}
static void max30102_init(i2c_port_t i2c_num){
	printf("***MAX30102 initialization***\n");

	maxim_max30102_write_reg(REG_INTR_ENABLE_1,i2c_num,0x80);	//0b1100 0000
	maxim_max30102_write_reg(REG_INTR_ENABLE_2,i2c_num,0x00);	//Temperature interrupt
	maxim_max30102_write_reg(REG_FIFO_WR_PTR,i2c_num,0x00);		//
	maxim_max30102_write_reg(REG_OVF_COUNTER,i2c_num,0x00);		//
	maxim_max30102_write_reg(REG_FIFO_RD_PTR,i2c_num,0x00);		//
	maxim_max30102_write_reg(REG_FIFO_CONFIG,i2c_num,get_FIFO_CONF_REG());		//4 average sample + FIFO_A_FULL 15
	maxim_max30102_write_reg(REG_MODE_CONFIG,i2c_num,0x03);		//0x07 -> Multi-LED  //0x03 -> SpO2 mode //0x02 -> HR mode
	maxim_max30102_write_reg(REG_SPO2_CONFIG,i2c_num,get_SPO2_CONF_REG());		//01 adc range + 100 samples/sec + 18  bit resolution
	maxim_max30102_write_reg(REG_LED1_PA,i2c_num,0x24);			//RED LED current
	maxim_max30102_write_reg(REG_LED2_PA,i2c_num,0x24);			//IR  LED current
	maxim_max30102_write_reg(REG_PILOT_PA,i2c_num,0x7f);		//Multimode registers (not used)
}

static void IRAM_ATTR gpio_isr_handler(void* arg){
	uint32_t gpio_num = (uint32_t) arg;
	xTaskCreate(i2c_task_1, "i2c_test_task_1", 1024 * 2, (void* ) 0, 10, NULL);
	//xQueueSendFromISR(gpio_evt_queue, &gpio_num, NULL);
}

void app_main()
{
		printf("Start app_main\n");
		intr_init();
		i2c_init(I2C_NUM_1);
		max30102_reset(I2C_NUM_1);
		max30102_init(I2C_NUM_1);
		//xTaskCreate(i2c_task_0, "i2c_test_task_0", 1024 * 2, (void* ) 0, 10, NULL);


#ifdef MY_LER_FIFO

		uint32_t io_num;
		/*while(1){
			maxim_max30102_read_fifo(I2C_NUM_1, &sensorData);

		}*/

#endif

#ifdef MY_LED_TEST
		int k = 0;
		while (1){
		printf("Valor: %02x\n",valor);
		maxim_max30102_write_reg(REG_LED1_PA,I2C_NUM_1,valor);		//RED Led
		maxim_max30102_write_reg(REG_LED2_PA,I2C_NUM_1,valor);	//IR Led
		valor+=k;
		if (valor+(1*k) == 0xdf)
			k=-1;
		else if(valor+(1*k) == 0x00){
			k=1;
		}
		//max30102_shutdown();

		//maxim_max30102_read_reg(REG_LED1_PA,I2C_NUM_1, data_h, data_l);
		//vTaskDelay(pdMS_TO_TICKS(1000));
		}
#endif

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

	uint8_t sensor_data_h, sensor_data_l;
	while(1){
		//ret = i2c_example_master_sensor_test( I2C_NUM_1, &sensor_data_h, &sensor_data_l);
		ret = maxim_max30102_read_reg(REG_LED1_PA, I2C_NUM_1, &sensor_data_h, &sensor_data_l);
		check_ret(ret,sensor_data_h,sensor_data_l);
		vTaskDelay(pdMS_TO_TICKS(1000));
	}
	vTaskDelete(NULL);

}

void i2c_task_1(void* arg)
{
	printf("Start task 1!\n");
	struct SensorData sensorData[FIFO_A_FULL];
	maxim_max30102_read_fifo(I2C_NUM_1, &sensorData);
	vTaskDelete(NULL);

}

void blink_task(void* arg)
{
	gpio_pad_select_gpio(BLINK_GPIO);
	gpio_set_direction(BLINK_GPIO, GPIO_MODE_OUTPUT); // @suppress("Symbol is not resolved")
	printf("Start blinking\n");
	int led = 1,i=0;

	while(1){
		//printf("Blink\t%d\n",i++);
		gpio_set_level(BLINK_GPIO, led=!led);
		vTaskDelay(pdMS_TO_TICKS(1000));
	}
	vTaskDelete(NULL);

}


esp_err_t maxim_max30102_write_reg(uint8_t uch_addr, i2c_port_t i2c_num, uint8_t puch_data){

	i2c_cmd_handle_t cmd = i2c_cmd_link_create();
	i2c_master_start(cmd);
	i2c_master_write_byte(cmd, MAXREFDES117_ADDR << 1 | WRITE_BIT, ACK_CHECK_EN);
	i2c_master_write_byte(cmd, uch_addr, ACK_CHECK_EN);
	i2c_master_write_byte(cmd, puch_data, ACK_CHECK_EN);
    i2c_master_stop(cmd);
    int ret = i2c_master_cmd_begin(i2c_num, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
    printf("writing to 0x%02x\n",uch_addr);
    //check_ret(ret,0,0);

	return ret;
}

esp_err_t maxim_max30102_read_reg(uint8_t uch_addr,i2c_port_t i2c_num, uint8_t* data_h, uint8_t* data_l)
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
	        printf("ESP NOT OK\n");
	    	return ret;
	    }
	    vTaskDelay(50 / portTICK_RATE_MS);
	    cmd = i2c_cmd_link_create();
	    i2c_master_start(cmd);
	    i2c_master_write_byte(cmd, MAXREFDES117_ADDR << 1 | READ_BIT, ACK_CHECK_EN);
	    i2c_master_read_byte(cmd, data_h, ACK_VAL);
	    i2c_master_read_byte(cmd, data_l, NACK_VAL);
	    i2c_master_stop(cmd);
	    ret = i2c_master_cmd_begin(i2c_num, cmd, 100 / portTICK_RATE_MS);
	    i2c_cmd_link_delete(cmd);
	    return ret;
}

esp_err_t maxim_max30102_read_fifo(i2c_port_t i2c_num, struct SensorData sensorData[FIFO_A_FULL])
{

	uint8_t LED_1[FIFO_A_FULL][3],LED_2[FIFO_A_FULL][3];

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

	for(int i=0; i < FIFO_A_FULL-1; i++){
		i2c_master_read_byte(cmd, &LED_1[i][0], ACK_VAL);
		i2c_master_read_byte(cmd, &LED_1[i][1], ACK_VAL);
		i2c_master_read_byte(cmd, &LED_1[i][2], ACK_VAL);

		i2c_master_read_byte(cmd, &LED_2[i][0], ACK_VAL);
		i2c_master_read_byte(cmd, &LED_2[i][1], ACK_VAL);
		i2c_master_read_byte(cmd, &LED_2[i][2], i==FIFO_A_FULL-1? NACK_VAL: ACK_VAL);	//NACK_VAL on the last iteration

		sensorData[i].LED_1 = (((LED_1[i][0] && 0b00000011) <<16) + (LED_1[i][1] <<8) + LED_1[i][2])>>(18-SPO2_RES);	//TODO rever se faz bem as contas
		sensorData[i].LED_2 = (((LED_2[i][0] && 0b00000011) <<16) + (LED_2[i][1] <<8) + LED_2[i][2])>>(18-SPO2_RES);	//
		printf("%d, %d\n",sensorData[i].LED_1 ,sensorData[i].LED_2);
	}
		i2c_master_stop(cmd);
		ret = i2c_master_cmd_begin(i2c_num, cmd, 100 / portTICK_RATE_MS);
		i2c_cmd_link_delete(cmd);

		/*int data1 = (((LED_1[i][0] && 0b00000011) <<16) + (LED_1[i][1] <<8) + LED_1[i][2])>>(18-SPO2_RES);
		/int data2 = (((LED_2[i][0] && 0b00000011) <<16) + (LED_2[i][1] <<8) + LED_2[i][2])>>(18-SPO2_RES);	*/
		uint8_t data_h=0x00,data_l=0x00;
		maxim_max30102_read_reg(REG_INTR_STATUS_1,I2C_NUM_1,&data_h,&data_l);//clear interruplt pin
		printf("\t INT: 0x%02x\n",data_h);
		vTaskDelay(6000 / portTICK_RATE_MS);//magic delay

		return ret;
}

void check_ret(int ret,uint8_t sensor_data_h, uint8_t sensor_data_l){
	if(ret == ESP_ERR_TIMEOUT) {
		printf("I2C timeout\n");
	} else if(ret == ESP_OK) {
		printf("******************* \n");
		printf("TASK[%d]  MASTER READ SENSOR( MAX30102 )\n", 0);
		printf("*******************\n");
		printf("data_h: %02x\n", sensor_data_h);
		printf("data_l: %02x\n", sensor_data_l);
		printf("sensor val: %f\n", (sensor_data_h << 8 | sensor_data_l) / 1.2);
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
#define FIFO_A_FULL 17
#endif
	fifo_a_full = 32-FIFO_A_FULL;

	//printf("REGISTER is: 0x%x\n",smp_ave<<5 | fifo_rollover_en<<4 | fifo_a_full);
	return smp_ave<<5 | fifo_rollover_en<<4 | fifo_a_full;
}

void intr_init(){
	 gpio_config_t io_conf;
	 io_conf.intr_type = GPIO_PIN_INTR_NEGEDGE;		//interrupt of falling edge
	 io_conf.pin_bit_mask = GPIO_INPUT_PIN_SEL;		//bit mask of the pins, use GPIO4/5 here
	 io_conf.mode = GPIO_MODE_INPUT;				//set as input mode
	 io_conf.pull_down_en = 0;						//disable pull-down mode
	 io_conf.pull_up_en = 1;						//enable pull-up mode
	 gpio_config(&io_conf);
	 gpio_set_intr_type(INT_PIN_0, GPIO_INTR_NEGEDGE);	//interrupt of falling edge
	 gpio_set_intr_type(INT_PIN_1, GPIO_INTR_NEGEDGE);	//interrupt of falling edge


	 gpio_install_isr_service(ESP_INTR_FLAG_DEFAULT);		//install gpio isr service
	 //gpio_isr_handler_add(INT_PIN_0, gpio_isr_handler, (void*) INT_PIN_0);	//hook isr handler for specific gpio pin
	 gpio_isr_handler_add(INT_PIN_1, gpio_isr_handler, (void*) INT_PIN_1);	//hook isr handler for specific gpio pin
	 //gpio_isr_handler_add(INT_PIN_0, gpio_isr_handler, (void*) INT_PIN_0);	//hook isr handler for specific gpio pin
}


























