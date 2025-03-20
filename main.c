/* main.c - Application main entry point */

/*
 * Copyright (c) 2015-2016 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/types.h>
#include <stddef.h>
#include <zephyr/sys/printk.h>
#include <zephyr/sys/util.h>
#include <stdbool.h>
#include <zephyr/kernel.h>

#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/hci.h>

//Define I2C
#include <zephyr/drivers/i2c.h>

//From L6 E1, P.026 = SDA and P.027 = SCL

#define I2C0_NODE DT_NODELABEL(mysensor)

#define DEVICE_NAME CONFIG_BT_DEVICE_NAME
#define DEVICE_NAME_LEN (sizeof(DEVICE_NAME) - 1)

#define LED_DRIVE 			0x74	//bit 7 -0 off 1 on, bit 0:6 LED driving strength. 000 0000: 4mA 000 0001: 6mA 000 0010: 8mA

#define AS7341_ASTATUS_ 	0x94   // AS7341_ASTATUS, same as 0x60 (unused)
#define AS7341_STATUS2 		0xA3   // Measurement status flags; saturation, validity
#define ENABLE_REG 			0x80   // bit 4 sets SMUXEN (cleared when smux command is finished), bit 1 
#define AGAIN				0xAA   // Set gain value
#define ATIME 				0x81   // set number of integration steps 1 to 256 = ASTEP x (n+1)
#define ASTEP_L 			0xCA   // Integration step size low byte
#define ASTEP_H 			0xCB   // Integration step size high byte		
#define AS7341_CFG6 		0xAF   // Used to configure Smux
#define AS7341_CFG1 		0xAA   // Controls ADC Gain		
#define CONFIG				0x70	//Reg to set config nodes
#define REG_AS7341_CFG_0	0XA9
#define REG_AS7341_CFG_1	0XAA
#define REG_AS7341_GPIO_2	0XBE
#define REG_AS7341_INTENAB	0XF9

#define f4_left 			0x05	//Set f4 to ADC 3
#define f4_right 			0x0D
#define f7_left				0x07	//Set f7 to ADC 2
#define f7_right			0x0A

#define AS7341_CH0_DATA_L 0x95 ///< ADC Channel Data
#define AS7341_CH0_DATA_H 0x96 ///< ADC Channel Data
#define AS7341_CH1_DATA_L 0x97 ///< ADC Channel Data
#define AS7341_CH1_DATA_H 0x98 ///< ADC Channel Data
#define AS7341_CH2_DATA_L 0x99 ///< ADC Channel Data
#define AS7341_CH2_DATA_H 0x9A ///< ADC Channel Data
#define AS7341_CH3_DATA_L 0x9B ///< ADC Channel Data
#define AS7341_CH3_DATA_H 0x9C ///< ADC Channel Data
#define AS7341_CH4_DATA_L 0x9D ///< ADC Channel Data
#define AS7341_CH4_DATA_H 0x9E ///< ADC Channel Data
#define AS7341_CH5_DATA_L 0x9F ///< ADC Channel Data
#define AS7341_CH5_DATA_H 0xA0 ///< ADC Channel Data
#define AS7341_LED 0x74    ///< LED Register; Enables and sets current limit

//#define CH2_DATA_L			0x99	//f7
//#define CH2_DATA_H			0x9A
//#define CH3_DATA_L			0x9B	//f4
//#define CH3_DATA_H			0x9C
#define REG_AS7341_CH0_DATA_L  0X95
#define REG_AS7341_CH0_DATA_H  0X96

  typedef enum {
    eSpm = 0,/**<SPM>*/
    eSyns = 1,/**<SYNS*/
    eSynd = 3,/**<SYND>*/
    
  }eMode_t;

/*
	writeRegister(byte(0x05), byte(0x42)); // F4 left connected to ADC3/f2 left connected to ADC1
    writeRegister(byte(0x0D), byte(0x04)); // F4 right connected to ADC3
	writeRegister(byte(0x07), byte(0x03)); // F7 left connected to ADC2
	writeRegister(byte(0x0A), byte(0x03)); // F7 right connected to ADC2
*/


/*
 * Set Advertisement data. Based on the Eddystone specification:
 * https://github.com/google/eddystone/blob/master/protocol-specification.md
 * https://github.com/google/eddystone/tree/master/eddystone-url
 */
static const struct bt_data ad[] = {
	BT_DATA_BYTES(BT_DATA_FLAGS, BT_LE_AD_NO_BREDR),
	BT_DATA_BYTES(BT_DATA_UUID16_ALL, 0xaa, 0xfe),
	BT_DATA_BYTES(BT_DATA_SVC_DATA16,
		      0xaa, 0xfe, /* Eddystone UUID */
		      0x10, /* Eddystone-URL frame type */
		      0x00, /* Calibrated Tx power at 0m */
		      0x00, /* URL Scheme Prefix http://www. */
		      'z', 'e', 'p', 'h', 'y', 'r',
		      'p', 'r', 'o', 'j', 'e', 'c', 't',
		      0x08) /* .org */
};

static const struct i2c_dt_spec dev_i2c = I2C_DT_SPEC_GET(I2C0_NODE);

/* Set Scan Response data */
static const struct bt_data sd[] = {
	BT_DATA(BT_DATA_NAME_COMPLETE, DEVICE_NAME, DEVICE_NAME_LEN),
};

void enableAS7341(bool on)
{
	uint8_t ret;
	uint8_t enable[2] = {ENABLE_REG,0};
	ret = i2c_write_read_dt(&dev_i2c,&enable[0],1,&enable[1],1);
	if(ret != 0){
		printk("Failed to write/read I2C device address %x at Reg. %x \r\n", dev_i2c.addr,enable[0]);
	}
	if(on == true){
		enable[1] = enable[1] | (1<<0);
	} else {
		enable[1] = enable[1] & (~1);
	}
	i2c_write_dt(&dev_i2c,enable,sizeof(enable));
}
void enableSpectralMeasure(bool on)
{
	uint8_t ret;
	uint8_t spec[2] = {ENABLE_REG,0};
	ret = i2c_write_read_dt(&dev_i2c,&spec[0],1,&spec[1],1);
	if(ret != 0){
		printk("Failed to write/read I2C device address %x at Reg. %x \r\n", dev_i2c.addr,spec[0]);
	}
	if(on == true){
		spec[1] = spec[1] | (1<<1);
	} else {
		spec[1] = spec[1] & (~(1<<1));
	}
	i2c_write_dt(&dev_i2c,spec,sizeof(spec));
}

void enableSMUX(bool on){
	uint8_t ret;
	uint8_t smux[2] = {ENABLE_REG,0};
	ret = i2c_write_read_dt(&dev_i2c,&smux[0],1,&smux[1],1);
	if(ret != 0){
		printk("Failed to write/read I2C device address %x at Reg. %x \r\n", dev_i2c.addr,smux[0]);
	}
	if(on == true){
		smux[1] = smux[1] | (1<<4);
	} else {
		smux[1] = smux[1] & (~(1<<4));
	}
	i2c_write_dt(&dev_i2c,smux,sizeof(smux));
}

void setBank(uint8_t temp){
	uint8_t ret;
	uint8_t bank[2] = {REG_AS7341_CFG_0,0};
	ret = i2c_write_read_dt(&dev_i2c,&bank[0],1,&bank[1],1);
	if(ret != 0){
		printk("Failed to write/read I2C device address %x at Reg. %x \r\n", dev_i2c.addr,bank[0]);
	}
	if(temp == 1){
	
		bank[1] = bank[1] | (1<<4);
	}
	
	if(temp == 0){
	
		bank[1] = bank[1] & (~(1<<4));
	}
	i2c_write_dt(&dev_i2c,bank,sizeof(bank));
}

void config(eMode_t mode)
{
	setBank(1);
	uint8_t ret;
	uint8_t config[2] = {CONFIG,0};
	ret = i2c_write_read_dt(&dev_i2c,&config[0],1,&config[1],1);
	if(ret != 0){
		printk("Failed to write/read I2C device address %x at Reg. %x \r\n", dev_i2c.addr,config[0]);
	}
	//switch(mode){
	//	case eSpm : {
	config[1] = (config[1] & (~3)) | eSpm;
	//	};
	//	break;
	//	case eSyns : {
	//	data = (data & (~3)) | eSyns;
	//	};
	//	break;
	//	case eSynd : {
	//	data = (data & (~3)) | eSynd;
	//	};
	//	break;
	//	default : break;
	//}
	i2c_write_dt(&dev_i2c,config,sizeof(config));
	setBank(0);
}

bool measureComplete(){
	uint8_t data = 0;
	uint8_t ret;
	ret = i2c_reg_read_byte_dt(&dev_i2c, AS7341_STATUS2, &data);
	if(ret != 0){
		printk("Failed to write/read I2C device address %x at Reg. %x \r\n", dev_i2c.addr,AS7341_STATUS2);
	}
	if((data & (1<<6))){
		return true;
	}
	else{
		return false;
	}
}

void startMeasure()
{
	uint8_t ret;
	uint8_t meas[2] = {REG_AS7341_CFG_0,0};
	ret = i2c_write_read_dt(&dev_i2c,&meas[0],1,&meas[1],1);
	if(ret != 0){
		printk("Failed to write/read I2C device address %x at Reg. %x \r\n", dev_i2c.addr,meas[0]);
	}
	meas[1] = meas[1] & (~(1<<4));
	i2c_write_dt(&dev_i2c,meas,sizeof(meas));
	enableSpectralMeasure(false);
	i2c_reg_write_byte_dt(&dev_i2c, AS7341_CFG6, 0x10);
  	//Set up ADC channel to filter
	//if(channel==1){
	i2c_reg_write_byte_dt(&dev_i2c, f4_left, 0x42);
	i2c_reg_write_byte_dt(&dev_i2c, f4_right, 0x04);
		/*
		ret = i2c_reg_write_byte_dt(&dev_i2c,0x00, 0x30); 
		if(ret != 0){
			printk("Failed to write/read I2C device address %x at Reg. %x \r\n", dev_i2c.addr,0x00);
		}
		i2c_reg_write_byte_dt(&dev_i2c,0x01, 0x01); 
		i2c_reg_write_byte_dt(&dev_i2c,0x02, 0x00); 
		i2c_reg_write_byte_dt(&dev_i2c,0x03, 0x00); 
		i2c_reg_write_byte_dt(&dev_i2c,0x04, 0x00); 
		i2c_reg_write_byte_dt(&dev_i2c,0x05, 0x42); 
		i2c_reg_write_byte_dt(&dev_i2c,0x06, 0x00); 
		i2c_reg_write_byte_dt(&dev_i2c,0x07, 0x00); 
		i2c_reg_write_byte_dt(&dev_i2c,0x08, 0x50); 
		i2c_reg_write_byte_dt(&dev_i2c,0x09, 0x00); 
		i2c_reg_write_byte_dt(&dev_i2c,0x0A, 0x00); 
		i2c_reg_write_byte_dt(&dev_i2c,0x0B, 0x00); 
		i2c_reg_write_byte_dt(&dev_i2c,0x0C, 0x20); 
		i2c_reg_write_byte_dt(&dev_i2c,0x0D, 0x04); 
		i2c_reg_write_byte_dt(&dev_i2c,0x0E, 0x00); 
		i2c_reg_write_byte_dt(&dev_i2c,0x0F, 0x30); 
		i2c_reg_write_byte_dt(&dev_i2c,0x10, 0x01); 
		i2c_reg_write_byte_dt(&dev_i2c,0x11, 0x50); 
		i2c_reg_write_byte_dt(&dev_i2c,0x12, 0x00); 
		i2c_reg_write_byte_dt(&dev_i2c,0x13, 0x06); */
	//}
	//if (channel ==2){
	i2c_reg_write_byte_dt(&dev_i2c, f7_left, 0x13);
	i2c_reg_write_byte_dt(&dev_i2c, f7_right, 0x13);
		/*i2c_reg_write_byte_dt(&dev_i2c,0x00, 0x00); 
		i2c_reg_write_byte_dt(&dev_i2c,0x01, 0x00); 
		i2c_reg_write_byte_dt(&dev_i2c,0x02, 0x00); 
		i2c_reg_write_byte_dt(&dev_i2c,0x03, 0x40); 
		i2c_reg_write_byte_dt(&dev_i2c,0x04, 0x02); 
		i2c_reg_write_byte_dt(&dev_i2c,0x05, 0x00); 
		i2c_reg_write_byte_dt(&dev_i2c,0x06, 0x10); 
		i2c_reg_write_byte_dt(&dev_i2c,0x07, 0x03); 
		i2c_reg_write_byte_dt(&dev_i2c,0x08, 0x50); 
		i2c_reg_write_byte_dt(&dev_i2c,0x09, 0x10); 
		i2c_reg_write_byte_dt(&dev_i2c,0x0A, 0x03); 
		i2c_reg_write_byte_dt(&dev_i2c,0x0B, 0x00); 
		i2c_reg_write_byte_dt(&dev_i2c,0x0C, 0x00); 
		i2c_reg_write_byte_dt(&dev_i2c,0x0D, 0x00); 
		i2c_reg_write_byte_dt(&dev_i2c,0x0E, 0x24); 
		i2c_reg_write_byte_dt(&dev_i2c,0x0F, 0x00); 
		i2c_reg_write_byte_dt(&dev_i2c,0x10, 0x00); 
		i2c_reg_write_byte_dt(&dev_i2c,0x11, 0x50); 
		i2c_reg_write_byte_dt(&dev_i2c,0x12, 0x00); 
		i2c_reg_write_byte_dt(&dev_i2c,0x13, 0x06); */
	//};
  	enableSMUX(true);
	config(eSpm);
  	enableSpectralMeasure(true);
  	while(!measureComplete()){
    	k_sleep(K_MSEC(1));	//usec to wait
  		}
}


uint16_t getChannelData(uint8_t low_channel, uint8_t high_channel){
	uint8_t data[2]={0};
	uint16_t channelData = 0x0000;
	uint8_t ret;
	//ret = i2c_burst_read_dt(&dev_i2c,low_channel, data, sizeof(data));
	ret = i2c_reg_read_byte_dt(&dev_i2c,low_channel,&data[0]); //alter to burst read to ensure propper reading
	if(ret != 0){
		printk("Failed to write/read I2C device address %x at Reg. %x \r\n", dev_i2c.addr,low_channel);
	}
	ret = i2c_reg_read_byte_dt(&dev_i2c,high_channel,&data[1]);
	//if(ret != 0){
	//	printk("Failed to write/read I2C device address %x at Reg. %x \r\n", dev_i2c.addr,high_channel);
	//}
	channelData = data[1];
	channelData = (channelData<<8) | data[0];
	k_sleep(K_MSEC(50));	//usec to wait
	return channelData;
}

void setAtime(uint8_t value)
{
	uint8_t atime[2] = {ATIME, value};
	i2c_write_dt(&dev_i2c,atime,sizeof(atime));
}

void setAGAIN(uint8_t value)
{
	if(value > 10) value = 10;
	uint8_t again[2] = {REG_AS7341_CFG_1, value};
	i2c_write_dt(&dev_i2c,again,sizeof(again));
}

void setAstep(uint16_t value){
	uint8_t step_l[2] = {ASTEP_L, 0};
	uint8_t step_h[2] = {ASTEP_H, 0};
	uint8_t ret;
	step_l[1] = value & 0x00ff;
	step_h[1] = value >> 8 ;
	
	ret = i2c_write_dt(&dev_i2c,step_l,sizeof(step_l));
	if(ret != 0){
		printk("Failed to write/read I2C device address %x at Reg. %x \r\n", dev_i2c.addr,step_l[1]);
	}
	i2c_write_dt(&dev_i2c,step_h,sizeof(step_h));
}
/*
void enableSpectralInterrupt(bool on){
	uint8_t ret;
	uint8_t enspec[2] = {REG_AS7341_INTENAB, 0};
	ret = i2c_write_read_dt(&dev_i2c,&enspec[0],1,&enspec[1],1);
	if(ret != 0){
		printk("Failed to write/read I2C device address %x at Reg. %x \r\n", dev_i2c.addr,enspec[0]);
	}
	if(on == true){
    	enspec[1] = enspec[1] | (1<<3);
	
	i2c_write_dt(&dev_i2c,enspec,sizeof(enspec));
  	}
	else{
		enspec[1] = enspec[1] & (~(1<<3));
		i2c_write_dt(&dev_i2c,enspec,sizeof(enspec));
	}
  
}
*/
static void bt_ready(int err)
{
	char addr_s[BT_ADDR_LE_STR_LEN];
	bt_addr_le_t addr = {0};
	size_t count = 1;

	if (err) {
		printk("Bluetooth init failed (err %d)\n", err);
		return;
	}

	printk("Bluetooth initialized\n");

	/* Start advertising */
	err = bt_le_adv_start(BT_LE_ADV_NCONN_IDENTITY, ad, ARRAY_SIZE(ad),
			      sd, ARRAY_SIZE(sd));
	if (err) {
		printk("Advertising failed to start (err %d)\n", err);
		return;
	}


	/* For connectable advertising you would use
	 * bt_le_oob_get_local().  For non-connectable non-identity
	 * advertising an non-resolvable private address is used;
	 * there is no API to retrieve that.
	 */

	bt_id_get(&addr, &count);
	bt_addr_le_to_str(&addr, addr_s, sizeof(addr_s));

	printk("Beacon started, advertising as %s\n", addr_s);
}

int main(void)
{
	int err;
	int AF;
	int PPIX;
	int i=0;
	int CH0, CH1, CH2, CH3, CH4, CH5;

	//Retrive API-specvific device structure and ensure ready to use
	//Need to run through and esnure all functions required are present
	//static const struct i2c_dt_spec dev_i2c = I2C_DT_SPEC_GET(I2C0_NODE);
	if (!device_is_ready(dev_i2c.bus)) {
		printk("I2C bus %s is not ready!\n\r",dev_i2c.bus->name);
		return -1;
	}
	enableAS7341(true);
	setAGAIN(10); // sets gain to 512x
	setAstep(84);
	setAtime(84);
	startMeasure();
	for(i=0; i<10; i++){
		
		//read data from f4 (ch3)
		//AF = getChannelData(AS7341_CH3_DATA_L, AS7341_CH3_DATA_H);
		//printf("Value from f4: %d\n", AF);
		
		CH0 = getChannelData(AS7341_CH0_DATA_L, AS7341_CH0_DATA_H);
		CH1 = getChannelData(AS7341_CH1_DATA_L, AS7341_CH1_DATA_H);
		PPIX = getChannelData(AS7341_CH2_DATA_L, AS7341_CH2_DATA_H);
		AF = getChannelData(AS7341_CH3_DATA_L, AS7341_CH3_DATA_H);
		CH4 = getChannelData(AS7341_CH4_DATA_L, AS7341_CH4_DATA_H);
		CH5 = getChannelData(AS7341_CH5_DATA_L, AS7341_CH5_DATA_H);
		printk("Channel values: %d, %d, %d, %d, %d, %d\n", CH0, CH1, AF, PPIX, CH4, CH5);
		//printk("F4 Reading: %d\nF7 Reading: %d\n", AF, PPIX);
		/*startMeasure(2);
		CH0 = getChannelData(AS7341_CH0_DATA_L, AS7341_CH0_DATA_H);
		CH1 = getChannelData(AS7341_CH1_DATA_L, AS7341_CH1_DATA_H);
		CH2 = getChannelData(AS7341_CH2_DATA_L, AS7341_CH2_DATA_H);
		CH3 = getChannelData(AS7341_CH3_DATA_L, AS7341_CH3_DATA_H);
		CH4 = getChannelData(AS7341_CH4_DATA_L, AS7341_CH4_DATA_H);
		CH5 = getChannelData(AS7341_CH5_DATA_L, AS7341_CH5_DATA_H);
		printk("Channel values from 2: %d, %d, %d, %d, %d, %d\n", CH0, CH1, CH2, CH3, CH4, CH5);
		*/
		//startMeasure(2);
		//read data from f7 (ch2)
		//PPIX = getChannelData(AS7341_CH2_DATA_L, AS7341_CH2_DATA_H);
		//printf("Value from f7: %d\n", PPIX);
	}
	
	printk("Starting Beacon\n");

	/* Initialize the Bluetooth Subsystem */
	err = bt_enable(bt_ready);
	if (err) {
		printk("Bluetooth init failed (err %d)\n", err);
	}
	return 0;
}