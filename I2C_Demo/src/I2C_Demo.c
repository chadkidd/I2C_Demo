/*
 * Title: I2C_Demo.c
 *
 * Description: Demonstration to communicate with FXOS8700, uses smbus I2C
 *              direct functions rather than the FXOS8700 driver.
 *
 * Author: Chad Kidd
 *
 */

#include <errno.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <unistd.h>
#include <linux/i2c-dev.h>
#include <linux/i2c.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <math.h>

#include "smbus.h"
#include "FXOS8700CQ.h"

#define IMX_GPIO_NR(port, index)   ((((port)-1)*32)+((index)&31))
#define PI					  3.14159

int32_t i2c_file_desc;
//static int32_t remove_accel_driver(void);
static int32_t gpio_accel_reset();

int main(void) {

	int8_t filename[40];
	uint8_t AccelMagData[12];

	//if (remove_accel_driver() < 0) {
	//	exit(1);
	//}
	//usleep(1000);		// ~1ms delay

	if (gpio_accel_reset() < 0) {
		exit(1);
	}
	usleep(1000);		// ~1ms delay

	sprintf(filename,"/dev/i2c-0");
	if ((i2c_file_desc = open(filename, O_RDWR)) < 0) {
		printf("Failed to open the bus.");
		/* ERROR HANDLING; you can check errno to see what went wrong */
		exit(1);
	}

	if (ioctl(i2c_file_desc,I2C_SLAVE, FXOS8700CQ_I2C_ADDRESS) < 0) {
		printf("Failed to acquire bus access and/or talk to slave.\n");
		/* ERROR HANDLING; you can check errno to see what went wrong */
		exit(1);
	}

	int success = i2c_smbus_write_byte_data(i2c_file_desc, CTRL_REG2, 0x40);		// Reset all registers to POR values
	usleep(1000);		// ~1ms delay

	int32_t who_am_i_byte = i2c_smbus_read_byte_data(i2c_file_desc, WHO_AM_I_REG);

	success = i2c_smbus_write_byte_data(i2c_file_desc, CTRL_REG1, 0x00);		// Standby mode
	success = i2c_smbus_write_byte_data(i2c_file_desc, F_SETUP_REG, 0x40);	//FIFO continuous mode, no fifo watermark
	success = i2c_smbus_write_byte_data(i2c_file_desc, M_CTRL_REG1, 0x1F);	// Hybrid mode (accelerometer + magnetometer), max OSR
	success = i2c_smbus_write_byte_data(i2c_file_desc, M_CTRL_REG2, 0x00);	// hyb_autoinc_mode=0
	success = i2c_smbus_write_byte_data(i2c_file_desc, XYZ_DATA_CFG_REG, 0x01);		// 4g mode g=8192 counts / g after 2 bit left shift
	success = i2c_smbus_write_byte_data(i2c_file_desc, CTRL_REG2, 0x02);     // High Resolution mode
	success = i2c_smbus_write_byte_data(i2c_file_desc, CTRL_REG1, 0x0D);		// ODR = 200Hz, Reduced noise, Active mode

	uint32_t i = 0;
	int16_t Xout_Accel_14_bit, Yout_Accel_14_bit, Zout_Accel_14_bit;
	float Xout_g, Yout_g, Zout_g;
	uint16_t Xout_Mag_16_bit, Yout_Mag_16_bit, Zout_Mag_16_bit;
	float Xout_uT, Yout_uT, Zout_uT;
	float Heading;
	int32_t sensor_fifo_count;

	while(i < 10000)
	{

		//wait for data to be on the FIFO
		do
		{
			sensor_fifo_count = i2c_smbus_read_byte_data(i2c_file_desc, STATUS_REG);
		}while((sensor_fifo_count & 0x3F) == 0);

		// Read accelerometer registers 0x01-0x06
		success = i2c_smbus_read_block_data(i2c_file_desc, OUT_X_MSB_REG, 6, AccelMagData);

		Xout_Accel_14_bit = ((int16_t) (AccelMagData[0]<<8 | AccelMagData[1])) >> 2;		// Compute 14-bit X-axis acceleration output value
		Yout_Accel_14_bit = ((int16_t) (AccelMagData[2]<<8 | AccelMagData[3])) >> 2;		// Compute 14-bit Y-axis acceleration output value
		Zout_Accel_14_bit = ((int16_t) (AccelMagData[4]<<8 | AccelMagData[5])) >> 2;		// Compute 14-bit Z-axis acceleration output value

		Xout_g = ((float) Xout_Accel_14_bit) / SENSITIVITY_4G;		// Compute X-axis output value in g's
		Yout_g = ((float) Yout_Accel_14_bit) / SENSITIVITY_4G;		// Compute Y-axis output value in g's
		Zout_g = ((float) Zout_Accel_14_bit) / SENSITIVITY_4G;		// Compute Z-axis output value in g's

		//read Magnetometer registers - 0x33 - 0x38
		success = i2c_smbus_read_block_data(i2c_file_desc, MOUT_X_MSB_REG, 6, AccelMagData);
		Xout_Mag_16_bit = (short) (AccelMagData[0]<<8 | AccelMagData[1]);		// Compute 16-bit X-axis magnetic output value
		Yout_Mag_16_bit = (short) (AccelMagData[2]<<8 | AccelMagData[3]);		// Compute 16-bit Y-axis magnetic output value
		Zout_Mag_16_bit = (short) (AccelMagData[4]<<8 | AccelMagData[5]);		// Compute 16-bit Z-axis magnetic output value
		Xout_uT = (float) (Xout_Mag_16_bit) / SENSITIVITY_MAG;		// Compute X-axis output magnetic value in uT
		Yout_uT = (float) (Yout_Mag_16_bit) / SENSITIVITY_MAG;		// Compute Y-axis output magnetic value in uT
		Zout_uT = (float) (Zout_Mag_16_bit) / SENSITIVITY_MAG;		// Compute Z-axis output magnetic value in uT

		Heading = atan2(Yout_uT, Xout_uT) * 180 / PI;		// Compute Yaw angle

		if((i & 0x4F) == 0x4F){	//just display every Nth read
			//printf("index, fifo, accel data - %d, %d, %.3f, %.3f, %.3f \n", i, (sensor_fifo_count & 0x3F),
			//		Xout_g, Yout_g, Zout_g);
			printf("index, mag data - %d, %.3f, %.3f, %.3f, %.3f \n", i,
					Xout_uT, Yout_uT, Zout_uT, Heading);
		}

		usleep(4000);		// ~3ms delay
		i++;

	}

	close(i2c_file_desc);

}

/************************************************************
 * Name: remove_accel_driver
 *
 * Description:  experimental, uses modprobe to remove
 * 				 fxos8700 driver
 ************************************************************/
/*
static int32_t remove_accel_driver(void)
{
	int8_t buf[40];
	FILE *fp;
	char *s = malloc(sizeof(int8_t) * 200);
	int stat = 0;

	memset(buf, 0, sizeof(buf));
	sprintf(buf, "modprobe -r fxos8700");
	fp = popen(buf, "r");
	if(fp == NULL)
	{
		perror("Failed to remove accel/mag driver");
		free(s);
		return -1;

	}

	//wait for a response from the modprobe command
	//for(uint8_t i = 0; i < 4; i++)
	//{
	//	usleep(1000);
	//	fgets(s, sizeof(int8_t)*200, fp);
	//	printf("%s", s);
	//	if(strstr(s, "Success") != 0)
	//		break;
	//}

	stat = pclose(fp);
	free(s);
	return WEXITSTATUS(stat);
}
*/
/************************************************************
 * Name: gpio_accel_reset
 *
 * Description:  exports GPIO pin for accel/mag reset
 * 		   		 Toggles the reset high/low
 * 		   		 Lastly, frees the GPIO pin
 ************************************************************/
static int32_t gpio_accel_reset(void)
{
	int32_t gpio_file_desc;
	int8_t buf[40];
	int32_t write_len;
	int32_t gpio = IMX_GPIO_NR(1, 25);

	//Reserve (export the GPIO pin)
	memset(buf, 0, sizeof(buf));
	if ((gpio_file_desc = open("/sys/class/gpio/export", O_WRONLY)) < 0) {
		printf("Failed to open file to export the GPIO pin");
		return -1;
	}
	sprintf(buf, "%d", gpio);
	if (write(gpio_file_desc, buf, strlen(buf)) < 0)  {
		printf("Failed to write the export GPIO command");
		return -1;
	}
	close (gpio_file_desc);

	//Set the GPIO direction
	memset(buf, 0, sizeof(buf));
	sprintf(buf, "/sys/class/gpio/gpio%d/direction", gpio);
	if ((gpio_file_desc = open(buf, O_WRONLY)) < 0) {
		printf("Failed to open file to set the GPIO pin direction");
		return -1;
	}
	if (write(gpio_file_desc, "out", 3) < 0)  {
		printf("Failed to write the GPIO pin direction");
		return -1;
	}
	close (gpio_file_desc);

	//Set the GPIO value High
	memset(buf, 0, sizeof(buf));
	sprintf(buf, "/sys/class/gpio/gpio%d/value", gpio);
	if ((gpio_file_desc = open(buf, O_WRONLY)) < 0) {
		printf("Failed to open file to set GPIO value");
		return -1;
	}
	if (write(gpio_file_desc, "1", 1) < 0)  {
		printf("Failed to write the GPIO value");
		return -1;
	}
	close (gpio_file_desc);

	usleep(200);		// ~200uS delay
	//Set the GPIO direction
	memset(buf, 0, sizeof(buf));
	sprintf(buf, "/sys/class/gpio/gpio%d/direction", gpio);
	if ((gpio_file_desc = open(buf, O_WRONLY)) < 0) {
		printf("Failed to open file to set the GPIO pin direction");
		return -1;
	}
	if (write(gpio_file_desc, "out", 3) < 0)  {
		printf("Failed to write the GPIO pin direction");
		return -1;
	}
	close (gpio_file_desc);

	//write the GPIO value Low
	memset(buf, 0, sizeof(buf));
	sprintf(buf, "/sys/class/gpio/gpio%d/value", gpio);
	if ((gpio_file_desc = open(buf, O_WRONLY)) < 0) {
		printf("Failed to open file to set GPIO value");
		return -1;
	}
	if (write(gpio_file_desc, "0", 0) < 0)  {
		printf("Failed to write the GPIO value");
		return -1;
	}
	close (gpio_file_desc);

	//free (unexport) the GPIO pin)
	memset(buf, 0, sizeof(buf));
	if ((gpio_file_desc = open("/sys/class/gpio/unexport", O_WRONLY)) < 0) {
		printf("Failed to open file to free the GPIO pin");
		return -1;
	}
	sprintf(buf, "%d", gpio);
	if (write(gpio_file_desc, buf, strlen(buf)) < 0)  {
		printf("Failed to write the free GPIO command");
		return -1;
	}
	close (gpio_file_desc);

	return 0;
}



