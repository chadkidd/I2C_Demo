/*
 * Title: smbus.c
 *
 * Description: Functions to access the i2c smbus.
 *              These functions are declared in i2c-dev.h
 *              but these are local implementations.
 *
 * Author: Chad Kidd
 *
 * Copyright (C) Advanced Input Systems 2018.
 */

#include <sys/ioctl.h>
#include <linux/i2c-dev.h>
#include <linux/i2c.h>
#include <stdint.h>

#include "smbus.h"

/************************************************************
 *  Name: i2c_smbus_access
 *
 *  Description: i2c smbus wrapper - uses ioctl to access i2c bus
 *
 ************************************************************/
int32_t i2c_smbus_access(int32_t file, int8_t read_write, uint8_t command,
                                     int32_t size, union i2c_smbus_data *data)
{
	struct i2c_smbus_ioctl_data args;

	args.read_write = read_write;
	args.command = command;
	args.size = size;
	args.data = data;
	return ioctl(file,I2C_SMBUS,&args);
}

/************************************************************
 *  Name: i2c_smbus_write_byte_data
 *
 *  Description: i2c smbus - write a single byte of data
 *
 ************************************************************/
int32_t i2c_smbus_write_byte_data(int32_t file, uint8_t command, uint8_t value)
{
	union i2c_smbus_data data;
	data.byte = value;
	return i2c_smbus_access(file,I2C_SMBUS_WRITE,command,
							I2C_SMBUS_BYTE_DATA, &data);
}


/************************************************************
 *  Name: i2c_smbus_read_byte_data
 *
 *  Description: i2c smbus - read a single byte of data
 *
 ************************************************************/
int32_t i2c_smbus_read_byte_data(int file, uint8_t command)
{
	union i2c_smbus_data data;
	if (i2c_smbus_access(file,I2C_SMBUS_READ,command,
	                     I2C_SMBUS_BYTE_DATA,&data))
		return -1;
	else
		return 0x0FF & data.byte;
}

/************************************************************
 *  Name: i2c_smbus_read_block_data
 *
 *  Description: i2c smbus - read a block of data, len specified
 *
 ************************************************************/
int32_t i2c_smbus_read_block_data(int file, uint8_t command,
                                      uint8_t length, uint8_t *values)
{
	union i2c_smbus_data data;
	int i;

	if (length > 32)
		length = 32;
	data.block[0] = length;
	if (i2c_smbus_access(file,I2C_SMBUS_READ,command,
	                     length == 32 ? I2C_SMBUS_I2C_BLOCK_BROKEN :
	                      I2C_SMBUS_I2C_BLOCK_DATA,&data))
		return -1;
	else {
		for (i = 1; i <= data.block[0]; i++)
			values[i-1] = data.block[i];
		return data.block[0];
	}
}

