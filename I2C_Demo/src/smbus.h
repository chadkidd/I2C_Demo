/*
 * Title: smbus.h
 *
 * Description: Headers for Functions to access the i2c smbus.
 *              These functions are declared in i2c-dev.h
 *              but here are local implementations.
 *
 * Author: Chad Kidd
 *
 * Copyright (C) Advanced Input Systems 2018.
 */

#ifndef SMBUS_H_
#define SMBUS_H_

#include <stdint.h>

int32_t i2c_smbus_access(int32_t file, int8_t read_write, uint8_t command,
                                     int32_t size, union i2c_smbus_data *data);
int32_t i2c_smbus_write_byte_data(int32_t file, uint8_t command, uint8_t value);
int32_t i2c_smbus_read_byte_data(int file, uint8_t command);
int32_t i2c_smbus_read_block_data(int file, uint8_t command,
                                      uint8_t length, uint8_t *values);

#endif /* SMBUS_H_ */
