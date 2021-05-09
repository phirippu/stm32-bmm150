/**\
 * Copyright (c) 2020 Bosch Sensortec GmbH. All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 **/

/******************************************************************************/
#include <stdio.h>
#include "bmm150_common.h"
#include "bmm150.h"
#include "main.h"
#include "i2c.h"
#include "tim.h"

/******************************************************************************/
/*!                Static variable definition                                 */

/*! Variable that holds the I2C device address or SPI chip selection */
static uint8_t dev_addr;

/******************************************************************************/
/*!                User interface functions                                   */

/*!
 * @brief Function for initialization of I2C bus.
 */
int8_t bmm150_user_i2c_init(void) {
    /* Implement I2C bus initialization according to the target machine. */

    if (HAL_I2C_IsDeviceReady(&hi2c1, BMM150_I2C_ADDRESS_CSB_HIGH_SDO_HIGH << 1, 2, 2000) == HAL_OK) {
        printf("BMM150 is ready\n");
        return 0;
    }
    return 1;
}

/*!
 * @brief Function for initialization of SPI bus.
 */
int8_t bmm150_user_spi_init(void) {

    /* Implement SPI bus initialization according to the target machine. */
    return 0;
}

/*!
 * @brief This function provides the delay for required time (Microseconds) as per the input provided in some of the
 * APIs.
 */
void bmm150_user_delay_us(uint32_t period_us, void *intf_ptr) {
    __HAL_TIM_SET_COUNTER(&htim10, 0);  // set the counter value a 0
    while (__HAL_TIM_GET_COUNTER(&htim10) < period_us);  // wait for the counter to reach the us input in the parameter
    /* Wait for a period amount of microseconds. */
}

/*!
 * @brief This function is for writing the sensor's registers through I2C bus.
 */
int8_t bmm150_user_i2c_reg_write(uint8_t reg_addr, const uint8_t *reg_data, uint32_t length, void *intf_ptr) {

    /* Write to registers using I2C. Return 0 for a successful execution. */
    if (HAL_I2C_Mem_Write(&hi2c1, BMM150_I2C_ADDRESS_CSB_HIGH_SDO_HIGH << 1, reg_addr, 1, reg_data, length, 1000) == HAL_OK) {
        return BMM150_OK;
    };
    return 1;
}

/*!
 * @brief This function is for reading the sensor's registers through I2C bus.
 */
int8_t bmm150_user_i2c_reg_read(uint8_t reg_addr, uint8_t *reg_data, uint32_t length, void *intf_ptr) {

    /* Read from registers using I2C. Return 0 for a successful execution. */
    if (HAL_I2C_Mem_Read(&hi2c1, BMM150_I2C_ADDRESS_CSB_HIGH_SDO_HIGH << 1, reg_addr, 1, reg_data, length, 1000) == HAL_OK) {
        return BMM150_OK;
    };
    return 1;
}

/*!
 * @brief This function is for writing the sensor's registers through SPI bus.
 */
int8_t bmm150_user_spi_reg_write(uint8_t reg_addr, const uint8_t *reg_data, uint32_t length, void *intf_ptr) {

    /* Write to registers using SPI. Return 0 for a successful execution. */
    return 0;
}

/*!
 * @brief This function is for reading the sensor's registers through SPI bus.
 */
int8_t bmm150_user_spi_reg_read(uint8_t reg_addr, uint8_t *reg_data, uint32_t length, void *intf_ptr) {

    /* Read from registers using SPI. Return 0 for a successful execution. */
    return 0;
}

/*!
 *  @brief This function is to select the interface between SPI and I2C.
 */
int8_t bmm150_interface_selection(struct bmm150_dev *dev) {
    int8_t rslt = BMM150_OK;

    if (dev != NULL) {
        /* Select the interface for execution
         * For I2C : BMM150_I2C_INTF
         * For SPI : BMM150_SPI_INTF
         */
        dev->intf = BMM150_I2C_INTF;

        /* Bus configuration : I2C */
        if (dev->intf == BMM150_I2C_INTF) {
            printf("I2C Interface \n");

            /* To initialize the user I2C function */
            bmm150_user_i2c_init();

            dev_addr = BMM150_I2C_ADDRESS_CSB_HIGH_SDO_HIGH;
            dev->read = bmm150_user_i2c_reg_read;
            dev->write = bmm150_user_i2c_reg_write;
        }
            /* Bus configuration : SPI */
        else if (dev->intf == BMM150_SPI_INTF) {
            printf("SPI Interface \n");

            /* To initialize the user SPI function */
            bmm150_user_spi_init();

            dev_addr = 0;
            dev->read = bmm150_user_spi_reg_read;
            dev->write = bmm150_user_spi_reg_write;
        }

        /* Assign device address to interface pointer */
        dev->intf_ptr = &dev_addr;

        /* Configure delay in microseconds */
        dev->delay_us = bmm150_user_delay_us;
    } else {
        rslt = BMM150_E_NULL_PTR;
    }

    return rslt;
}

/*!
 * @brief This internal API prints the execution status
 */
void bmm150_error_codes_print_result(const char api_name[], int8_t rslt) {
    if (rslt != BMM150_OK) {
        printf("%s\t", api_name);

        switch (rslt) {
            case BMM150_E_NULL_PTR:
                printf("Error [%d] : Null pointer error.", rslt);
                printf(
                        "It occurs when the user tries to assign value (not address) to a pointer, which has been initialized to NULL.\r\n");
                break;

            case BMM150_E_COM_FAIL:
                printf("Error [%d] : Communication failure error.", rslt);
                printf(
                        "It occurs due to read/write operation failure and also due to power failure during communication\r\n");
                break;

            case BMM150_E_DEV_NOT_FOUND:
                printf("Error [%d] : Device not found error. It occurs when the device chip id is incorrectly read\r\n",
                       rslt);
                break;

            case BMM150_E_INVALID_CONFIG:
                printf("Error [%d] : Invalid sensor configuration.", rslt);
                printf(" It occurs when there is a mismatch in the requested feature with the available one\r\n");
                break;

            default:
                printf("Error [%d] : Unknown error code\r\n", rslt);
                break;
        }
    }
}
