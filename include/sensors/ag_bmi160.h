/**-------------------------------------------------------------------------
@file	ag_bmi160.h

@brief	implementation of BOSCH BMI160 sensor
			Accel, Gyro

@author	Hoang Nguyen Hoan
@date	Nov. 18, 2017

@license

MIT License

Copyright (c) 2017 I-SYST inc. All rights reserved.

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.

----------------------------------------------------------------------------*/

#ifndef __AG_BMI160_H__
#define __AG_BMI160_H__

#include <stdint.h>

#include "coredev/iopincfg.h"
#include "sensors/accel_sensor.h"
#include "sensors/gyro_sensor.h"
#include "sensors/mag_bmm150.h"

#define BMI160_I2C_7BITS_DEVADDR							0x68
#define BMI160_I2C_7BITS_DEVADDR_ALT						0x69	// When SDO pull to VDDIO

#define BMI160_CHIP_ID_REG          0x0

#define BMI160_CHIP_ID                                      0xD1

#define BMI160_ERR_REG              0x2
#define BMI160_ERR_REG_FATAL_ERR                            (1<<0)
#define BMI160_ERR_REG_ERR_CODE_MASK                        (0xF<<1)
#define BMI160_ERR_REG_I2C_FAIL_ERR                         (1<<5)
#define BMI160_ERR_REG_DROP_CMD_ERR                         (1<<6)
#define BMI160_ERR_REG_MAG_DRDDY_ERR                        (1<<7)

#define BMI160_PMU_STATUS           0x3
#define BMI160_PMU_STATUS_MAG_PMU_STATUS_MASK               (3<<0)
#define BMI160_PMU_STATUS_MAG_PMU_STATUS_SUSPEND			(0<<0)
#define BMI160_PMU_STATUS_MAG_PMU_STATUS_NORMAL				(1<<0)
#define BMI160_PMU_STATUS_MAG_PMU_STATUS_LOWPOWER			(2<<0)

#define BMI160_PMU_STATUS_GYR_PMU_STATUS_MASK               (3<<2)
#define BMI160_PMU_STATUS_GYR_PMU_STATUS_SUSPEND			(0<<2)
#define BMI160_PMU_STATUS_GYR_PMU_STATUS_NORMAL				(1<<2)
#define BMI160_PMU_STATUS_GYR_PMU_STATUS_FASTSTARTUP		(3<<2)

#define BMI160_PMU_STATUS_ACC_PMU_STATUS_MASK               (3<<4)
#define BMI160_PMU_STATUS_ACC_PMU_STATUS_SUSPEND			(0<<4)
#define BMI160_PMU_STATUS_ACC_PMU_STATUS_NORMAL				(1<<4)
#define BMI160_PMU_STATUS_ACC_PMU_STATUS_LOWPOWER			(2<<4)


#define BMI160_DATA_MAG_X_LSB		0x4
#define BMI160_DATA_MAG_X_MSB		0x5
#define BMI160_DATA_MAG_Y_LSB		0x6
#define BMI160_DATA_MAG_Y_MSB		0x7
#define BMI160_DATA_MAG_Z_LSB		0x8
#define BMI160_DATA_MAG_Z_MSB		0x9
#define BMI160_DATA_RHALL_LSB		0xA
#define BMI160_DATA_RHALL_MSB		0xB
#define BMI160_DATA_GYRO_X_LSB		0xC
#define BMI160_DATA_GYRO_X_MSB		0xD
#define BMI160_DATA_GYRO_Y_LSB		0xE
#define BMI160_DATA_GYRO_Y_MSB		0xF
#define BMI160_DATA_GYRO_Z_LSB		0x10
#define BMI160_DATA_GYRO_Z_MSB		0x11
#define BMI160_DATA_ACC_X_LSB		0x12
#define BMI160_DATA_ACC_X_MSB		0x13
#define BMI160_DATA_ACC_Y_LSB		0x14
#define BMI160_DATA_ACC_Y_MSB		0x15
#define BMI160_DATA_ACC_Z_LSB		0x16
#define BMI160_DATA_ACC_Z_MSB		0x17
#define BMI160_SENSORTIME_0         0x18
#define BMI160_SENSORTIME_1         0x19
#define BMI160_SENSORTIME_2         0x1A

#define BMI160_STATUS               0x1B
#define BMI160_STATUS_GYR_SELF_TEST_OK                      (1<<1)
#define BMI160_STATUS_MAG_MAN_OP                            (1<<2)
#define BMI160_STATUS_FOC_RDY                               (1<<3)
#define BMI160_STATUS_NVM_RDY                               (1<<4)
#define BMI160_STATUS_DRDY_MAG                              (1<<5)
#define BMI160_STATUS_DRDY_GYR                              (1<<6)
#define BMI160_STATUS_DRDY_ACC                              (1<<7)

#define BMI160_INT_STATUS_0             0x1C
#define BMI160_INT_STATUS_0_STEP_INT                            (1<<0)
#define BMI160_INT_STATUS_0_SIGMOT_INT                          (1<<1)
#define BMI160_INT_STATUS_0_ANYM_INT                            (1<<2)
#define BMI160_INT_STATUS_0_PMU_TRIGGER_INT                     (1<<3)
#define BMI160_INT_STATUS_0_D_TAP_INT                           (1<<4)
#define BMI160_INT_STATUS_0_S_TAP_INT                           (1<<5)
#define BMI160_INT_STATUS_0_ORIENT_INT                          (1<<6)
#define BMI160_INT_STATUS_0_FLAT_INT                            (1<<7)

#define BMI160_INT_STATUS_1             0x1D
#define BMI160_INT_STATUS_1_HIGHG_INT                           (1<<2)
#define BMI160_INT_STATUS_1_LOWG_INT                            (1<<3)
#define BMI160_INT_STATUS_1_DRDY_INT                            (1<<4)
#define BMI160_INT_STATUS_1_FFULL_INT                           (1<<5)
#define BMI160_INT_STATUS_1_FWM_INT                             (1<<6)
#define BMI160_INT_STATUS_1_NOMO_INT                            (1<<7)

#define BMI160_INT_STATUS_2             0x1E
#define BMI160_INT_STATUS_2_ANYM_FIRST_X                        (1<<0)
#define BMI160_INT_STATUS_2_ANYM_FIRST_Y                        (1<<1)
#define BMI160_INT_STATUS_2_ANYM_FIRST_Z                        (1<<2)
#define BMI160_INT_STATUS_2_ANYM_SIGN                           (1<<3)
#define BMI160_INT_STATUS_2_TAP_FIRST_X                         (1<<4)
#define BMI160_INT_STATUS_2_TAP_FIRST_Y                         (1<<5)
#define BMI160_INT_STATUS_2_TAP_FIRST_Z                         (1<<6)
#define BMI160_INT_STATUS_2_TAP_SIGN                            (1<<7)

#define BMI160_INT_STATUS_3             0x1F
#define BMI160_INT_STATUS_3_HIGH_FIRST_X                        (1<<0)
#define BMI160_INT_STATUS_3_HIGH_FIRST_Y                        (1<<1)
#define BMI160_INT_STATUS_3_HIHI_FIRST_Z                        (1<<2)
#define BMI160_INT_STATUS_3_HIGH_SIGN                           (1<<3)
#define BMI160_INT_STATUS_3_ORIENT_1_0_MASK                     (3<<4)
#define BMI160_INT_STATUS_3_ORIENT_2                            (1<<6)
#define BMI160_INT_STATUS_3_FLAT                                (1<<7)

#define BMI160_TEMPERATURE_0        0x20

#define BMI160_TEMPERATURE_1        0x21

#define BMI160_FIFO_LENGTH_0        0x22
#define BMI160_FIFO_LENGTH_0_FIFO_BYTE_COUNTER_7_0_MASK     (0xFF)

#define BMI160_FIFO_LENGTH_1        0x23
#define BMI160_FIFO_LENGTH_1_FIFO_BYTE_COUNTER_10_8_MASK    (7)

#define BMI160_FIFO_DATA            0x24

#define BMI160_ACC_CONF             0x40
#define BMI160_ACC_CONF_ACC_ODR_MASK                        (0xF)
#define BMI160_ACC_CONF_ACC_BWP_MASK                        (0x7<<4)
#define BMI160_ACC_CONF_ACC_BWP_OSR4						(0<<4)
#define BMI160_ACC_CONF_ACC_BWP_OSR2						(1<<4)
#define BMI160_ACC_CONF_ACC_BWP_NORMAL_AVG4					(2<<4)
#define BMI160_ACC_CONF_ACC_BWP_AVG8						(3<<4)
#define BMI160_ACC_CONF_ACC_BWP_AVG16						(4<<4)
#define BMI160_ACC_CONF_ACC_BWP_AVG32						(5<<4)
#define BMI160_ACC_CONF_ACC_BWP_AVG64						(6<<4)
#define BMI160_ACC_CONF_ACC_BWP_AVG128						(7<<4)
#define BMI160_ACC_CONF_ACC_US                              (1<<7)

#define BMI160_ACC_RANGE            0x41
#define BMI160_ACC_RANGE_ACC_RANGE_MASK                     (0xF)
#define BMI160_ACC_RANGE_ACC_RANGE_2G						(3)
#define BMI160_ACC_RANGE_ACC_RANGE_4G						(5)
#define BMI160_ACC_RANGE_ACC_RANGE_8G						(8)
#define BMI160_ACC_RANGE_ACC_RANGE_16G						(0xC)

#define BMI160_GYR_CONF             0x42
#define BMI160_GYR_CONF_GYR_ODR_MASK                        (0xF<<0)
#define BMI160_GYR_CONF_GYR_BWP_MASK                        (3<<4)
#define BMI160_GYR_CONF_GYR_BWP_OSR4						(0<<4)
#define BMI160_GYR_CONF_GYR_BWP_OSR2						(1<<4)
#define BMI160_GYR_CONF_GYR_BWP_NORMAL						(2<<4)

#define BMI160_GYR_RANGE            0x43
#define BMI160_GYR_RANGE_GYR_RANGE_MASK                     (7<<0)
#define BMI160_GYR_RANGE_GYR_RANGE_2000						(0<<0)
#define BMI160_GYR_RANGE_GYR_RANGE_1000						(1<<0)
#define BMI160_GYR_RANGE_GYR_RANGE_500						(2<<0)
#define BMI160_GYR_RANGE_GYR_RANGE_250						(3<<0)
#define BMI160_GYR_RANGE_GYR_RANGE_125						(4<<0)

#define BMI160_MAG_CONF             0x44
#define BMI160_MAG_CONF_MAG_ODR_MASK                        (0xF<<0)

#define BMI160_FIFO_DOWNS           0x45
#define BMI160_FIFO_DOWNS_GYR_FIFO_DOWNS_MASK               (7<<0)
#define BMI160_FIFO_DOWNS_GYR_FIFO_DOWNS_BITPOS				0
#define BMI160_FIFO_DOWNS_GYR_FIFO_FILT_DATA                (1<<3)
#define BMI160_FIFO_DOWNS_ACC_FIFO_DOWNS_MASK               (7<<4)
#define BMI160_FIFO_DOWNS_ACC_FIFO_DOWNS_BITPOS             4
#define BMI160_FIFO_DOWNS_ACC_FIFO_FILT_DATA                (1<<7)

#define BMI160_FIFO_CONFIG_0        0x46
#define BMI160_FIFO_CONFIG_0_FIFO_WATER_MARK_MASK           (0xFF)

#define BMI160_FIFO_CONFIG_1        0x47
#define BMI160_FIFO_CONFIG_1_FIFO_TIME_EN                   (1<<1)
#define BMI160_FIFO_CONFIG_1_FIFO_TAG_INT2_EN               (1<<2)
#define BMI160_FIFO_CONFIG_1_FIFO_TAG_INT1_EN               (1<<3)
#define BMI160_FIFO_CONFIG_1_FIFO_HEADER_EN                 (1<<4)
#define BMI160_FIFO_CONFIG_1_FIFO_MAG_EN                    (1<<5)
#define BMI160_FIFO_CONFIG_1_FIFO_ACC_EN                    (1<<6)
#define BMI160_FIFO_CONFIG_1_FIFO_GYR_EN                    (1<<7)

#define BMI160_MAG_IF_0             0x4B
#define BMI160_MAG_IF_0_I2C_DEVICE_ADDR_MASK                (0x7<<1)

#define BMI160_MAG_IF_1             0x4C
#define BMI160_MAG_IF_1_MAG_RD_BURST_MASK                   (3<<0)
#define BMI160_MAG_IF_1_MAG_RD_BURST_1						(0<<0)
#define BMI160_MAG_IF_1_MAG_RD_BURST_2						(1<<0)
#define BMI160_MAG_IF_1_MAG_RD_BURST_6						(2<<0)
#define BMI160_MAG_IF_1_MAG_RD_BURST_8						(3<<0)
#define BMI160_MAG_IF_1_MAG_OFFSET_MASK                     (0xF<<2)
#define BMI160_MAG_IF_1_MAG_MANUAL_EN                       (1<<7)

#define BMI160_MAG_IF_2             0x4D
#define BMI160_MAG_IF_2_READ_ADDR_MASK                      (0xFF)

#define BMI160_MAG_IF_3             0x4E
#define BMI160_MAG_IF_3_WRITE_ADDR_MASK                     (0xFF)

#define BMI160_MAG_IF_4             0x4F
#define BMI160_MAG_IF_4_WRITE_DATA_MASK                     (0xFF)

#define BMI160_INT_EN_0             0x50
#define BMI160_INT_EN_0_INT_ANYMO_X_EN                      (1<<0)
#define BMI160_INT_EN_0_INT_ANYMO_Y_EN                      (1<<1)
#define BMI160_INT_EN_0_INT_ANYMO_Z_EN                      (1<<2)
#define BMI160_INT_EN_0_INT_D_TAP_EN                        (1<<4)
#define BMI160_INT_EN_0_INT_S_TAP_EN                        (1<<5)
#define BMI160_INT_EN_0_INT_ORIENT_EN                       (1<<6)
#define BMI160_INT_EN_0_INT_FLAT_EN                         (1<<7)

#define BMI160_INT_EN_1             0x51
#define BMI160_INT_EN_1_INT_HIGHG_X_EN                      (1<<0)
#define BMI160_INT_EN_1_INT_HIGHG_Y_EN                      (1<<1)
#define BMI160_INT_EN_1_INT_HIGHG_Z_EN                      (1<<2)
#define BMI160_INT_EN_1_INT_LOW_EN                          (1<<3)
#define BMI160_INT_EN_1_INT_DRDY_EN                         (1<<4)
#define BMI160_INT_EN_1_INT_FFULL_EN                        (1<<5)
#define BMI160_INT_EN_1_INT_FWM_EN                          (1<<6)

#define BMI160_INT_EN_2             0x52
#define BMI160_INT_EN_2_INT_NOMOX_EN                        (1<<0)
#define BMI160_INT_EN_2_INT_NOMOY_EN                        (1<<1)
#define BMI160_INT_EN_2_INT_NOMOZ_EN                        (1<<2)
#define BMI160_INT_EN_2_INT_STEP_DET_EN                     (1<<3)

#define BMI160_INT_OUT_CTRL         0x53
#define BMI160_INT_OUT_CTRL_INT1_EDGE_CTRL                  (1<<0)
#define BMI160_INT_OUT_CTRL_INT1_LVL                        (1<<1)
#define BMI160_INT_OUT_CTRL_INT1_OD                         (1<<2)
#define BMI160_INT_OUT_CTRL_INT1_OUTPUT_EN                  (1<<3)
#define BMI160_INT_OUT_CTRL_INT2_EDGE_CTRL                  (1<<4)
#define BMI160_INT_OUT_CTRL_INT2_LVL                        (1<<5)
#define BMI160_INT_OUT_CTRL_INT2_OD                         (1<<6)
#define BMI160_INT_OUT_CTRL_INT2_OUTPUT_EN                  (1<<7)

#define BMI160_LATCH                0x54
#define BMI160_LATCH_INT_LATCH_MASK                         (0xF)
#define BMI160_LATCH_INT1_INPUT_EN                          (1<<4)
#define BMI160_LATCH_INT2_INPUT_EN                          (1<<5)

#define BMI160_INT_MAP_0            0x55
#define BMI160_INT_MAP_0_INT1_LOWG_STEP                     (1<<0)
#define BMI160_INT_MAP_0_INT1_HIGHG                         (1<<1)
#define BMI160_INT_MAP_0_INT1_ANYMOTION                     (1<<2)
#define BMI160_INT_MAP_0_INT1_NOMOTION                      (1<<3)
#define BMI160_INT_MAP_0_INT1_D_TAP                         (1<<4)
#define BMI160_INT_MAP_0_INT1_S_TAP                         (1<<5)
#define BMI160_INT_MAP_0_INT1_ORIENT                        (1<<6)
#define BMI160_INT_MAP_0_INT1_FLAT                          (1<<7)

#define BMI160_INT_MAP_1            0x56
#define BMI160_INT_MAP_1_INT2_PMU_TRIG                      (1<<0)
#define BMI160_INT_MAP_1_INT2_FFULL                         (1<<1)
#define BMI160_INT_MAP_1_INT2_FWM                           (1<<2)
#define BMI160_INT_MAP_1_INT2_DRDY                          (1<<3)
#define BMI160_INT_MAP_1_INT1_PMU_TRIG                      (1<<4)
#define BMI160_INT_MAP_1_INT1_FFULL                         (1<<5)
#define BMI160_INT_MAP_1_INT1_FWM                           (1<<6)
#define BMI160_INT_MAP_1_INT1_DRDY                          (1<<7)

#define BMI160_INT_MAP_2            0x57
#define BMI160_INT_MAP_2_INT2_LOWG_STEP                     (1<<0)
#define BMI160_INT_MAP_2_INT2_HIGHG                         (1<<1)
#define BMI160_INT_MAP_2_INT2_ANYMOTION                     (1<<2)
#define BMI160_INT_MAP_2_INT2_NOMOTION                      (1<<3)
#define BMI160_INT_MAP_2_INT2_D_TAP                         (1<<4)
#define BMI160_INT_MAP_2_INT2_S_TAP                         (1<<5)
#define BMI160_INT_MAP_2_INT2_ORIENT                        (1<<6)
#define BMI160_INT_MAP_2_INT2_FLAT                          (1<<7)

#define BMI160_INT_DATA_0           0x58
#define BMI160_INT_DATA_0_INT_TAP_SRC                       (1<<3)
#define BMI160_INT_DATA_0_INT_LOW_HIGH_SRC                  (1<<7)

#define BMI160_INT_DATA_1           0x59
#define BMI160_INT_DATA_1_INT_MOTION_SRC                    (1<<7)

#define BMI160_INT_LOWHIGH_0        0x5A
#define BMI160_INT_LOWHIGH_0_INT_LOW_DUR_MASK               0xFF

#define BMI160_INT_LOWHIGH_1        0x5B
#define BMI160_INT_LOWHIGH_1_INT_LOW_TH_MASK                0xFF

#define BMI160_INT_LOWHIGH_2        0x5C
#define BMI160_INT_LOWHIGH_2_INT_LOW_HY_MASK                (3<<0)
#define BMI160_INT_LOWHIGH_2_INT_LOW_MODE                   (1<<2)
#define BMI160_INT_LOWHIGH_2_INT_HIGH_HY                    (3<<6)

#define BMI160_INT_LOWHIGH_3        0x5D
#define BMI160_INT_LOWHIGH_3_INT_HIGH_DUR_MASK              0xFF

#define BMI160_INT_LOWHIGH_4        0x5E
#define BMI160_INT_LOWHIGH_4_INT_HIGH_TH_MASK               0xFF

#define BMI160_INT_MOTION_0         0x5F
#define BMI160_INT_MOTION_0_INT_ANYM_DUR_MASK               (3<<0)
#define BMI160_INT_MOTION_0_INT_SLO_NOMO_DUR_MASK           (0x3F<<2)

#define BMI160_INT_MOTION_1         0x60
#define BMI160_INT_MOTION_1_INT_ANYMO_TH_MASK               0xFF

#define BMI160_INT_MOTION_2         0x61
#define BMI160_INT_MOTION_2_INT_SLO_NOMO_TH_MASK            0xFF

#define BMI160_INT_MOTION_3         0x62
#define BMI160_INT_MOTION_3_INT_SLO_NOMO_SEL                (1<<0)
#define BMI160_INT_MOTION_3_INT_SIG_MOT_SEL                 (1<<1)
#define BMI160_INT_MOTION_3_INT_SIG_MOT_SKIP_MASK           (3<<2)
#define BMI160_INT_MOTION_3_INT_SIG_MOT_PROOF_MASK          (3<<4)

#define BMI160_INT_TAP_0            0x63
#define BMI160_INT_TAP_0_INT_TAP_DUR_MASK                   (7)
#define BMI160_INT_TAP_0_INT_TAP_SHOCK                      (1<<6)
#define BMI160_INT_TAP_0_INT_TAP_QUIET                      (1<<7)

#define BMI160_INT_TAP_1            0x64
#define BMI160_INT_TAP_1_INT_TAP_TH_MASK                    (0x1F)

#define BMI160_INT_ORIENT_0         0x65
#define BMI160_INT_ORIENT_0_INT_ORIENT_MODE_MASK            (3<<0)
#define BMI160_INT_ORIENT_0_INT_ORIENT_BLOCKING_MASK        (3<<2)
#define BMI160_INT_ORIENT_0_INT_ORIENT_HY_MASK              (0xF<<4)

#define BMI160_INT_ORIENT_1         0x66
#define BMI160_INT_ORIENT_1_INT_ORIENT_THETA_MASK           (0x3F)
#define BMI160_INT_ORIENT_1_INT_ORIENT_UD_EN                (1<<6)
#define BMI160_INT_ORIENT_1_INT_ORIENT_AXES_EX              (1<<7)

#define BMI160_INT_FLAT_0           0x67
#define BMI160_INT_FLAT_0_INT_FLAT_THETA_MASK               (0x3F)

#define BMI160_INT_FLAT_1           0x68
#define BMI160_INT_FLAT_1_INT_FLAT_HY_MASK                  (0xF<<0)
#define BMI160_INT_FLAT_1_INT_FLAT_HOLD_MASK                (3<<4)

#define BMI160_FOC_CONF             0x69
#define BMI160_FOC_CONF_FOC_ACC_Z_MASK                      (3<<0)
#define BMI160_FOC_CONF_FOC_ACC_Y_MASK                      (3<<2)
#define BMI160_FOC_CONF_FOC_ACC_X_MASK                      (3<<4)
#define BMI160_FOC_CONF_FOC_GYR_EN                          (1<<6)

#define BMI160_CONF                 0x6A
#define BMI160_CONF_NVM_PROG_EN                             (1<<1)

#define BMI160_IF_CONF              0x6B
#define BMI160_IF_CONF_SPI_3WIRE                            (1<<0)
#define BMI160_IF_CONF_IF_MODE_MASK                         (3<<4)
#define BMI160_IF_CONF_IF_MODE_AUTO_OFF						(0<<4)
#define BMI160_IF_CONF_IF_MODE_I2C_OIS						(1<<4)
#define BMI160_IF_CONF_IF_MODE_AUTO_MAG						(2<<4)

#define BMI160_PMU_TRIGGER          0x6C
#define BMI160_PMU_TRIGGER_GYR_SLEEP_TRIGGER_MASK           (7<<0)
#define BMI160_PMU_TRIGGER_GYR_WAKEUP_TRIGGER_MASK          (3<<3)
#define BMI160_PMU_TRIGGER_GYR_SLEEP_STATE                  (1<<5)
#define BMI160_PMU_TRIGGER_WAKEUP_INT                       (1<<6)

#define BMI160_SELF_TEST            0x6D

#define BMI160_NV_CONF              0x70
#define BMI160_NV_CONF_SPI_EN                               (1<<0)
#define BMI160_NV_CONF_I2C_WDT_SEL                          (1<<1)
#define BMI160_NV_CONF_I2C_WDT_EN                           (1<<2)
#define BMI160_NV_CONF_U_SPARE_0                            (1<<3)

#define BMI160_OFFSET_0             0x71
#define BMI160_OFFSET_1             0x72
#define BMI160_OFFSET_2             0x73
#define BMI160_OFFSET_3             0x74
#define BMI160_OFFSET_4             0x75
#define BMI160_OFFSET_5             0x76
#define BMI160_OFFSET_6             0x77
#define BMI160_STEP_CNT_0           0x78
#define BMI160_STEP_CNT_1           0x79


#define BMI160_STEP_CONF_0          0x7A
#define BMI160_STEP_CONF_0_STEPTIME_MIN_MASK                (7<<0)
#define BMI160_STEP_CONF_0_MIN_THRESHOLD_MAS                (3<<3)
#define BMI160_STEP_CONF_0_ALPHA_MASK                       (7<<4)

#define BMI160_STEP_CONF_1          0x7B
#define BMI160_STEP_CONF_1_MIN_STEP_BUF_MASK                (7<<0)
#define BMI160_STEP_CONF_1_STEP_CNT_EN                      (1<<3)

#define BMI160_CMD                  0x7E

#define BMI160_CMD_START_FOC						(3)
#define BMI160_CMD_ACC_SET_PMU_MODE_SUSPEND			(0x10)
#define BMI160_CMD_ACC_SET_PMU_MODE_NORMAL			(0x11)
#define BMI160_CMD_ACC_SET_PMU_MODE_LOWPOWER		(0x12)
#define BMI160_CMD_GYRO_SET_PMU_MODE_SUSPEND		(0x14)
#define BMI160_CMD_GYRO_SET_PMU_MODE_NORMAL			(0x15)
#define BMI160_CMD_GYRO_SET_PMU_MODE_FASTSTARTUP	(0x17)
#define BMI160_CMD_MAG_SET_PMU_MODE_SUSPEND			(0x18)
#define BMI160_CMD_MAG_SET_PMU_MODE_NORMAL			(0x19)
#define BMI160_CMD_MAG_SET_PMU_MODE_LOWPOWER		(0x1A)
#define BMI160_CMD_PROG_NVM							(0xA0)
#define BMI160_CMD_FIFO_FLUSH						(0xB0)
#define BMI160_CMD_INT_RESET						(0xB1)
#define BMI160_CMD_SOFT_RESET						(0xB6)
#define BMI160_CMD_STEP_CNT_CLR						(0xB2)

#define BMI160_FIFO_MAX_SIZE						1024

#pragma pack(push, 1)

typedef enum __BMI160_Frame_Mode {
	BMI160_FRAME_TYPE_RESV0,
	BMI160_FRAME_TYPE_CONTROL,
	BMI160_FRAME_TYPE_DATA,
	BMI160_FRAME_TYPE_RESV3,
} BMI160_FRAME_TYPE;

#define BMI160_FRAME_CONTROL_PARM_SKIP				0
#define BMI160_FRAME_CONTROL_PARM_TIME				1
#define BMI160_FRAME_CONTROL_PARM_INPUT				2

// Orable data frame param
#define BMI160_FRAME_DATA_PARM_ACCEL				1
#define BMI160_FRAME_DATA_PARM_GYRO					2
#define BMI160_FRAME_DATA_PARM_MAG					4

#define BMI160_TIME_RESOLUTION_USEC					39

typedef union __BMI160_Fifo_Header {
	uint8_t Hdr;
	struct {
		uint8_t fh_ext:2;
		uint8_t Parm:4;
		BMI160_FRAME_TYPE Type:2;
	};
} BMI160_HEADER;

#pragma pack(pop)

#ifdef __cplusplus

class AccelBmi160 : public AccelSensor {
public:
	virtual bool Init(const ACCELSENSOR_CFG &Cfg, DeviceIntrf * const pIntrf, Timer * const pTimer = NULL);
	virtual uint32_t SamplingFrequency(uint32_t Freq);
	virtual uint8_t Scale(uint8_t Value);

	/**
	 * @brief	Set and enable filter cutoff frequency
	 *
	 * Optional implementation can override this to implement filtering supported by the device
	 *
	 * @param	Freq : Filter frequency in mHz
	 *
	 * @return	Actual frequency in mHz
	 */
	virtual uint32_t FilterFreq(uint32_t Freq);
	virtual bool Enable();
	virtual void Disable();

private:
	virtual bool Init(uint32_t DevAddr, DeviceIntrf * const pIntrf, Timer * const pTimer = NULL) = 0;
};

class GyroBmi160 : public GyroSensor {
public:
	virtual bool Init(const GYROSENSOR_CFG &Cfg, DeviceIntrf * const pIntrf, Timer * const pTimer = NULL);
	virtual uint32_t SamplingFrequency(uint32_t Freq);
	virtual uint32_t Sensitivity(uint32_t Value);

	/**
	 * @brief	Set and enable filter cutoff frequency
	 *
	 * Optional implementation can override this to implement filtering supported by the device
	 *
	 * @param	Freq : Filter frequency in mHz
	 *
	 * @return	Actual frequency in mHz
	 */
	virtual uint32_t FilterFreq(uint32_t Freq);
	virtual bool Enable();
	virtual void Disable();
	virtual void Reset() {}

private:
	virtual bool Init(uint32_t DevAddr, DeviceIntrf * const pIntrf, Timer * const pTimer = NULL) = 0;
};

class MagBmi160 : public MagBmm150 {
public:
	virtual bool Init(const MAGSENSOR_CFG &Cfg, DeviceIntrf * const pIntrf, Timer * const pTimer = NULL);
	virtual uint32_t SamplingFrequency(uint32_t Freq);
	virtual bool Enable();
	virtual void Disable();
	virtual void Reset() { MagBmm150::Reset(); }

private:
	uint32_t vDevAddr;
	int Read(uint32_t DevAddr, uint8_t *pCmdAddr, int CmdAddrLen, uint8_t *pBuff, int BuffLen);
	int Write(uint32_t DevAddr, uint8_t *pCmdAddr, int CmdAddrLen, uint8_t *pData, int DataLen);
	/**
	 * @brief	Get device's map address
	 *
	 * @return	Address or chip select pin zero based index
	 */
	virtual uint32_t DeviceAddress() { return MagBmi160::vDevAddr; }
	virtual bool Init(uint32_t DevAddr, DeviceIntrf * const pIntrf, Timer * const pTimer = NULL) = 0;
};

class AgBmi160 : public AccelBmi160, public GyroBmi160, public MagBmi160 {
public:
	virtual bool Init(const ACCELSENSOR_CFG &Cfg, DeviceIntrf * const pIntrf, Timer * const pTimer = NULL) {
		vbSensorEnabled[0] = AccelBmi160::Init(Cfg, pIntrf, pTimer); return vbSensorEnabled[0];
	}
	virtual bool Init(const GYROSENSOR_CFG &Cfg, DeviceIntrf * const pIntrf, Timer * const pTimer = NULL) {
		vbSensorEnabled[1] = GyroBmi160::Init(Cfg, pIntrf, pTimer); return vbSensorEnabled[1];
	}
	virtual bool Init(const MAGSENSOR_CFG &Cfg, DeviceIntrf * const pIntrf, Timer * const pTimer = NULL) {
		vbSensorEnabled[2] = MagBmi160::Init(Cfg, pIntrf, pTimer); return vbSensorEnabled[2];
	}

	virtual bool Enable();
	virtual void Disable();
	virtual void Reset();
	virtual bool Read(ACCELSENSOR_RAWDATA &Data) { return AccelSensor::Read(Data); }
	virtual bool Read(ACCELSENSOR_DATA &Data) { return AccelSensor::Read(Data); }
	virtual bool Read(GYROSENSOR_RAWDATA &Data) { return GyroSensor::Read(Data); }
	virtual bool Read(GYROSENSOR_DATA &Data) { return GyroSensor::Read(Data); }
	virtual bool Read(MAGSENSOR_RAWDATA &Data) { return MagSensor::Read(Data); }
	virtual bool Read(MAGSENSOR_DATA &Data) { return MagSensor::Read(Data); }
	virtual void IntHandler();
	virtual bool StartSampling() { return true; }

	bool UpdateData();
protected:
	virtual bool Init(uint32_t DevAddr, DeviceIntrf * const pIntrf, Timer * const pTimer = NULL);

	int Read(uint8_t *pCmdAddr, int CmdAddrLen, uint8_t *pBuff, int BuffLen) {
		return Device::Read(pCmdAddr, CmdAddrLen, pBuff, BuffLen);
	}
	int Write(uint8_t *pCmdAddr, int CmdAddrLen, uint8_t *pData, int DataLen) {
		return Device::Write(pCmdAddr, CmdAddrLen, pData, DataLen);
	}

	bool vbInitialized;
	bool vbSensorEnabled[3];
};

#endif // __cplusplus

#endif // __AG_BMI160_H__
