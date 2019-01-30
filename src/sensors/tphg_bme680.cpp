/**-------------------------------------------------------------------------
@file	tphg_bme680.cpp

@brief	Implementation of Bosch #BME680 low power gas, pressure, temperature & humidity sensor.

The Air Quality Index is undocumented.  It requires the library
Bosch Sensortec Environmental Cluster (BSEC) Software. Download from
https://www.bosch-sensortec.com/bst/products/all_products/bsec and put in
external folder as indicated on the folder tree.

The BSEC library must be initialized in the main application prior to initializing
this driver by calling the function

bsec_library_return_t res = bsec_init();

Specs:

The BME680 is a digital 4-in-1 sensor with gas, humidity, pressure and temperature
measurement based on proven sensing principles. The sensor module is housed in an
extremely compact metal-lid LGA package with a footprint of only 3.0 × 3.0 mm2 with
a maximum height of 1.00 mm (0.93 ± 0.07 mm). Its small dimensions and its low power
consumption enable the integration in battery-powered or frequency-coupled devices,
such as handsets or wearables.

Key features

- Package : 3.0 mm x 3.0 mm x 0.93 mm metal lid LGA
- Digital interface : I2C (up to 3.4 MHz) and SPI (3 and 4 wire, up to 10 MHz)
- Supply voltage : VDD main supply voltage range: 1.71 V to 3.6 V. VDDIO interface voltage range: 1.2 V to 3.6 V
- Current consumption : 2.1 μA at 1 Hz humidity and temperature. 3.1 μA at 1 Hz pressure and temperature
3.7 μA at 1 Hz humidity, pressure and temperature. 0.09‒12 mA for p/h/T/gas depending on operation mode. 0.15 μA in sleep mode
- Operating range : -40‒+85 °C, 0‒100% r.H., 300‒1100 hPa
- Individual humidity, pressure and gas sensors can be independently enabled/disabled

Key parameters for gas sensor

- Response time (𝜏33−63%) : < 1 s (for new sensors)
- Power consumption : < 0.1 mA in ultra-low power mode
- Output data processing : direct indoor air quality (IAQ) index output

Key parameters for humidity sensor

- Response time (𝜏0−63%) : ~8 s
- Accuracy tolerance : ±3% r.H.
- Hysteresis : ±1.5% r.H.

Key parameters for pressure sensor

- RMS Noise 0.12 Pa, equiv. to 1.7 cm
- Offset temperature coefficient ±1.3 Pa/K, equiv. to ±10.9 cm at 1 °C temperature change

@author	Hoang Nguyen Hoan
@date	Oct. 15, 2017

@license

Copyright (c) 2017, I-SYST inc., all rights reserved

Permission to use, copy, modify, and distribute this software for any purpose
with or without fee is hereby granted, provided that the above copyright
notice and this permission notice appear in all copies, and none of the
names : I-SYST or its contributors may be used to endorse or
promote products derived from this software without specific prior written
permission.

For info or contributing contact : hnhoan at i-syst dot com

THIS SOFTWARE IS PROVIDED BY THE REGENTS AND CONTRIBUTORS ``AS IS'' AND ANY
EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE REGENTS OR CONTRIBUTORS BE LIABLE FOR ANY
DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

----------------------------------------------------------------------------*/
#include <stdint.h>
#include <string.h>

#ifndef __cplusplus
#include <stdbool.h>
#endif

#include "idelay.h"
#include "device_intrf.h"
#include "coredev/iopincfg.h"
#include "sensors/tphg_bme680.h"
#include "bsec_interface.h"


// 16 bits binary fixed point
static const uint32_t s_Bme860GasCalib1[16] = {
	65536, 65536, 65536, 65536, 65536, 64881, 65536, 65012, 65536, 65536, 65405, 65208, 65536, 64881, 65536, 65536
};

// 16 bits binary fixed point
const uint64_t s_Bme860GasCalib2[16] = {
	524288000000LL, 262144000000LL, 131072000000LL, 65536000000LL, 32735264735LL, 16270109232LL, 8192000000LL, 4129032258LL,
	2050050050LL, 1024000000LL, 512000000LL, 256000000LL, 128000000LL, 64000000LL, 32000000LL, 16000000LL
};

#if 0
// 16 bits binary fixed point
static const float s_fBme860GasCalib1[16] = {
	1.0, 1.0, 1.0, 1.0, 1.0, 0.99, 1.0, 0.992, 1.0, 1.0, 0.998, 0.995, 1.0, 0.99, 1.0, 1.0
};

// 16 bits binary fixed point
const float s_fBme860GasCalib2[16] = {
	8000000.0, 4000000.0, 2000000.0, 1000000.0, 499500.4995, 248262.1648, 125000.0, 63004.03226,
	31281.28128, 15625.0, 7812.5, 3906.25, 1953.125, 976.5625, 488.28125, 244.140625
};
#endif

TphgBme680::TphgBme680()
{
	vbMeasGas = false;
	vbGasData = false;
	vbSpi = false;
}

// Returns temperature in DegC, resolution is 0.01 DegC. Output value of “5123” equals 51.23 DegC.
// t_fine carries fine temperature as global value
int32_t TphgBme680::CalcTemperature(int32_t RawAdcTemp)
{
	int32_t var1, var2;

	var1 = (RawAdcTemp - ((uint32_t)vCalibData.par_T1 << 4L));
	var2 = (int32_t)(((int64_t)vCalibData.par_T2 * (int64_t)var1) >> 14LL);
	var1 = (int32_t)(((int64_t)var1 * (int64_t)var1 * (int64_t)vCalibData.par_T3) >> 30LL);
	vCalibTFine = var1 + var2;
	int32_t t = (vCalibTFine * 5L + 128L) >> 8L;

	return t;
}

// Returns pressure in Pa as unsigned 32 bit integer
// Output value in Pa
uint32_t TphgBme680::CalcPressure(int32_t RawAdcPres)
{
	int64_t var1, var2, var3;
	int64_t p;

	var1 = ((int64_t) vCalibTFine >> 1LL) - 64000LL;
	var3 = (var1 * var1) >> 15LL;
	var2 = (var3 * (int64_t)vCalibData.par_P6) +
		   ((var1 * (int64_t)vCalibData.par_P5) >> 1LL) +
		   ((int64_t)vCalibData.par_P4 << 16LL);
	var1 = (var3 * (int64_t)vCalibData.par_P3 + ((int64_t)vCalibData.par_P2 >> 1LL)) >> 18LL;
	var1 = ((var1 + 32768LL) * (uint64_t)vCalibData.par_P1) >> 15LL;

	if (var1 == 0)
	{
		return 0; // avoid exception caused by division by zero
	}

	p = 1048576LL - RawAdcPres;
	p = (p - (var2 >> 12LL)) * 3125LL;
	p = (p << 1LL) / var1;

	var3 = (p * p) >> 16LL;
	var1 = ((int64_t)vCalibData.par_P9 * var3) >> 15LL;
	var2 = (p * (int64_t)vCalibData.par_P8) >> 15LL;
	var3 = (var3 * p * (uint32_t)vCalibData.par_P10) >> 25LL;
	p += ((var1 + var2 + var3 + ((int64_t)vCalibData.par_P7 << 7LL))) >> 4LL;

	return (uint32_t)p;
}

// Returns humidity in %RH as unsigned 32 bit integer in Q22.10 format (22 integer and 10 fractional bits).
// Output value of “4744” represents 46.44 %RH
uint32_t TphgBme680::CalcHumidity(int32_t RawAdcHum)
{
	int64_t var1;
	int64_t var2;
	int64_t var3;
	int64_t var4;
	int64_t var5;
	int32_t h;
	int64_t temp_scaled = ((vTphData.Temperature << 8LL) + 50LL) / 100LL;

	var1 = (int64_t)((RawAdcHum - ((uint32_t)vCalibData.par_H1 << 4L)) << 8LL) -
		   ((temp_scaled * (int64_t)vCalibData.par_H3) >> 1LL);
	var2 = ((uint64_t)vCalibData.par_H2 * (temp_scaled * (int64_t)vCalibData.par_H4
			+ ((temp_scaled * temp_scaled * (int64_t)vCalibData.par_H5 ) >> 14LL)
			+ 4194304LL)) >> 10LL;
	var3 = (var1 * var2) >> 8LL;
	var4 = (((uint64_t)vCalibData.par_H6 << 15LL) + (int64_t)temp_scaled * (int64_t)vCalibData.par_H7) >> 4LL;
	var5 = (var4 * (var3 >> 14LL)  * (var3 >> 14LL)) >> 27LL;

	h = ((var3 + var5) * 100LL) >> 30LL;

	if (h > 10000) /* Cap at 100%rH */
		h = 10000;
	else if (h < 0)
		h = 0;

	return h;
}

uint32_t TphgBme680::CalcGas(uint16_t RawAdcGas, uint8_t Range)
{
	uint32_t gval;
	uint64_t var1;
	uint64_t var2;

 	var1 = ((1340LL + (5LL * (uint64_t) vCalibData.range_sw_err)) * (uint64_t) s_Bme860GasCalib1[Range]);
	var2 = (((uint64_t)RawAdcGas - 512LL) << 16LL) + var1;
	gval = (uint32_t)((((var1 * s_Bme860GasCalib2[Range] + (var2 >> 1)) / var2) + 32768LL) >> 16LL);

	return gval;
}
#if 0
float TphgBme680::fCalcGas(uint16_t RawAdcGas, uint8_t Range)
{
	float gval;
	float var1;
	float var2;

  	var1 = (1340.0 + 5.0 * vCalibData.range_sw_err) * s_fBme860GasCalib1[Range];
	var2 = (float)RawAdcGas - 512.0 + var1;
	gval = var1 * s_fBme860GasCalib2[Range] / var2;

	return gval;
}
#endif

uint8_t TphgBme680::CalcHeaterResistance(uint16_t Temp)
{
	uint32_t var1;
	uint32_t var2;
	uint32_t var3;
	uint32_t var4;
	uint8_t hres;

	var1 = ((int32_t)vCalibData.par_GH1 << 12L) + 3211264L;
	var2 = ((int32_t)vCalibData.par_GH2 << 1) * 33L + 154L;
	var3 = ((int32_t)vCalibData.par_GH3 << 6L);
	var4 = var1 * (65536 + var2 * (Temp << 16L) / 100L);
	uint64_t var5 = var4 + var3 * (vTphData.Temperature << 16L) / 100L;
	uint64_t var6 = 262144 / (4 + vCalibData.res_heat_range);
	uint64_t var7 = 4294967296LL / (uint64_t)(65536L + (uint32_t)vCalibData.res_heat_val * 131L);
	hres = (uint8_t)((222822L * (var5 * var6 * var7 - 1638400)) >> 16LL);

	return hres;
}

// TPH sensor init
bool TphgBme680::Init(const TPHSENSOR_CFG &CfgData, DeviceIntrf *pIntrf, Timer *pTimer)
{
	uint8_t regaddr;
	uint8_t d;

	if (pIntrf != NULL && pIntrf != Interface())
	{
		Interface(pIntrf);
		DeviceAddess(CfgData.DevAddr);
	}

	if (pTimer != NULL)
	{
		vpTimer = pTimer;
	}

	if (CfgData.DevAddr == BME680_I2C_DEV_ADDR0 || CfgData.DevAddr == BME680_I2C_DEV_ADDR1)
	{
		// I2C mode
		vbSpi = false;
	}
	else
	{
		vbSpi = true;

		// Set SPI register page to 0
		// for reading chip id later
		regaddr = BME680_REG_STATUS;
		d = 0;
		Write((uint8_t*)&regaddr, 1, &d, 1);
		vRegPage = 0;
	}

	// Read chip id
	regaddr = BME680_REG_ID;
	Read((uint8_t*)&regaddr, 1, &d, 1);
	if (d != BME680_ID)
	{
		return false;
	}

	// Device found

	DeviceID(d);
	Valid(true);

	Reset();

	usDelay(30000);

	// Load calibration data

	memset(&vCalibData, 0xFF, sizeof(vCalibData));
	regaddr = BME680_REG_CALIB_00_23_START;
	Read(&regaddr, 1, (uint8_t*)&vCalibData, 24);

	uint8_t *p =  (uint8_t*)&vCalibData;

	uint8_t cd[16];

	regaddr = BME680_REG_CALIB_24_40_START;
	Read(&regaddr, 1, cd, 16);

	vCalibData.par_H1 = ((int16_t)cd[2] << 4) | (cd[1] & 0xF);
	vCalibData.par_H2 = ((int16_t)cd[0] << 4) | (cd[1] >> 4);
	memcpy(&p[27], &cd[3], 13);

	regaddr = BME680_REG_RES_HEAT_RANGE;
	Read(&regaddr, 1, &d, 1);
	vCalibData.res_heat_range = (d & BME680_REG_RES_HEAT_RANGE_MASK) >> 4;

	regaddr = BME680_REG_RES_HEAT_VAL;
	Read(&regaddr, 1, &d, 1);
	vCalibData.res_heat_val = d;

	regaddr = BME680_REG_RANGE_SW_ERR;
	Read(&regaddr, 1, &d, 1);
	vCalibData.range_sw_err = (d & BME680_REG_RANGE_SW_ERR_MASK) >> 4;

	// Setup oversampling.  Datasheet recommend write humidity oversampling first
	// follow by temperature & pressure in single write operation
	d = 0;
	if (CfgData.HumOvrs > 0)
	{
		d = (CfgData.HumOvrs & 3);
	}

	regaddr = BME680_REG_CTRL_HUM;
	Write((uint8_t*)&regaddr, 1, &d, 1);

	// Need to keep temperature & pressure oversampling
	// because of shared register with operating mode settings

	vCtrlReg = 0;

	if (CfgData.TempOvrs > 0)
	{
		vCtrlReg |= (CfgData.TempOvrs & 3) << BME680_REG_CTRL_MEAS_OSRS_T_BITPOS;
	}

	if (CfgData.PresOvrs > 0)
	{
		vCtrlReg |= (CfgData.PresOvrs & 3) << BME680_REG_CTRL_MEAS_OSRS_P_BITPOS;
	}

	regaddr = BME680_REG_CTRL_MEAS;
	Write((uint8_t*)&regaddr, 1, &vCtrlReg, 1);


	regaddr = BME680_REG_CONFIG;
	Read((uint8_t*)&regaddr, 1, &d, 1);

	d |= (CfgData.FilterCoeff << BME680_REG_CONFIG_FILTER_BITPOS) & BME680_REG_CONFIG_FILTER_MASK;
	Write((uint8_t*)&regaddr, 1, &d, 1);

	State(SENSOR_STATE_SLEEP);

	Mode(CfgData.OpMode, CfgData.Freq);

	if (CfgData.DataRdyCB != NULL)
		TphSensor::vDataRdyHandler = CfgData.DataRdyCB;

	return true;
}

// Gas sensor init
bool TphgBme680::Init(const GASSENSOR_CFG &CfgData, DeviceIntrf *pIntrf, Timer *pTimer)
{
	if (pIntrf != NULL && pIntrf != Interface())
	{
		Interface(pIntrf);
		DeviceAddess(CfgData.DevAddr);
	}


	if (pTimer != NULL)
	{
		vpTimer = pTimer;
	}

	bsec_library_return_t bsec_status;

/*	bsec_status = bsec_init();

	if (bsec_status != BSEC_OK)
	{
		return false;
	}*/

	bsec_sensor_configuration_t requested_virtual_sensors[1];
	uint8_t n_requested_virtual_sensors = 1;

	requested_virtual_sensors[0].sensor_id = BSEC_OUTPUT_IAQ;
	// the BSEC library does not seems to work with any other setting than BSEC_SAMPLE_RATE_LP
	requested_virtual_sensors[0].sample_rate = BSEC_SAMPLE_RATE_LP;

	// Allocate a struct for the returned phyisical sensor settings
	bsec_sensor_configuration_t required_sensor_settings[BSEC_MAX_PHYSICAL_SENSOR];
	uint8_t  n_required_sensor_settings = BSEC_MAX_PHYSICAL_SENSOR;

    // Call bsec_update_subscription() to enable/disable the requested virtual sensors
	bsec_status = bsec_update_subscription(requested_virtual_sensors, n_requested_virtual_sensors, required_sensor_settings, &n_required_sensor_settings);

	if (bsec_status != BSEC_OK)
	{
		return false;
	}

	vbMeasGas = true;
	vbGasData = false;

	vNbHeatPoint = CfgData.NbHeatPoint;
	memcpy(vHeatPoints, CfgData.pHeatProfile, vNbHeatPoint * sizeof(GASSENSOR_HEAT));

	SetHeatingProfile(CfgData.NbHeatPoint, CfgData.pHeatProfile);

	uint8_t reg = BME680_REG_CTRL_GAS1;
	vCtrlGas1Reg = BME680_REG_CTRL_GAS1_RUN_GAS | ((vNbHeatPoint - 1) & BME680_REG_CTRL_GAS1_NB_CONV_MASK);

	Write(&reg, 1, &vCtrlGas1Reg, 1);

	return true;
}

/**
 * @brief	Set gas heating profile
 *
 * @param	Count : Number of heating temperature settings
 * 			pProfile : Pointer to array of temperature/duration settings
 *
 * @return	true - success
 */
bool TphgBme680::SetHeatingProfile(int Count, const GASSENSOR_HEAT *pProfile)
{
	if (Count == 0 || pProfile == NULL)
		return false;

	uint8_t regt = BME680_REG_RES_HEAT_X_START;
	uint8_t regd = BME680_REG_GAS_WAIT_X_START;

	for (int i = 0; i < Count; i++)
	{
		uint8_t ht = CalcHeaterResistance(pProfile[i].Temp);
		uint16_t dur = pProfile[i].Dur;
		uint8_t df = 0x4;
		int mul = 0;
		if (dur > 0x3F)
		{
			int delta = 0xFF;
			for (int j = 1; j < 4; j++)
			{
				if ((dur / df) < 0x40)
				{
					int x = dur % df;
					if (x < delta)
					{
						delta = x;
						mul = j;
					}
				}
				df <<= 2;
			}
			dur = dur / (1 << (mul << 1));
			dur |= (mul << 6);
		}

		Write(&regt, 1, &ht, 1);
		Write(&regd, 1, (uint8_t*)&dur, 1);

		regt++;
		regd++;
	}

	return true;
}

/**
 * @brief	Set current sensor state
 *
 * @param 	State
 *				- SENSOR_STATE_SLEEP	// Sleep state low power
 *				- SENSOR_STATE_IDLE		// Idle state powered on
 *				- SENSOR_STATE_SAMPLING	// Sampling in progress
 *
 * @return	Actual state. In the case where the new state could
 * 			not be set, it returns the actual state of the sensor.
 */
SENSOR_STATE TphgBme680::State(SENSOR_STATE State) {

	if (State == SENSOR_STATE_SLEEP)
	{
		uint8_t regaddr = BME680_REG_CTRL_MEAS;
		vCtrlReg &= ~BME680_REG_CTRL_MEAS_MODE_MASK;
		Write(&regaddr, 1, &vCtrlReg, 1);
	}

	return Sensor::State(State);
}

/**
 * @brief Set operating mode
 *
 * @param OpMode : Operating mode
 * 					- TPHSENSOR_OPMODE_SLEEP
 * 					- TPHSENSOR_OPMODE_SINGLE
 * 					- TPHSENSOR_OPMODE_CONTINUOUS
 * @param Freq : Sampling frequency in mHz for continuous mode
 *
 * @return true- if success
 */
bool TphgBme680::Mode(SENSOR_OPMODE OpMode, uint32_t Freq)
{
//	uint8_t regaddr;
	//uint8_t d = 0;

	vOpMode = OpMode;
	Sensor::SamplingFrequency(Freq);

	// read current ctrl_meas register
	//regaddr = BME680_REG_CTRL_MEAS;
	//Read(&regaddr, 1, &vCtrlReg, 1);

	vCtrlReg &= ~BME680_REG_CTRL_MEAS_MODE_MASK;

	switch (OpMode)
	{
/*		case SENSOR_OPMODE_SLEEP:
			regaddr = BME680_REG_CTRL_MEAS & vRegWrMask;
			Write(&regaddr, 1, &vCtrlReg, 1);
			break;*/
		case SENSOR_OPMODE_CONTINUOUS:
			// There is no continuous mode.  Force back to single
		case SENSOR_OPMODE_SINGLE:

			vCtrlReg |= BME680_REG_CTRL_MEAS_MODE_FORCED;
			break;
		case SENSOR_OPMODE_TIMER:
			break;
	}

	//StartSampling();

	return true;
}

/**
 * @brief	Start sampling data
 *
 * @return	true - success
 */
bool TphgBme680::StartSampling()
{
	uint8_t regaddr = BME680_REG_MEAS_STATUS_0;
	uint8_t status = 0;

	Read(&regaddr, 1, &status, 1);

	if ((status & BME680_REG_MEAS_STATUS_0_BUSY) == 0)
	{
		regaddr = BME680_REG_CTRL_MEAS;
		Write(&regaddr, 1, &vCtrlReg, 1);
		vbSampling = true;

		if (vpTimer)
		{
			vSampleTime = vpTimer->uSecond();
		}
		else
		{
			vSampleTime += vSampPeriod;
		}

	    bsec_bme_settings_t sensor_settings;

		bsec_library_return_t bsec_status = bsec_sensor_control(vSampleTime * 1000LL, &sensor_settings);

		return true;
	}

	return false;
}

bool TphgBme680::Enable()
{
	//SetMode(SENSOR_OPMODE_CONTINUOUS, vSampFreq);

	return true;
}

void TphgBme680::Disable()
{
	State(SENSOR_STATE_SLEEP);
}

void TphgBme680::Reset()
{
	uint8_t addr = 0;
	uint8_t d = BME680_REG_RESET_VAL;

	addr = BME680_REG_RESET;
	Write(&addr, 1, &d, 1);
}

bool TphgBme680::UpdateData()
{
	uint8_t addr = BME680_REG_MEAS_STATUS_0;
	uint8_t status = 0;
	uint8_t	gasidx;
	bsec_input_t inputs[BSEC_MAX_PHYSICAL_SENSOR];
	uint8_t icnt = 0;
    bsec_output_t outputs[BSEC_NUMBER_OUTPUTS];
    uint8_t ocnt = BSEC_NUMBER_OUTPUTS;

	Read(&addr, 1, &status, 1);

	gasidx = status & BME680_REG_MEAS_STATUS_0_GAS_MEAS_IDX_0;

	if (status & BME680_REG_MEAS_STATUS_0_NEW_DATA)
	{
		uint8_t d[8];
		addr = BME680_REG_PRESS_MSB;

		if (vpTimer)
		{
			vTphData.Timestamp = vpTimer->uSecond();
		}

		if (Read(&addr, 1, d, 8) == 8)
		{
			int32_t p = (((uint32_t)d[0] << 12) | ((uint32_t)d[1] << 4) | ((uint32_t)d[2] >> 4));
			int32_t t = (((uint32_t)d[3] << 12) | ((uint32_t)d[4] << 4) | ((uint32_t)d[5] >> 4));
			int32_t h = (((uint32_t)d[6] << 8) | d[7]);

			vTphData.Temperature = CalcTemperature(t);
			vTphData.Pressure = CalcPressure(p);
			vTphData.Humidity = CalcHumidity(h);
			vTphData.Timestamp = vSampleTime;

			inputs[0].sensor_id = BSEC_INPUT_TEMPERATURE;
			inputs[0].signal = vTphData.Temperature / 100.0;
			inputs[0].time_stamp = vSampleTime * 1000LL;
			inputs[1].sensor_id = BSEC_INPUT_HUMIDITY;
			inputs[1].signal = vTphData.Humidity / 100.0;
			inputs[1].time_stamp = inputs[0].time_stamp;
			inputs[2].sensor_id = BSEC_INPUT_PRESSURE;
			inputs[2].signal = vTphData.Pressure;
			inputs[2].time_stamp = inputs[0].time_stamp;
			icnt = 3;
		}

		addr = BME680_REG_GAS_R_MSB;

		if (Read(&addr, 1, d, 2) == 2)
		{
			int32_t grange = d[1] & BME680_REG_GAS_R_LSB_GAS_RANGE_R;
			int32_t gadc = (d[1] >> 5) | (d[0] << 2);
			if (d[1] & BME680_REG_GAS_R_LSB_GAS_VALID_R)// | BME680_REG_GAS_R_LSB_HEAT_STAB_R)) ==
//					(BME680_REG_GAS_R_LSB_GAS_VALID_R | BME680_REG_GAS_R_LSB_HEAT_STAB_R))
			{
				vGasData.GasRes[gasidx] = CalcGas(gadc, grange);
				vGasData.MeasIdx = gasidx;
				vbGasData = true;
				vGasData.Timestamp = vSampleTime;
				//vGasResInt = iCalcGas(gadc, grange);
				inputs[icnt].sensor_id = BSEC_INPUT_GASRESISTOR;
				inputs[icnt].signal = vGasData.GasRes[gasidx];
				inputs[icnt].time_stamp = vGasData.Timestamp * 1000LL;// g_Timer.mSecond();

				icnt++;
			}

		}

		vSampleCnt++;

		bsec_library_return_t bsec_status = bsec_do_steps(inputs, icnt, outputs, &ocnt);
		if (bsec_status == BSEC_OK)
		{
	        for (int i = 0; i < ocnt; i++)
	        {
	            if (outputs[i].sensor_id == BSEC_OUTPUT_IAQ)
	            {
					vGasData.AirQualIdx = outputs[i].signal;
                   // iaq_accuracy = bsec_outputs[index].accuracy;
	            }
	        }
		}

		vbSampling = false;

		return true;
	}

	return false;
}

bool TphgBme680::Read(TPHSENSOR_DATA &TphData)
{
	bool retval = UpdateData();

	uint8_t reg = BME680_REG_CTRL_GAS1;
	uint8_t d = vCtrlGas1Reg & ~BME680_REG_CTRL_GAS1_RUN_GAS;

	Write(&reg, 1, &d, 1);

	memcpy(&TphData, &vTphData, sizeof(TPHSENSOR_DATA));

	return retval;
}

bool TphgBme680::Read(GASSENSOR_DATA &GasData)
{
	bool retval = vbGasData;

	if (vbGasData == false)
		retval = UpdateData();

	memcpy(&GasData, &vGasData, sizeof(GASSENSOR_DATA));

	uint8_t reg = BME680_REG_CTRL_GAS1;

	//if (retval == true)
		Write(&reg, 1, &vCtrlGas1Reg, 1);

	vbGasData = false;

	return retval;
}

void TphgBme680::SelectRegPage(uint8_t Reg)
{
	// reg < 0x80 - Page 1
	// reg >= 0x80 - Page 0
	uint8_t page = (Reg ^ 0x80) >> 7;

	if (page == vRegPage)
		return;

	vRegPage = page;
	uint8_t regaddr = BME680_REG_STATUS;
	uint8_t d = (vRegPage & 1) << BME680_REG_STATUS_SPI_MEM_PG_BITPOS;
	Device::Write(&regaddr, 1, &d, 1);
}

int TphgBme680::Read(uint8_t *pCmdAddr, int CmdAddrLen, uint8_t *pBuff, int BuffLen)
{
	if (vbSpi == true)
	{
		SelectRegPage(*pCmdAddr);
		*pCmdAddr |= 0x80;
	}

	return Device::Read(pCmdAddr, CmdAddrLen, pBuff, BuffLen);
}

int TphgBme680::Write(uint8_t *pCmdAddr, int CmdAddrLen, uint8_t *pData, int DataLen)
{
	if (vbSpi == true)
	{
		SelectRegPage(*pCmdAddr);
		*pCmdAddr &= 0x7F;
	}

	return Device::Write(pCmdAddr, CmdAddrLen, pData, DataLen);
}
