/**-------------------------------------------------------------------------
@file	mag_sensor.h

@brief	Generic magnetometer sensor abstraction

@author	Hoang Nguyen Hoan
@date	Nov. 18, 2017

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

#ifndef __MAG_SENSOR_H__
#define __MAG_SENSOR_H__

#include <stdint.h>

#include "coredev/iopincfg.h"
#include "sensors/sensor.h"

#pragma pack(push, 1)

/// Magnetometer raw sensor data
typedef struct __MagSensor_Raw_Data {
    uint64_t Timestamp; 		//!< Time stamp count in usec
    uint16_t Sensitivity[3];	//!< Scale factor in nanoTesla of the sensor
    union {
        int16_t Val[3];
        struct {
            int16_t X;          //!< X axis
            int16_t Y;          //!< Y axis
            int16_t Z;          //!< Z axis
        };
    };
} MAGSENSOR_RAWDATA;

/// Magnetometer sensor data
typedef struct __MagSensor_Data {
	uint64_t Timestamp;			//!< Time stamp count in usec
	union {
	    float Val[3];	//!< Mag data in uT
		struct {
	        float X;			//!< X axis
	        float Y;			//!< Y axis
	        float Z;			//!< Z axis
		};
	};
} MAGSENSOR_DATA;

typedef enum __MagSensor_Precision {
	MAGSENSOR_PRECISION_LOW,
	MAGSENSOR_PRECISION_HIGH
} MAGSENSOR_PRECISION;

/// Mag configuration data
typedef struct __MagSensor_Config {
	uint32_t		DevAddr;	//!< Either I2C 7 bits device address or CS index select if SPI is used
	SENSOR_OPMODE 	OpMode;		//!< Operating mode
	uint32_t		Freq;		//!< Sampling frequency in mHz (miliHertz) if continuous mode is used
	MAGSENSOR_PRECISION	Precision;	//!< Sampling precision in bits
	bool 			bInter;		//!< true - enable interrupt
	DEVINTR_POL		IntPol;		//!< Interrupt polarity
} MAGSENSOR_CFG;

#pragma pack(pop)

#ifdef __cplusplus

class MagSensor : public Sensor {
public:
	/**
	 * @brief	Sensor initialization
	 *
	 * @param 	Cfg		: Sensor configuration data
	 * @param 	pIntrf	: Pointer to communication interface
	 * @param 	pTimer	: Pointer to Timer use for time stamp
	 *
	 * @return	true - Success
	 */
	virtual bool Init(const MAGSENSOR_CFG &Cfg, DeviceIntrf * const pIntrf, Timer * const pTimer) = 0;

    virtual bool Read(MAGSENSOR_RAWDATA &Data) {
        Data = vData;
        return true;
    }

    /**
	 * @brief	Read last updated sensor data
	 *
	 * This function read the currently stored data last updated by UdateData().
	 * Device implementation can add validation if needed and return true or false
	 * in the case of data valid or not.  This default implementation only returns
	 * the stored data with success.
	 *
	 * @param 	Data : Reference to data storage for the returned data
	 *
	 * @return	True - Success.
	 */
    virtual bool Read(MAGSENSOR_DATA &Data);

    virtual MAGSENSOR_PRECISION Precision() { return vPrecision; }
    virtual MAGSENSOR_PRECISION Precision(MAGSENSOR_PRECISION Val) { vPrecision = Val; return vPrecision; }
    virtual void SetCalibration(float (&Gain)[3][3], float (&Offset)[3]);
    virtual void ClearCalibration();
    virtual void Sensitivity(uint16_t (&Sen)[3]);
    MagSensor() {
    	Type(SENSOR_TYPE_MAG);
    	ClearCalibration();
    }

protected:
    // These functions allow override for device hook up on the secondary interface
	virtual int Read(uint8_t *pCmdAddr, int CmdAddrLen, uint8_t *pBuff, int BuffLen) {
		return Read(DeviceAddress(), pCmdAddr, CmdAddrLen, pBuff, BuffLen);
	}
	virtual int Write(uint8_t *pCmdAddr, int CmdAddrLen, uint8_t *pData, int DataLen) {
		return Write(DeviceAddress(), pCmdAddr, CmdAddrLen, pData, DataLen);
	}
	virtual int Read(uint32_t DevAddr, uint8_t *pCmdAddr, int CmdAddrLen, uint8_t *pBuff, int BuffLen) {
		return vpIntrf->Read(DevAddr, pCmdAddr, CmdAddrLen, pBuff, BuffLen);
	}
	virtual int Write(uint32_t DevAddr, uint8_t *pCmdAddr, int CmdAddrLen, uint8_t *pData, int DataLen) {
		return vpIntrf->Write(DevAddr, pCmdAddr, CmdAddrLen, pData, DataLen);
	}

	uint16_t vSensitivity[3];		//!< Sample scaling factor in nanoTesla
	MAGSENSOR_PRECISION vPrecision;	//!< Sampling precision in bits
	MAGSENSOR_RAWDATA vData;	//!< Current sensor data updated with UpdateData()
	float vCalibGain[3][3];
	float vCalibOffset[3];
private:

};

#endif // __cplusplus

#endif // __MAG_SENSOR_H__
