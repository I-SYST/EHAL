/**-------------------------------------------------------------------------
@file	analog_comp.h

@brief	Generic analog comparator


@author	Hoang Nguyen Hoan
@date	Aug. 13, 2018

@license

Copyright (c) 2018, I-SYST inc., all rights reserved

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

#ifndef __ANALOG_COMP_H__
#define __ANALOG_COMP_H__

#include <inttypes.h>

typedef enum __Analog_Comp_Mode {
	ANALOG_COMP_MODE_SINGLE,		//!< Single shot
	ANALOG_COMP_MODE_CONTINUOUS		//!< Continuous
} ANALOG_COMP_MODE;

typedef enum __Analog_Comp_Evt {
	ANALOG_COMP_EVT_UNKNOWN,
	ANALOG_COMP_EVT_LOWER,	//!< Transition lower
	ANALOG_COMP_EVT_HIGHER	//!< Transition higher
} ANALOG_COMP_EVT;

typedef struct __Analog_Comp_Dev AnalogCompDev_t;

typedef void (*AnalogCompEvtHandler)(AnalogCompDev_t *pDev, ANALOG_COMP_EVT Evt);

#pragma pack(push, 4)

typedef struct __Analog_Comp_Cfg {
	int DevNo;				//!< Device No
	ANALOG_COMP_MODE Mode;	//!< Operating mode
	int RefSrc;				//!< Reference source voltage mV (set 0 for internal source or external source # + 1)
	int RefVolt;			//!< Reference voltage
	int CompVolt;			//!< Comparison voltage mV
	int AnalogIn;			//!< Analog input source
	bool bHystersys;		//!< true - enable hysteresis
	int IntPrio;			//!< Interrupt priority
	AnalogCompEvtHandler EvtHandler;	//!< Pointer to event handler function
} AnalogCompCfg_t;

/// Device data.  The pointer to this structure is used as handle
/// for the driver.
struct __Analog_Comp_Dev {
	ANALOG_COMP_MODE Mode;	//!< Operating mode
	int RefSrc;				//!< Reference source voltage mV (set 0 for external source or VDD)
	uint16_t ExtVRefPin;	//!< External reference voltage pin
	int CompVolt;			//!< Comparison voltage mV
	int AnalogIn;			//!< Analog input source
	bool bHystersys;		//!< true - enable hysteresis
	AnalogCompEvtHandler EvtHandler;	//!< Pointer to event handler function
	void *pPrivate;			//!< Private data for use by implementer, usually is the pointer to the AnalogCmp class
	bool (*Enable)(AnalogCompDev_t *pDev);
	void (*Disable)(AnalogCompDev_t *pDev);
	bool (*Start)(AnalogCompDev_t *pDev);
	void (*PowerOff)(AnalogCompDev_t *pDev);
};

#pragma pack(pop)

#ifdef __cplusplus

class AnalogComp {
public:
	virtual bool Init(const AnalogCompCfg_t &Cfg);
	virtual bool Enable() { return vDevData.Enable(&vDevData); }
	virtual void Disable() { vDevData.Disable(&vDevData); }
	virtual bool Start() { return vDevData.Start(&vDevData); };
	virtual void PowerOff() { vDevData.PowerOff(&vDevData); };

protected:
	AnalogCompDev_t vDevData;
private:
};

extern "C" {
#endif	// __cplusplus

// C function prototypes

/**
 * @brief	Analog comparator driver instantiation initialization
 *
 * This function must be implemented by driver implementer
 */
bool AnalogCompInit(AnalogCompDev_t *pDev, AnalogCompCfg_t *pCfg);

#ifdef __cplusplus
}
#endif	// __cplusplus

#endif // __ANALOG_COMP_H__

