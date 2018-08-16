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

typedef struct __Analog_Comp_Dev ANALOG_COMP_DEV;

typedef void (*ANALOG_COMP_CB)(ANALOG_COMP_EVT Evt, ANALOG_COMP_DEV *pDev);

typedef struct __Analog_Comp_Cfg {
	int DevNo;				//!< Device No
	ANALOG_COMP_MODE Mode;	//!< Operating mode
	int RefSrc;				//!< Reference source voltage mV (set 0 for external source or VDD)
	int CompVolt;			//!< Comparison voltage mV
	int AnalogIn;			//!< Analog input source
	bool bHystersys;		//!< true - enable hysteresis
	int IntPrio;			//!< Interrupt priority
	ANALOG_COMP_CB EvtHandler;	//!< Pointer to event handler function
} ANALOG_COMP_CFG;

/// Device data.  The pointer to this structure is used as handle
/// for the driver.
struct __Analog_Comp_Dev {
	ANALOG_COMP_MODE Mode;	//!< Operating mode
	int RefSrc;				//!< Reference source voltage mV (set 0 for external source or VDD)
	int CompVolt;			//!< Comparison voltage mV
	int AnalogIn;			//!< Analog input source
	bool bHystersys;		//!< true - enable hysteresis
	ANALOG_COMP_CB EvtHandler;	//!< Pointer to event handler function
	void *pPrivate;			//!< Private data for use by implementer, usually is the pointer to the AnalogCmp class
	bool (*Enable)(ANALOG_COMP_DEV *pDev);
	void (*Disable)(ANALOG_COMP_DEV *pDev);
};

#ifdef __cplusplus

class AnalogComp {
public:
	virtual bool Init(const ANALOG_COMP_CFG &Cfg) = 0;
	virtual bool Enable() = 0;
	virtual void Disable() = 0;
	virtual bool Start() = 0;
	virtual void Stop() = 0;

protected:
	ANALOG_COMP_DEV vDevData;
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
bool AnalogCompInit(ANALOG_COMP_DEV *pDev, ANALOG_COMP_CFG *pCfg);

#ifdef __cplusplus
}
#endif	// __cplusplus

#endif // __ANALOG_COMP_H__

