/*--------------------------------------------------------------------------
File   : atomic.h

Author : Hoang Nguyen Hoan          Sep. 12, 1996

Desc   : Atomic operations.
         Because of it's platform dependent nature, this file requires conditional
         compilation for each platform port.

         Compile macro :
            WIN32             - Windows
            __TCS__           - Trimedia
            __ADSPBLACKFIN__  - ADSP Blackfin

Copyright (c) 1996-2008, I-SYST, all rights reserved

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

----------------------------------------------------------------------------
Modified by         Date           	Description
Hoan				17 nov. 2014	Adapt to GNU GCC
----------------------------------------------------------------------------*/
#ifndef __ATOMIC_H__
#define __ATOMIC_H__

#include <signal.h>

#if defined(_WIN32) || defined(WIN32)
//
// MS Windows
//
#define WIN32_LEAN_AND_MEAN
#include <windows.h>

#elif defined(__TCS__)
//
// Trimedia/Nexperia
//
//#include "tmlib/AppModel.h"

#elif defined(__ADSPBLACKFIN__)
//
// ADI Blackfin
//
#include <ccblkfn.h>
#elif defined(__GNUC__)
//GCC_VERSION) && GCC_VERSION >= 40700
#ifdef __arm__
#if defined ( __GNUC__ )
#ifndef __ASM
	#define __ASM            __asm                                      /*!< asm keyword for GNU Compiler */
#endif
#ifndef __INLINE
	#define __INLINE         inline                                     /*!< inline keyword for GNU Compiler */
#endif
#ifndef __STATIC_INLINE
	#define __STATIC_INLINE  static inline
#endif
#endif

#include "core_cmFunc.h"
#endif

#else
#pragma message ("Platform undefined")
#error Platform not implemented

#endif   // Platform definitions

#include "istddef.h"

/**
 * Atomic increment
 *
 * @Param   pVar : Pointer to data value to be increased
 *
 * @Return  Newly incremented value
 */
#if defined(__TSOK__) || defined(__ADSPBLACKFIN__)

sig_atomic_t AtomicInc(sig_atomic_t *pVar);

#else

static inline sig_atomic_t AtomicInc(sig_atomic_t *pVar) {

#if defined(_WIN32) || defined(WIN32)
//
// MS Windows
//
   return InterlockedIncrement((LONG *)pVar);
#elif defined(__TCS__)  // Trimedia
//
// Trimedia
//
	#pragma TCS_atomic
    // AppModel_suspend_scheduling();
   	return ++(*pVar);
   	//   AppModel_resume_scheduling();
#elif defined(__GNUC__)
   	__atomic_store_n(pVar, *pVar + 1, __ATOMIC_SEQ_CST);
    return *pVar;
#else
#error Platform not implemented
#endif

}
#endif

/**
 * Atomic decrement
 *
 * @Param   pVar : Pointer to data value to be decreased
 *
 * @Return  Newly decremented value
 */
#if defined(__TSOK__) || defined(__ADSPBLACKFIN__)

sig_atomic_t AtomicDec(sig_atomic_t *pVar);

#else

static inline sig_atomic_t AtomicDec(sig_atomic_t *pVar) {

#if defined(_WIN32) || defined(WIN32)
//
// MS Windows
//
	return InterlockedDecrement((LONG *)pVar);

#elif defined(__TCS__)
//
// Trimedia
//
	#pragma TCS_atomic
	//   AppModel_suspend_scheduling();
	return --(*pVar);
	//   AppModel_resume_scheduling();
#elif defined(__GNUC__)
	return __atomic_fetch_sub (pVar, 1, __ATOMIC_SEQ_CST);
#endif
}
#endif  // __TSOK__

/**
 * Atomic assign value
 *
 * @Param   pVar   : Pointer to data value to be decreased
 * @param   NewVal : New value to be assigned to pVar
 */
#if defined(__TSOK__) || defined(__ADSPBLACKFIN__)

void AtomicAssign(sig_atomic_t *pVar, sig_atomic_t NewVal);

#else

static inline void AtomicAssign(sig_atomic_t *pVar, sig_atomic_t NewVal) {

#if defined(_WIN32) || defined(WIN32)
//
// MS Windows
//
   InterlockedExchange((LONG *)pVar, (LONG)NewVal);

#elif defined(__TCS__)
//
// Trimedia
//
   #pragma TCS_atomic
//   AppModel_suspend_scheduling();
   *pVar = NewVal;
//   AppModel_resume_scheduling();
#elif defined(__GNUC__)
   __atomic_store_n (pVar, NewVal, __ATOMIC_SEQ_CST);
#endif
}
#endif // __TSOK__

static inline uint32_t EnterCriticalSection(void) {
#ifdef __arm__
	uint32_t __state = __get_PRIMASK();
	__disable_irq();
	return __state;
#endif
}

static inline void ExitCriticalSection(uint32_t State) {
#ifdef __arm__
	__set_PRIMASK(State);
#endif
}

#ifdef __arm__
static inline uint32_t DisableInterrupt() {
	uint32_t __primmask = __get_PRIMASK();
	__disable_irq();
	return __primmask;
}

static inline void EnableInterrupt(uint32_t __primmask) {
	__set_PRIMASK(__primmask);
}
#endif

#endif // __ATOMIC_H__




