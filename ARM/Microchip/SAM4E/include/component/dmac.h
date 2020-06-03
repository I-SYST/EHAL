/**
 * \file
 *
 * Copyright (c) 2014-2018 Microchip Technology Inc. and its subsidiaries.
 *
 * \asf_license_start
 *
 * \page License
 *
 * Subject to your compliance with these terms, you may use Microchip
 * software and any derivatives exclusively with Microchip products.
 * It is your responsibility to comply with third party license terms applicable
 * to your use of third party software (including open source software) that
 * may accompany Microchip software.
 *
 * THIS SOFTWARE IS SUPPLIED BY MICROCHIP "AS IS". NO WARRANTIES,
 * WHETHER EXPRESS, IMPLIED OR STATUTORY, APPLY TO THIS SOFTWARE,
 * INCLUDING ANY IMPLIED WARRANTIES OF NON-INFRINGEMENT, MERCHANTABILITY,
 * AND FITNESS FOR A PARTICULAR PURPOSE. IN NO EVENT WILL MICROCHIP BE
 * LIABLE FOR ANY INDIRECT, SPECIAL, PUNITIVE, INCIDENTAL OR CONSEQUENTIAL
 * LOSS, DAMAGE, COST OR EXPENSE OF ANY KIND WHATSOEVER RELATED TO THE
 * SOFTWARE, HOWEVER CAUSED, EVEN IF MICROCHIP HAS BEEN ADVISED OF THE
 * POSSIBILITY OR THE DAMAGES ARE FORESEEABLE.  TO THE FULLEST EXTENT
 * ALLOWED BY LAW, MICROCHIP'S TOTAL LIABILITY ON ALL CLAIMS IN ANY WAY
 * RELATED TO THIS SOFTWARE WILL NOT EXCEED THE AMOUNT OF FEES, IF ANY,
 * THAT YOU HAVE PAID DIRECTLY TO MICROCHIP FOR THIS SOFTWARE.
 *
 * \asf_license_stop
 *
 */
/*
 * Support and FAQ: visit <a href="https://www.microchip.com/support/">Microchip Support</a>
 */

#ifndef _SAM4E_DMAC_COMPONENT_
#define _SAM4E_DMAC_COMPONENT_

/* ============================================================================= */
/**  SOFTWARE API DEFINITION FOR DMA Controller */
/* ============================================================================= */
/** \addtogroup SAM4E_DMAC DMA Controller */
/*@{*/

#if !(defined(__ASSEMBLY__) || defined(__IAR_SYSTEMS_ASM__))
/** \brief DmacCh_num hardware registers */
typedef struct {
  __IO uint32_t DMAC_SADDR;   /**< \brief (DmacCh_num Offset: 0x0) DMAC Channel Source Address Register */
  __IO uint32_t DMAC_DADDR;   /**< \brief (DmacCh_num Offset: 0x4) DMAC Channel Destination Address Register */
  __IO uint32_t DMAC_DSCR;    /**< \brief (DmacCh_num Offset: 0x8) DMAC Channel Descriptor Address Register */
  __IO uint32_t DMAC_CTRLA;   /**< \brief (DmacCh_num Offset: 0xC) DMAC Channel Control A Register */
  __IO uint32_t DMAC_CTRLB;   /**< \brief (DmacCh_num Offset: 0x10) DMAC Channel Control B Register */
  __IO uint32_t DMAC_CFG;     /**< \brief (DmacCh_num Offset: 0x14) DMAC Channel Configuration Register */
  __I  uint32_t Reserved1[4];
} DmacCh_num;
/** \brief Dmac hardware registers */
#define DMACCH_NUM_NUMBER 4
typedef struct {
  __IO uint32_t   DMAC_GCFG;                      /**< \brief (Dmac Offset: 0x000) DMAC Global Configuration Register */
  __IO uint32_t   DMAC_EN;                        /**< \brief (Dmac Offset: 0x004) DMAC Enable Register */
  __IO uint32_t   DMAC_SREQ;                      /**< \brief (Dmac Offset: 0x008) DMAC Software Single Request Register */
  __IO uint32_t   DMAC_CREQ;                      /**< \brief (Dmac Offset: 0x00C) DMAC Software Chunk Transfer Request Register */
  __IO uint32_t   DMAC_LAST;                      /**< \brief (Dmac Offset: 0x010) DMAC Software Last Transfer Flag Register */
  __I  uint32_t   Reserved1[1];
  __O  uint32_t   DMAC_EBCIER;                    /**< \brief (Dmac Offset: 0x018) DMAC Error, Chained Buffer Transfer Completed Interrupt and Buffer Transfer Completed Interrupt Enable register. */
  __O  uint32_t   DMAC_EBCIDR;                    /**< \brief (Dmac Offset: 0x01C) DMAC Error, Chained Buffer Transfer Completed Interrupt and Buffer Transfer Completed Interrupt Disable register. */
  __I  uint32_t   DMAC_EBCIMR;                    /**< \brief (Dmac Offset: 0x020) DMAC Error, Chained Buffer Transfer Completed Interrupt and Buffer transfer completed Mask Register. */
  __I  uint32_t   DMAC_EBCISR;                    /**< \brief (Dmac Offset: 0x024) DMAC Error, Chained Buffer Transfer Completed Interrupt and Buffer transfer completed Status Register. */
  __O  uint32_t   DMAC_CHER;                      /**< \brief (Dmac Offset: 0x028) DMAC Channel Handler Enable Register */
  __O  uint32_t   DMAC_CHDR;                      /**< \brief (Dmac Offset: 0x02C) DMAC Channel Handler Disable Register */
  __I  uint32_t   DMAC_CHSR;                      /**< \brief (Dmac Offset: 0x030) DMAC Channel Handler Status Register */
  __I  uint32_t   Reserved2[2];
       DmacCh_num DMAC_CH_NUM[DMACCH_NUM_NUMBER]; /**< \brief (Dmac Offset: 0x3C) ch_num = 0 .. 3 */
  __I  uint32_t   Reserved3[66];
  __IO uint32_t   DMAC_WPMR;                      /**< \brief (Dmac Offset: 0x1E4) DMAC Write Protect Mode Register */
  __I  uint32_t   DMAC_WPSR;                      /**< \brief (Dmac Offset: 0x1E8) DMAC Write Protect Status Register */
} Sam4eDmac;
#endif /* !(defined(__ASSEMBLY__) || defined(__IAR_SYSTEMS_ASM__)) */
/* -------- DMAC_GCFG : (DMAC Offset: 0x000) DMAC Global Configuration Register -------- */
#define DMAC_GCFG_ARB_CFG (0x1u << 4) /**< \brief (DMAC_GCFG) Arbiter Configuration */
#define   DMAC_GCFG_ARB_CFG_FIXED (0x0u << 4) /**< \brief (DMAC_GCFG) Fixed priority arbiter (see "Basic Definitions" ) */
#define   DMAC_GCFG_ARB_CFG_ROUND_ROBIN (0x1u << 4) /**< \brief (DMAC_GCFG) Modified round robin arbiter. */
/* -------- DMAC_EN : (DMAC Offset: 0x004) DMAC Enable Register -------- */
#define DMAC_EN_ENABLE (0x1u << 0) /**< \brief (DMAC_EN) General Enable of DMA */
/* -------- DMAC_SREQ : (DMAC Offset: 0x008) DMAC Software Single Request Register -------- */
#define DMAC_SREQ_SSREQ0 (0x1u << 0) /**< \brief (DMAC_SREQ) Source Request */
#define DMAC_SREQ_DSREQ0 (0x1u << 1) /**< \brief (DMAC_SREQ) Destination Request */
#define DMAC_SREQ_SSREQ1 (0x1u << 2) /**< \brief (DMAC_SREQ) Source Request */
#define DMAC_SREQ_DSREQ1 (0x1u << 3) /**< \brief (DMAC_SREQ) Destination Request */
#define DMAC_SREQ_SSREQ2 (0x1u << 4) /**< \brief (DMAC_SREQ) Source Request */
#define DMAC_SREQ_DSREQ2 (0x1u << 5) /**< \brief (DMAC_SREQ) Destination Request */
#define DMAC_SREQ_SSREQ3 (0x1u << 6) /**< \brief (DMAC_SREQ) Source Request */
#define DMAC_SREQ_DSREQ3 (0x1u << 7) /**< \brief (DMAC_SREQ) Destination Request */
/* -------- DMAC_CREQ : (DMAC Offset: 0x00C) DMAC Software Chunk Transfer Request Register -------- */
#define DMAC_CREQ_SCREQ0 (0x1u << 0) /**< \brief (DMAC_CREQ) Source Chunk Request */
#define DMAC_CREQ_DCREQ0 (0x1u << 1) /**< \brief (DMAC_CREQ) Destination Chunk Request */
#define DMAC_CREQ_SCREQ1 (0x1u << 2) /**< \brief (DMAC_CREQ) Source Chunk Request */
#define DMAC_CREQ_DCREQ1 (0x1u << 3) /**< \brief (DMAC_CREQ) Destination Chunk Request */
#define DMAC_CREQ_SCREQ2 (0x1u << 4) /**< \brief (DMAC_CREQ) Source Chunk Request */
#define DMAC_CREQ_DCREQ2 (0x1u << 5) /**< \brief (DMAC_CREQ) Destination Chunk Request */
#define DMAC_CREQ_SCREQ3 (0x1u << 6) /**< \brief (DMAC_CREQ) Source Chunk Request */
#define DMAC_CREQ_DCREQ3 (0x1u << 7) /**< \brief (DMAC_CREQ) Destination Chunk Request */
/* -------- DMAC_LAST : (DMAC Offset: 0x010) DMAC Software Last Transfer Flag Register -------- */
#define DMAC_LAST_SLAST0 (0x1u << 0) /**< \brief (DMAC_LAST) Source Last */
#define DMAC_LAST_DLAST0 (0x1u << 1) /**< \brief (DMAC_LAST) Destination Last */
#define DMAC_LAST_SLAST1 (0x1u << 2) /**< \brief (DMAC_LAST) Source Last */
#define DMAC_LAST_DLAST1 (0x1u << 3) /**< \brief (DMAC_LAST) Destination Last */
#define DMAC_LAST_SLAST2 (0x1u << 4) /**< \brief (DMAC_LAST) Source Last */
#define DMAC_LAST_DLAST2 (0x1u << 5) /**< \brief (DMAC_LAST) Destination Last */
#define DMAC_LAST_SLAST3 (0x1u << 6) /**< \brief (DMAC_LAST) Source Last */
#define DMAC_LAST_DLAST3 (0x1u << 7) /**< \brief (DMAC_LAST) Destination Last */
/* -------- DMAC_EBCIER : (DMAC Offset: 0x018) DMAC Error, Chained Buffer Transfer Completed Interrupt and Buffer Transfer Completed Interrupt Enable register. -------- */
#define DMAC_EBCIER_BTC0 (0x1u << 0) /**< \brief (DMAC_EBCIER) Buffer Transfer Completed [3:0] */
#define DMAC_EBCIER_BTC1 (0x1u << 1) /**< \brief (DMAC_EBCIER) Buffer Transfer Completed [3:0] */
#define DMAC_EBCIER_BTC2 (0x1u << 2) /**< \brief (DMAC_EBCIER) Buffer Transfer Completed [3:0] */
#define DMAC_EBCIER_BTC3 (0x1u << 3) /**< \brief (DMAC_EBCIER) Buffer Transfer Completed [3:0] */
#define DMAC_EBCIER_CBTC0 (0x1u << 8) /**< \brief (DMAC_EBCIER) Chained Buffer Transfer Completed [3:0] */
#define DMAC_EBCIER_CBTC1 (0x1u << 9) /**< \brief (DMAC_EBCIER) Chained Buffer Transfer Completed [3:0] */
#define DMAC_EBCIER_CBTC2 (0x1u << 10) /**< \brief (DMAC_EBCIER) Chained Buffer Transfer Completed [3:0] */
#define DMAC_EBCIER_CBTC3 (0x1u << 11) /**< \brief (DMAC_EBCIER) Chained Buffer Transfer Completed [3:0] */
#define DMAC_EBCIER_ERR0 (0x1u << 16) /**< \brief (DMAC_EBCIER) Access Error [3:0] */
#define DMAC_EBCIER_ERR1 (0x1u << 17) /**< \brief (DMAC_EBCIER) Access Error [3:0] */
#define DMAC_EBCIER_ERR2 (0x1u << 18) /**< \brief (DMAC_EBCIER) Access Error [3:0] */
#define DMAC_EBCIER_ERR3 (0x1u << 19) /**< \brief (DMAC_EBCIER) Access Error [3:0] */
/* -------- DMAC_EBCIDR : (DMAC Offset: 0x01C) DMAC Error, Chained Buffer Transfer Completed Interrupt and Buffer Transfer Completed Interrupt Disable register. -------- */
#define DMAC_EBCIDR_BTC0 (0x1u << 0) /**< \brief (DMAC_EBCIDR) Buffer Transfer Completed [3:0] */
#define DMAC_EBCIDR_BTC1 (0x1u << 1) /**< \brief (DMAC_EBCIDR) Buffer Transfer Completed [3:0] */
#define DMAC_EBCIDR_BTC2 (0x1u << 2) /**< \brief (DMAC_EBCIDR) Buffer Transfer Completed [3:0] */
#define DMAC_EBCIDR_BTC3 (0x1u << 3) /**< \brief (DMAC_EBCIDR) Buffer Transfer Completed [3:0] */
#define DMAC_EBCIDR_CBTC0 (0x1u << 8) /**< \brief (DMAC_EBCIDR) Chained Buffer Transfer Completed [3:0] */
#define DMAC_EBCIDR_CBTC1 (0x1u << 9) /**< \brief (DMAC_EBCIDR) Chained Buffer Transfer Completed [3:0] */
#define DMAC_EBCIDR_CBTC2 (0x1u << 10) /**< \brief (DMAC_EBCIDR) Chained Buffer Transfer Completed [3:0] */
#define DMAC_EBCIDR_CBTC3 (0x1u << 11) /**< \brief (DMAC_EBCIDR) Chained Buffer Transfer Completed [3:0] */
#define DMAC_EBCIDR_ERR0 (0x1u << 16) /**< \brief (DMAC_EBCIDR) Access Error [3:0] */
#define DMAC_EBCIDR_ERR1 (0x1u << 17) /**< \brief (DMAC_EBCIDR) Access Error [3:0] */
#define DMAC_EBCIDR_ERR2 (0x1u << 18) /**< \brief (DMAC_EBCIDR) Access Error [3:0] */
#define DMAC_EBCIDR_ERR3 (0x1u << 19) /**< \brief (DMAC_EBCIDR) Access Error [3:0] */
/* -------- DMAC_EBCIMR : (DMAC Offset: 0x020) DMAC Error, Chained Buffer Transfer Completed Interrupt and Buffer transfer completed Mask Register. -------- */
#define DMAC_EBCIMR_BTC0 (0x1u << 0) /**< \brief (DMAC_EBCIMR) Buffer Transfer Completed [3:0] */
#define DMAC_EBCIMR_BTC1 (0x1u << 1) /**< \brief (DMAC_EBCIMR) Buffer Transfer Completed [3:0] */
#define DMAC_EBCIMR_BTC2 (0x1u << 2) /**< \brief (DMAC_EBCIMR) Buffer Transfer Completed [3:0] */
#define DMAC_EBCIMR_BTC3 (0x1u << 3) /**< \brief (DMAC_EBCIMR) Buffer Transfer Completed [3:0] */
#define DMAC_EBCIMR_CBTC0 (0x1u << 8) /**< \brief (DMAC_EBCIMR) Chained Buffer Transfer Completed [3:0] */
#define DMAC_EBCIMR_CBTC1 (0x1u << 9) /**< \brief (DMAC_EBCIMR) Chained Buffer Transfer Completed [3:0] */
#define DMAC_EBCIMR_CBTC2 (0x1u << 10) /**< \brief (DMAC_EBCIMR) Chained Buffer Transfer Completed [3:0] */
#define DMAC_EBCIMR_CBTC3 (0x1u << 11) /**< \brief (DMAC_EBCIMR) Chained Buffer Transfer Completed [3:0] */
#define DMAC_EBCIMR_ERR0 (0x1u << 16) /**< \brief (DMAC_EBCIMR) Access Error [3:0] */
#define DMAC_EBCIMR_ERR1 (0x1u << 17) /**< \brief (DMAC_EBCIMR) Access Error [3:0] */
#define DMAC_EBCIMR_ERR2 (0x1u << 18) /**< \brief (DMAC_EBCIMR) Access Error [3:0] */
#define DMAC_EBCIMR_ERR3 (0x1u << 19) /**< \brief (DMAC_EBCIMR) Access Error [3:0] */
/* -------- DMAC_EBCISR : (DMAC Offset: 0x024) DMAC Error, Chained Buffer Transfer Completed Interrupt and Buffer transfer completed Status Register. -------- */
#define DMAC_EBCISR_BTC0 (0x1u << 0) /**< \brief (DMAC_EBCISR) Buffer Transfer Completed [3:0] */
#define DMAC_EBCISR_BTC1 (0x1u << 1) /**< \brief (DMAC_EBCISR) Buffer Transfer Completed [3:0] */
#define DMAC_EBCISR_BTC2 (0x1u << 2) /**< \brief (DMAC_EBCISR) Buffer Transfer Completed [3:0] */
#define DMAC_EBCISR_BTC3 (0x1u << 3) /**< \brief (DMAC_EBCISR) Buffer Transfer Completed [3:0] */
#define DMAC_EBCISR_CBTC0 (0x1u << 8) /**< \brief (DMAC_EBCISR) Chained Buffer Transfer Completed [3:0] */
#define DMAC_EBCISR_CBTC1 (0x1u << 9) /**< \brief (DMAC_EBCISR) Chained Buffer Transfer Completed [3:0] */
#define DMAC_EBCISR_CBTC2 (0x1u << 10) /**< \brief (DMAC_EBCISR) Chained Buffer Transfer Completed [3:0] */
#define DMAC_EBCISR_CBTC3 (0x1u << 11) /**< \brief (DMAC_EBCISR) Chained Buffer Transfer Completed [3:0] */
#define DMAC_EBCISR_ERR0 (0x1u << 16) /**< \brief (DMAC_EBCISR) Access Error [3:0] */
#define DMAC_EBCISR_ERR1 (0x1u << 17) /**< \brief (DMAC_EBCISR) Access Error [3:0] */
#define DMAC_EBCISR_ERR2 (0x1u << 18) /**< \brief (DMAC_EBCISR) Access Error [3:0] */
#define DMAC_EBCISR_ERR3 (0x1u << 19) /**< \brief (DMAC_EBCISR) Access Error [3:0] */
/* -------- DMAC_CHER : (DMAC Offset: 0x028) DMAC Channel Handler Enable Register -------- */
#define DMAC_CHER_ENA0 (0x1u << 0) /**< \brief (DMAC_CHER) Enable [3:0] */
#define DMAC_CHER_ENA1 (0x1u << 1) /**< \brief (DMAC_CHER) Enable [3:0] */
#define DMAC_CHER_ENA2 (0x1u << 2) /**< \brief (DMAC_CHER) Enable [3:0] */
#define DMAC_CHER_ENA3 (0x1u << 3) /**< \brief (DMAC_CHER) Enable [3:0] */
#define DMAC_CHER_SUSP0 (0x1u << 8) /**< \brief (DMAC_CHER) Suspend [3:0] */
#define DMAC_CHER_SUSP1 (0x1u << 9) /**< \brief (DMAC_CHER) Suspend [3:0] */
#define DMAC_CHER_SUSP2 (0x1u << 10) /**< \brief (DMAC_CHER) Suspend [3:0] */
#define DMAC_CHER_SUSP3 (0x1u << 11) /**< \brief (DMAC_CHER) Suspend [3:0] */
#define DMAC_CHER_KEEP0 (0x1u << 24) /**< \brief (DMAC_CHER) Keep on [3:0] */
#define DMAC_CHER_KEEP1 (0x1u << 25) /**< \brief (DMAC_CHER) Keep on [3:0] */
#define DMAC_CHER_KEEP2 (0x1u << 26) /**< \brief (DMAC_CHER) Keep on [3:0] */
#define DMAC_CHER_KEEP3 (0x1u << 27) /**< \brief (DMAC_CHER) Keep on [3:0] */
/* -------- DMAC_CHDR : (DMAC Offset: 0x02C) DMAC Channel Handler Disable Register -------- */
#define DMAC_CHDR_DIS0 (0x1u << 0) /**< \brief (DMAC_CHDR) Disable [3:0] */
#define DMAC_CHDR_DIS1 (0x1u << 1) /**< \brief (DMAC_CHDR) Disable [3:0] */
#define DMAC_CHDR_DIS2 (0x1u << 2) /**< \brief (DMAC_CHDR) Disable [3:0] */
#define DMAC_CHDR_DIS3 (0x1u << 3) /**< \brief (DMAC_CHDR) Disable [3:0] */
#define DMAC_CHDR_RES0 (0x1u << 8) /**< \brief (DMAC_CHDR) Resume [3:0] */
#define DMAC_CHDR_RES1 (0x1u << 9) /**< \brief (DMAC_CHDR) Resume [3:0] */
#define DMAC_CHDR_RES2 (0x1u << 10) /**< \brief (DMAC_CHDR) Resume [3:0] */
#define DMAC_CHDR_RES3 (0x1u << 11) /**< \brief (DMAC_CHDR) Resume [3:0] */
/* -------- DMAC_CHSR : (DMAC Offset: 0x030) DMAC Channel Handler Status Register -------- */
#define DMAC_CHSR_ENA0 (0x1u << 0) /**< \brief (DMAC_CHSR) Enable [3:0] */
#define DMAC_CHSR_ENA1 (0x1u << 1) /**< \brief (DMAC_CHSR) Enable [3:0] */
#define DMAC_CHSR_ENA2 (0x1u << 2) /**< \brief (DMAC_CHSR) Enable [3:0] */
#define DMAC_CHSR_ENA3 (0x1u << 3) /**< \brief (DMAC_CHSR) Enable [3:0] */
#define DMAC_CHSR_SUSP0 (0x1u << 8) /**< \brief (DMAC_CHSR) Suspend [3:0] */
#define DMAC_CHSR_SUSP1 (0x1u << 9) /**< \brief (DMAC_CHSR) Suspend [3:0] */
#define DMAC_CHSR_SUSP2 (0x1u << 10) /**< \brief (DMAC_CHSR) Suspend [3:0] */
#define DMAC_CHSR_SUSP3 (0x1u << 11) /**< \brief (DMAC_CHSR) Suspend [3:0] */
#define DMAC_CHSR_EMPT0 (0x1u << 16) /**< \brief (DMAC_CHSR) Empty [3:0] */
#define DMAC_CHSR_EMPT1 (0x1u << 17) /**< \brief (DMAC_CHSR) Empty [3:0] */
#define DMAC_CHSR_EMPT2 (0x1u << 18) /**< \brief (DMAC_CHSR) Empty [3:0] */
#define DMAC_CHSR_EMPT3 (0x1u << 19) /**< \brief (DMAC_CHSR) Empty [3:0] */
#define DMAC_CHSR_STAL0 (0x1u << 24) /**< \brief (DMAC_CHSR) Stalled [3:0] */
#define DMAC_CHSR_STAL1 (0x1u << 25) /**< \brief (DMAC_CHSR) Stalled [3:0] */
#define DMAC_CHSR_STAL2 (0x1u << 26) /**< \brief (DMAC_CHSR) Stalled [3:0] */
#define DMAC_CHSR_STAL3 (0x1u << 27) /**< \brief (DMAC_CHSR) Stalled [3:0] */
/* -------- DMAC_SADDR : (DMAC Offset: N/A) DMAC Channel Source Address Register -------- */
#define DMAC_SADDR_SADDR_Pos 0
#define DMAC_SADDR_SADDR_Msk (0xffffffffu << DMAC_SADDR_SADDR_Pos) /**< \brief (DMAC_SADDR) Channel x Source Address */
#define DMAC_SADDR_SADDR(value) ((DMAC_SADDR_SADDR_Msk & ((value) << DMAC_SADDR_SADDR_Pos)))
/* -------- DMAC_DADDR : (DMAC Offset: N/A) DMAC Channel Destination Address Register -------- */
#define DMAC_DADDR_DADDR_Pos 0
#define DMAC_DADDR_DADDR_Msk (0xffffffffu << DMAC_DADDR_DADDR_Pos) /**< \brief (DMAC_DADDR) Channel x Destination Address */
#define DMAC_DADDR_DADDR(value) ((DMAC_DADDR_DADDR_Msk & ((value) << DMAC_DADDR_DADDR_Pos)))
/* -------- DMAC_DSCR : (DMAC Offset: N/A) DMAC Channel Descriptor Address Register -------- */
#define DMAC_DSCR_DSCR_Pos 2
#define DMAC_DSCR_DSCR_Msk (0x3fffffffu << DMAC_DSCR_DSCR_Pos) /**< \brief (DMAC_DSCR) Buffer Transfer Descriptor Address */
#define DMAC_DSCR_DSCR(value) ((DMAC_DSCR_DSCR_Msk & ((value) << DMAC_DSCR_DSCR_Pos)))
/* -------- DMAC_CTRLA : (DMAC Offset: N/A) DMAC Channel Control A Register -------- */
#define DMAC_CTRLA_BTSIZE_Pos 0
#define DMAC_CTRLA_BTSIZE_Msk (0xffffu << DMAC_CTRLA_BTSIZE_Pos) /**< \brief (DMAC_CTRLA) Buffer Transfer Size */
#define DMAC_CTRLA_BTSIZE(value) ((DMAC_CTRLA_BTSIZE_Msk & ((value) << DMAC_CTRLA_BTSIZE_Pos)))
#define DMAC_CTRLA_SRC_WIDTH_Pos 24
#define DMAC_CTRLA_SRC_WIDTH_Msk (0x3u << DMAC_CTRLA_SRC_WIDTH_Pos) /**< \brief (DMAC_CTRLA) Transfer Width for the Source */
#define   DMAC_CTRLA_SRC_WIDTH_BYTE (0x0u << 24) /**< \brief (DMAC_CTRLA) the transfer size is set to 8-bit width */
#define   DMAC_CTRLA_SRC_WIDTH_HALF_WORD (0x1u << 24) /**< \brief (DMAC_CTRLA) the transfer size is set to 16-bit width */
#define   DMAC_CTRLA_SRC_WIDTH_WORD (0x2u << 24) /**< \brief (DMAC_CTRLA) the transfer size is set to 32-bit width */
#define DMAC_CTRLA_DST_WIDTH_Pos 28
#define DMAC_CTRLA_DST_WIDTH_Msk (0x3u << DMAC_CTRLA_DST_WIDTH_Pos) /**< \brief (DMAC_CTRLA) Transfer Width for the Destination */
#define   DMAC_CTRLA_DST_WIDTH_BYTE (0x0u << 28) /**< \brief (DMAC_CTRLA) the transfer size is set to 8-bit width */
#define   DMAC_CTRLA_DST_WIDTH_HALF_WORD (0x1u << 28) /**< \brief (DMAC_CTRLA) the transfer size is set to 16-bit width */
#define   DMAC_CTRLA_DST_WIDTH_WORD (0x2u << 28) /**< \brief (DMAC_CTRLA) the transfer size is set to 32-bit width */
#define DMAC_CTRLA_DONE (0x1u << 31) /**< \brief (DMAC_CTRLA) Current Descriptor Stop Command and Transfer Completed Memory Indicator */
/* -------- DMAC_CTRLB : (DMAC Offset: N/A) DMAC Channel Control B Register -------- */
#define DMAC_CTRLB_SRC_DSCR (0x1u << 16) /**< \brief (DMAC_CTRLB) Source Address Descriptor */
#define   DMAC_CTRLB_SRC_DSCR_FETCH_FROM_MEM (0x0u << 16) /**< \brief (DMAC_CTRLB) Source address is updated when the descriptor is fetched from the memory. */
#define   DMAC_CTRLB_SRC_DSCR_FETCH_DISABLE (0x1u << 16) /**< \brief (DMAC_CTRLB) Buffer Descriptor Fetch operation is disabled for the source. */
#define DMAC_CTRLB_DST_DSCR (0x1u << 20) /**< \brief (DMAC_CTRLB) Destination Address Descriptor */
#define   DMAC_CTRLB_DST_DSCR_FETCH_FROM_MEM (0x0u << 20) /**< \brief (DMAC_CTRLB) Destination address is updated when the descriptor is fetched from the memory. */
#define   DMAC_CTRLB_DST_DSCR_FETCH_DISABLE (0x1u << 20) /**< \brief (DMAC_CTRLB) Buffer Descriptor Fetch operation is disabled for the destination. */
#define DMAC_CTRLB_FC_Pos 21
#define DMAC_CTRLB_FC_Msk (0x3u << DMAC_CTRLB_FC_Pos) /**< \brief (DMAC_CTRLB) Flow Control */
#define   DMAC_CTRLB_FC_MEM2MEM_DMA_FC (0x0u << 21) /**< \brief (DMAC_CTRLB) Memory-to-Memory Transfer DMAC is flow controller */
#define   DMAC_CTRLB_FC_MEM2PER_DMA_FC (0x1u << 21) /**< \brief (DMAC_CTRLB) Memory-to-Peripheral Transfer DMAC is flow controller */
#define   DMAC_CTRLB_FC_PER2MEM_DMA_FC (0x2u << 21) /**< \brief (DMAC_CTRLB) Peripheral-to-Memory Transfer DMAC is flow controller */
#define   DMAC_CTRLB_FC_PER2PER_DMA_FC (0x3u << 21) /**< \brief (DMAC_CTRLB) Peripheral-to-Peripheral Transfer DMAC is flow controller */
#define DMAC_CTRLB_SRC_INCR_Pos 24
#define DMAC_CTRLB_SRC_INCR_Msk (0x3u << DMAC_CTRLB_SRC_INCR_Pos) /**< \brief (DMAC_CTRLB) Incrementing, Decrementing or Fixed Address for the Source */
#define   DMAC_CTRLB_SRC_INCR_INCREMENTING (0x0u << 24) /**< \brief (DMAC_CTRLB) The source address is incremented */
#define   DMAC_CTRLB_SRC_INCR_DECREMENTING (0x1u << 24) /**< \brief (DMAC_CTRLB) The source address is decremented */
#define   DMAC_CTRLB_SRC_INCR_FIXED (0x2u << 24) /**< \brief (DMAC_CTRLB) The source address remains unchanged */
#define DMAC_CTRLB_DST_INCR_Pos 28
#define DMAC_CTRLB_DST_INCR_Msk (0x3u << DMAC_CTRLB_DST_INCR_Pos) /**< \brief (DMAC_CTRLB) Incrementing, Decrementing or Fixed Address for the Destination */
#define   DMAC_CTRLB_DST_INCR_INCREMENTING (0x0u << 28) /**< \brief (DMAC_CTRLB) The destination address is incremented */
#define   DMAC_CTRLB_DST_INCR_DECREMENTING (0x1u << 28) /**< \brief (DMAC_CTRLB) The destination address is decremented */
#define   DMAC_CTRLB_DST_INCR_FIXED (0x2u << 28) /**< \brief (DMAC_CTRLB) The destination address remains unchanged */
#define DMAC_CTRLB_IEN (0x1u << 30) /**< \brief (DMAC_CTRLB) Interrupt Enable Not */
/* -------- DMAC_CFG : (DMAC Offset: N/A) DMAC Channel Configuration Register -------- */
#define DMAC_CFG_SRC_PER_Pos 0
#define DMAC_CFG_SRC_PER_Msk (0xfu << DMAC_CFG_SRC_PER_Pos) /**< \brief (DMAC_CFG) Source with Peripheral identifier */
#define DMAC_CFG_SRC_PER(value) ((DMAC_CFG_SRC_PER_Msk & ((value) << DMAC_CFG_SRC_PER_Pos)))
#define DMAC_CFG_DST_PER_Pos 4
#define DMAC_CFG_DST_PER_Msk (0xfu << DMAC_CFG_DST_PER_Pos) /**< \brief (DMAC_CFG) Destination with Peripheral identifier */
#define DMAC_CFG_DST_PER(value) ((DMAC_CFG_DST_PER_Msk & ((value) << DMAC_CFG_DST_PER_Pos)))
#define DMAC_CFG_SRC_H2SEL (0x1u << 9) /**< \brief (DMAC_CFG) Software or Hardware Selection for the Source */
#define   DMAC_CFG_SRC_H2SEL_SW (0x0u << 9) /**< \brief (DMAC_CFG) Software handshaking interface is used to trigger a transfer request. */
#define   DMAC_CFG_SRC_H2SEL_HW (0x1u << 9) /**< \brief (DMAC_CFG) Hardware handshaking interface is used to trigger a transfer request. */
#define DMAC_CFG_DST_H2SEL (0x1u << 13) /**< \brief (DMAC_CFG) Software or Hardware Selection for the Destination */
#define   DMAC_CFG_DST_H2SEL_SW (0x0u << 13) /**< \brief (DMAC_CFG) Software handshaking interface is used to trigger a transfer request. */
#define   DMAC_CFG_DST_H2SEL_HW (0x1u << 13) /**< \brief (DMAC_CFG) Hardware handshaking interface is used to trigger a transfer request. */
#define DMAC_CFG_SOD (0x1u << 16) /**< \brief (DMAC_CFG) Stop On Done */
#define   DMAC_CFG_SOD_DISABLE (0x0u << 16) /**< \brief (DMAC_CFG) STOP ON DONE disabled, the descriptor fetch operation ignores DONE Field of CTRLA register. */
#define   DMAC_CFG_SOD_ENABLE (0x1u << 16) /**< \brief (DMAC_CFG) STOP ON DONE activated, the DMAC module is automatically disabled if DONE FIELD is set to 1. */
#define DMAC_CFG_LOCK_IF (0x1u << 20) /**< \brief (DMAC_CFG) Interface Lock */
#define   DMAC_CFG_LOCK_IF_DISABLE (0x0u << 20) /**< \brief (DMAC_CFG) Interface Lock capability is disabled */
#define   DMAC_CFG_LOCK_IF_ENABLE (0x1u << 20) /**< \brief (DMAC_CFG) Interface Lock capability is enabled */
#define DMAC_CFG_LOCK_B (0x1u << 21) /**< \brief (DMAC_CFG) Bus Lock */
#define   DMAC_CFG_LOCK_B_DISABLE (0x0u << 21) /**< \brief (DMAC_CFG) AHB Bus Locking capability is disabled. */
#define DMAC_CFG_LOCK_IF_L (0x1u << 22) /**< \brief (DMAC_CFG) Master Interface Arbiter Lock */
#define   DMAC_CFG_LOCK_IF_L_CHUNK (0x0u << 22) /**< \brief (DMAC_CFG) The Master Interface Arbiter is locked by the channel x for a chunk transfer. */
#define   DMAC_CFG_LOCK_IF_L_BUFFER (0x1u << 22) /**< \brief (DMAC_CFG) The Master Interface Arbiter is locked by the channel x for a buffer transfer. */
#define DMAC_CFG_AHB_PROT_Pos 24
#define DMAC_CFG_AHB_PROT_Msk (0x7u << DMAC_CFG_AHB_PROT_Pos) /**< \brief (DMAC_CFG) AHB Protection */
#define DMAC_CFG_AHB_PROT(value) ((DMAC_CFG_AHB_PROT_Msk & ((value) << DMAC_CFG_AHB_PROT_Pos)))
#define DMAC_CFG_FIFOCFG_Pos 28
#define DMAC_CFG_FIFOCFG_Msk (0x3u << DMAC_CFG_FIFOCFG_Pos) /**< \brief (DMAC_CFG) FIFO Configuration */
#define   DMAC_CFG_FIFOCFG_ALAP_CFG (0x0u << 28) /**< \brief (DMAC_CFG) The largest defined length AHB burst is performed on the destination AHB interface. */
#define   DMAC_CFG_FIFOCFG_HALF_CFG (0x1u << 28) /**< \brief (DMAC_CFG) When half FIFO size is available/filled, a source/destination request is serviced. */
#define   DMAC_CFG_FIFOCFG_ASAP_CFG (0x2u << 28) /**< \brief (DMAC_CFG) When there is enough space/data available to perform a single AHB access, then the request is serviced. */
/* -------- DMAC_WPMR : (DMAC Offset: 0x1E4) DMAC Write Protect Mode Register -------- */
#define DMAC_WPMR_WPEN (0x1u << 0) /**< \brief (DMAC_WPMR) Write Protect Enable */
#define DMAC_WPMR_WPKEY_Pos 8
#define DMAC_WPMR_WPKEY_Msk (0xffffffu << DMAC_WPMR_WPKEY_Pos) /**< \brief (DMAC_WPMR) Write Protect KEY */
#define   DMAC_WPMR_WPKEY_PASSWD (0x444D41u << 8) /**< \brief (DMAC_WPMR) Writing any other value in this field aborts the write operation of the WPEN bit.Always reads as 0. */
/* -------- DMAC_WPSR : (DMAC Offset: 0x1E8) DMAC Write Protect Status Register -------- */
#define DMAC_WPSR_WPVS (0x1u << 0) /**< \brief (DMAC_WPSR) Write Protect Violation Status */
#define DMAC_WPSR_WPVSRC_Pos 8
#define DMAC_WPSR_WPVSRC_Msk (0xffffu << DMAC_WPSR_WPVSRC_Pos) /**< \brief (DMAC_WPSR) Write Protect Violation Source */

/*@}*/


#endif /* _SAM4E_DMAC_COMPONENT_ */
