/*
*********************************************************************************************************
*                                              uC/TCP-IP
*                                      The Embedded TCP/IP Suite
*
*                    Copyright 2004-2021 Silicon Laboratories Inc. www.silabs.com
*
*                                 SPDX-License-Identifier: APACHE-2.0
*
*               This software is subject to an open source license and is distributed by
*                Silicon Laboratories Inc. pursuant to the terms of the Apache License,
*                    Version 2.0 available at www.apache.org/licenses/LICENSE-2.0.
*
*********************************************************************************************************
*/

/*
*********************************************************************************************************
*
*                            NETWORK BOARD SUPPORT PACKAGE (BSP) FUNCTIONS
*
*                                              TEMPLATE
*
* Filename : bsp_net_eth.h
* Version  : V3.06.01
*********************************************************************************************************
*/


/*
*********************************************************************************************************
*                                               MODULE
*
* Note(s) : (1) This TCPIP device driver board support package function header file is protected from
*               multiple pre-processor inclusion through use of the TCPIP module present pre processor
*               macro definition.
*********************************************************************************************************
*/

#ifndef  BSP_NET_ETH_PRESENT                                    /* See Note #1.                                         */
#define  BSP_NET_ETH_PRESENT


/*
*********************************************************************************************************
*                                            INCLUDE FILES
*********************************************************************************************************
*/

#include  <net_if_ether.h>



/** @defgroup ETH_DMA_Interrupts ETH DMA Interrupts 
  * @{
  */ 
#define ETH_DMA_IT_TST       0x20000000U  /*!< Time-stamp trigger interrupt (on DMA) */
#define ETH_DMA_IT_PMT       0x10000000U  /*!< PMT interrupt (on DMA) */
#define ETH_DMA_IT_MMC       0x08000000U  /*!< MMC interrupt (on DMA) */
#define ETH_DMA_IT_NIS       0x00010000U  /*!< Normal interrupt summary */
#define ETH_DMA_IT_AIS       0x00008000U  /*!< Abnormal interrupt summary */
#define ETH_DMA_IT_ER        0x00004000U  /*!< Early receive interrupt */
#define ETH_DMA_IT_FBE       0x00002000U  /*!< Fatal bus error interrupt */
#define ETH_DMA_IT_ET        0x00000400U  /*!< Early transmit interrupt */
#define ETH_DMA_IT_RWT       0x00000200U  /*!< Receive watchdog timeout interrupt */
#define ETH_DMA_IT_RPS       0x00000100U  /*!< Receive process stopped interrupt */
#define ETH_DMA_IT_RBU       0x00000080U  /*!< Receive buffer unavailable interrupt */
#define ETH_DMA_IT_R         0x00000040U  /*!< Receive interrupt */
#define ETH_DMA_IT_TU        0x00000020U  /*!< Underflow interrupt */
#define ETH_DMA_IT_RO        0x00000010U  /*!< Overflow interrupt */
#define ETH_DMA_IT_TJT       0x00000008U  /*!< Transmit jabber timeout interrupt */
#define ETH_DMA_IT_TBU       0x00000004U  /*!< Transmit buffer unavailable interrupt */
#define ETH_DMA_IT_TPS       0x00000002U  /*!< Transmit process stopped interrupt */
#define ETH_DMA_IT_T         0x00000001U  /*!< Transmit interrupt */

/** @defgroup ETH_DMA_Flags ETH DMA Flags
  * @{
  */ 
#define ETH_DMA_FLAG_TST               0x20000000U  /*!< Time-stamp trigger interrupt (on DMA) */
#define ETH_DMA_FLAG_PMT               0x10000000U  /*!< PMT interrupt (on DMA) */
#define ETH_DMA_FLAG_MMC               0x08000000U  /*!< MMC interrupt (on DMA) */
#define ETH_DMA_FLAG_DATATRANSFERERROR 0x00800000U  /*!< Error bits 0-Rx DMA, 1-Tx DMA */
#define ETH_DMA_FLAG_READWRITEERROR    0x01000000U  /*!< Error bits 0-write transfer, 1-read transfer */
#define ETH_DMA_FLAG_ACCESSERROR       0x02000000U  /*!< Error bits 0-data buffer, 1-desc. access */
#define ETH_DMA_FLAG_NIS               0x00010000U  /*!< Normal interrupt summary flag */
#define ETH_DMA_FLAG_AIS               0x00008000U  /*!< Abnormal interrupt summary flag */
#define ETH_DMA_FLAG_ER                0x00004000U  /*!< Early receive flag */
#define ETH_DMA_FLAG_FBE               0x00002000U  /*!< Fatal bus error flag */
#define ETH_DMA_FLAG_ET                0x00000400U  /*!< Early transmit flag */
#define ETH_DMA_FLAG_RWT               0x00000200U  /*!< Receive watchdog timeout flag */
#define ETH_DMA_FLAG_RPS               0x00000100U  /*!< Receive process stopped flag */
#define ETH_DMA_FLAG_RBU               0x00000080U  /*!< Receive buffer unavailable flag */
#define ETH_DMA_FLAG_R                 0x00000040U  /*!< Receive flag */
#define ETH_DMA_FLAG_TU                0x00000020U  /*!< Underflow flag */
#define ETH_DMA_FLAG_RO                0x00000010U  /*!< Overflow flag */
#define ETH_DMA_FLAG_TJT               0x00000008U  /*!< Transmit jabber timeout flag */
#define ETH_DMA_FLAG_TBU               0x00000004U  /*!< Transmit buffer unavailable flag */
#define ETH_DMA_FLAG_TPS               0x00000002U  /*!< Transmit process stopped flag */
#define ETH_DMA_FLAG_T                 0x00000001U  /*!< Transmit flag */

/*
*********************************************************************************************************
*                                     EXTERNAL C LANGUAGE LINKAGE
*
* Note(s) : (1) C++ compilers MUST 'extern'ally declare ALL C function prototypes & variable/object
*               declarations for correct C language linkage.
*********************************************************************************************************
*/

#ifdef __cplusplus
extern  "C" {                                                   /* See Note #1.                                         */
#endif


/*
*********************************************************************************************************
*                           NETWORK BOARD SUPPORT PACKAGE (BSP) ERROR CODES
*********************************************************************************************************
*/


/*
*********************************************************************************************************
*                                          GLOBAL VARIABLES
*********************************************************************************************************
*/

#ifdef  NET_IF_ETHER_MODULE_EN
extern  const  NET_DEV_BSP_ETHER  NetDev_BSP_STM32F4xx;
#endif

void  NetDev_IntHandler (void);
/*
*********************************************************************************************************
*                                   EXTERNAL C LANGUAGE LINKAGE END
*********************************************************************************************************
*/

#ifdef __cplusplus
}                                                               /* End of 'extern'al C lang linkage.                    */
#endif


/*
*********************************************************************************************************
*                                              MODULE END
*********************************************************************************************************
*/

#endif
