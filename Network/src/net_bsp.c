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
* Filename : bsp_net_eth.c
* Version  : V3.06.01
*********************************************************************************************************
*/


/*
*********************************************************************************************************
*                                            INCLUDE FILES
*********************************************************************************************************
*/

#include <net_if_ether.h>
#include "main.h"
#include "net_bsp.h"
#include <stm32f4xx_gpio.h>
#include <stm32f4xx_rcc.h>
#include "stm32f4xx_syscfg.h"
#include "os.h"
#include "stm32f4xx_it.h"
#include "net_def.h"
#include "net_type.h"

/*
*********************************************************************************************************
*                                            LOCAL DEFINES
*********************************************************************************************************
*/
#define BSP_ETH_PHY_MODE_CONFIG            NET_PHY_BUS_MODE_RMII

/*
*********************************************************************************************************
*                                           LOCAL CONSTANTS
*********************************************************************************************************
*/


/*
*********************************************************************************************************
*                                          LOCAL DATA TYPES
*********************************************************************************************************
*/


/*
*********************************************************************************************************
*                                            LOCAL TABLES
*********************************************************************************************************
*/


/*
*********************************************************************************************************
*                                       LOCAL GLOBAL VARIABLES
*********************************************************************************************************
*/


/*
*********************************************************************************************************
*                                  ETHERNET DEVICE INTERFACE NUMBERS
*
* Note(s) : (1) Each network device maps to a unique network interface number.
*********************************************************************************************************
*/

static  NET_IF_NBR  Board_IF_Nbr = NET_IF_NBR_NONE;


/*
*********************************************************************************************************
*                                      LOCAL FUNCTION PROTOTYPES
*********************************************************************************************************
*/

#ifdef  NET_IF_ETHER_MODULE_EN

static  void        NetDev_CfgClk    (NET_IF   *p_if,
                                                NET_ERR  *p_err);

static  void        NetDev_CfgIntCtrl(NET_IF   *p_if,
                                                NET_ERR  *p_err);

static  void        NetDev_CfgGPIO   (NET_IF   *p_if,
                                                NET_ERR  *p_err);

static  CPU_INT32U  NetDev_ClkFreqGet(NET_IF   *p_if,
                                                NET_ERR  *p_err);

        void        BSP_NET_Template_IntHandler(void);


/*
*********************************************************************************************************
*                                    ETHERNET DEVICE BSP INTERFACE
*********************************************************************************************************
*/

const  NET_DEV_BSP_ETHER  NET_DrvBSP = {
                                        .CfgClk = NetDev_CfgClk,
                                        .CfgIntCtrl = NetDev_CfgIntCtrl,
                                        .CfgGPIO = NetDev_CfgGPIO,
                                        .ClkFreqGet = NetDev_ClkFreqGet
                                       };

																			 
/********************************************************************************************************
 *                                  NETWORK CONTROLLER CONFIGURATION
 *
 * Note(s) : (1) (a) Buffer & memory sizes & alignments configured in number of octets.
 *               (b) Data bus size configured in number of bits.
 *
 *           (2) (a) All network buffer data area sizes MUST be configured greater than or equal to
 *                   NET_BUF_DATA_SIZE_MIN.
 *               (b) Large transmit buffer data area sizes MUST be configured greater than or equal to
 *                   small transmit buffer data area sizes.
 *               (c) Small transmit buffer data area sizes MAY need to be configured greater than or
 *                   equal to the specific interface's minimum packet size.
 *
 *           (3) (a) MUST configure at least one (1) large receive  buffer.
 *               (b) MUST configure at least one (1) transmit buffer, however, zero (0) large OR
 *                   zero (0) small transmit buffers MAY be configured.
 *
 *           (4) Some processors or controllers may be more efficient & may even REQUIRE that buffer data
 *               areas align to specific CPU-word/octet address boundaries in order to successfully
 *               read/write data from/to controller. Therefore, it is recommended to align controllers'
 *               buffer data areas to the processor's or controller's data bus width.
 *
 *           (5) Positive offset from base receive/transmit index, if required by controller (or driver) :
 *
 *               (a) (1) Some controller's may receive or buffer additional octets prior to the actual
 *                       received packet. Thus an offset may be required to ignore these additional octets :
 *
 *                       (A) If a controller does NOT receive or buffer any  additional octets prior to
 *                           received packets, then the default offset of '0' SHOULD be configured.
 *
 *                       (B) However, if a controller does receive or buffer additional octets prior to
 *                           received packets, then configure the controller's receive offset with the
 *                           number of additional octets.
 *
 *                   (2) Some controllers/drivers may require additional octets prior to the actual
 *                       transmit packet. Thus an offset may be required to reserve additional octets :
 *
 *                       (A) If a controller/driver does NOT require any  additional octets prior to
 *                           transmit packets, then the default offset of '0' SHOULD be configured.
 *
 *                       (B) However, if a controller/driver does require additional octets prior to
 *                           transmit packets, then configure the controller's transmit offset with the
 *                           number of additional octets.
 *
 *               (b) Since each network buffer data area allocates additional octets for its configured
 *                   offset(s), the network buffer data area size does NOT need to be increased by the
 *                   number of additional offset octets.
 *
 *           (6) Flags to configure (optional) controller features; bit-field flags logically OR'd :
 *
 *               (a) NET_DEV_CFG_FLAG_NONE           No device configuration flags selected.
 *
 *               (b) NET_DEV_CFG_FLAG_SWAP_OCTETS    Swap data octets [i.e. swap data words' high-order
 *                                                       octet(s) with data words' low-order octet(s),
 *                                                       & vice-versa] if required by device-to-CPU data
 *                                                       bus wiring &/or CPU endian word order.
 *
 *           (7) Network controllers with receive descriptors MUST configure the number of receive buffers
 *               greater than the number of receive descriptors.
 *******************************************************************************************************/

//                                                                 Modify structure according to your application needs and controller particularities.
const NET_DEV_CFG_ETHER BSP_NetEther_CtrlrCfg = {
  .RxBufPoolType = NET_IF_MEM_TYPE_MAIN,                        // Desired receive buffer memory pool type :
                                                                // NET_IF_MEM_TYPE_MAIN : buffers allocated from main memory
                                                                // NET_IF_MEM_TYPE_DEDICATED : bufs alloc from dedicated memory
  .RxBufLargeSize = 1520u,                                      // Desired size of large receive buffers (in octets) [see Note #2].
  .RxBufLargeNbr = 10u,                                         // Desired number of large receive buffers [see Note #3a].
  .RxBufAlignOctets = 32u,                                      // Desired alignment of receive buffers (in octets) [see Note #4].
  .RxBufIxOffset = 0u,                                          // Desired offset from base receive index, if needed (in octets) [see Note #5a1].

  .TxBufPoolType = NET_IF_MEM_TYPE_MAIN,                        // Desired transmit buffer memory pool type :
                                                                // NET_IF_MEM_TYPE_MAIN : buffers allocated from main memory
                                                                // NET_IF_MEM_TYPE_DEDICATED : bufs alloc from dedicated memory
  .TxBufLargeSize = 1520u,                                      // Desired size of large transmit buffers (in octets) [see Note #2].
  .TxBufLargeNbr = 8u,                                          // Desired number of large transmit buffers [see Note #3b].
  .TxBufSmallSize = 60u,                                        // Desired size of small transmit buffers (in octets) [see Note #2].
  .TxBufSmallNbr = 4u,                                          // Desired number of small transmit buffers [see Note #3b].
  .TxBufAlignOctets = 16u,                                      // Desired alignment of transmit buffers (in octets) [see Note #4].
  .TxBufIxOffset = 0u,                                          // Desired offset from base transmit index, if needed (in octets) [see Note #5a2].

  .MemAddr = 0x00000000u,                                       // Base address of dedicated memory, if available.
  .MemSize = 0xFFFFu,                                           // Size of dedicated memory, if available (in octets).

  .Flags = NET_DEV_CFG_FLAG_NONE,                               // Desired option flags, if any (see Note #6).

  .RxDescNbr = 5u,                                              // Desired number of controller's receive descriptors (see Note #7).
  .TxDescNbr = 12u,                                             // Desired number of controller's transmit descriptors.

  .BaseAddr = ETH_BASE,                                         // Base address of controller's hardware/registers.

  .DataBusSizeNbrBits = 0u,                                     // Size of controller's data bus (in bits), if available.

  .HW_AddrStr = { DEF_NULL },                                   // Desired MAC hardware address; may be NULL address or string ...
                                                                // ... if  address configured or set at run-time.
};

/********************************************************************************************************
 ********************************************************************************************************
 *                                      NETWORK PHY CONFIGURATION
 ********************************************************************************************************
 *******************************************************************************************************/

const NET_PHY_CFG_ETHER BSP_NetEther_PhyCfg = {
	
  .BusAddr = NET_PHY_ADDR_AUTO,                                 // Phy bus address.
  .BusMode = BSP_ETH_PHY_MODE_CONFIG,                           // Phy bus mode.
  .Type = NET_PHY_TYPE_EXT,                                     // Phy type.
  .Spd = NET_PHY_SPD_AUTO,                                      // Auto-Negotiation determines link speed.
  .Duplex = NET_PHY_DUPLEX_AUTO,                                // Auto-Negotiation determines link duplex.
};

/*
*********************************************************************************************************
*                                     LOCAL CONFIGURATION ERRORS
*********************************************************************************************************
*/


/*
*********************************************************************************************************
*********************************************************************************************************
*                                           LOCAL FUNCTIONS
*********************************************************************************************************
*********************************************************************************************************
*/

/*
*********************************************************************************************************
*                                      BSP_NET_Template_CfgClk()
*
* Description : Configure clocks for the specified interface/device.
*
* Argument(s) : p_if     Pointer to network interface to configure.
*
*               p_err    Pointer to variable that will receive the return error code from this function:
*
*                            NET_DEV_ERR_NONE    Device clock(s) successfully configured.
*
* Return(s)   : none.
*
* Note(s)     : none.
*********************************************************************************************************
*/

static  void  NetDev_CfgClk(NET_IF   *p_if, NET_ERR  *p_err)
{
		NET_PHY_CFG_ETHER  *pphy_cfg;


    pphy_cfg = p_if->Ext_Cfg;                                    /* Obtain pointer to Phy cfg.                           */
    if (pphy_cfg == (NET_PHY_CFG_ETHER *)0u) {
       *p_err = NET_DEV_ERR_INVALID_CFG;
        return;
    }

    RCC_AHB1PeriphResetCmd(RCC_AHB1Periph_ETH_MAC, ENABLE);
    RCC_AHB1PeriphResetCmd(RCC_AHB1Periph_ETH_MAC, DISABLE);


    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_ETH_MAC     |
                           RCC_AHB1Periph_ETH_MAC_Tx  |
                           RCC_AHB1Periph_ETH_MAC_Rx,
                           ENABLE);

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA       |
                           RCC_AHB1Periph_GPIOB       |
                           RCC_AHB1Periph_GPIOC       |
                           RCC_AHB1Periph_GPIOE,
                           ENABLE);
    
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);  


    RCC_MCO1Config(RCC_MCO1Source_PLLCLK, RCC_MCO1Div_2);
    SYSCFG_ETH_MediaInterfaceConfig(SYSCFG_ETH_MediaInterface_RMII);

    *p_err = NET_DEV_ERR_NONE;
}


/*
*********************************************************************************************************
*                                    BSP_NET_Template_CfgIntCtrl()
*
* Description : Configure interrupts and/or interrupt controller for the specified interface/device.
*
* Argument(s) : p_if     Pointer to network interface to configure.
*
*               p_err    Pointer to variable that will receive the return error code from this function:
*
*                            NET_DEV_ERR_NONE    Device interrupt(s) successfully configured.
*
* Return(s)   : none.
*
* Note(s)     : none.
*********************************************************************************************************
*/

static  void  NetDev_CfgIntCtrl (NET_IF   *p_if, NET_ERR  *p_err)
{
    Board_IF_Nbr = p_if->Nbr; 
	
	//rx interrupt enable
		ETH->DMAIER |= ETH_DMA_IT_NIS | ETH_DMA_IT_R;
	
	 /* ETH interrupt Init */
    NVIC_EnableIRQ(ETH_IRQn);

    *p_err = NET_DEV_ERR_NONE;
}


/*
*********************************************************************************************************
*                                     BSP_NET_Template_CfgGPIO()
*
* Description : Configure general-purpose I/O (GPIO) for the specified interface/device.
*
* Argument(s) : p_if     Pointer to network interface to configure.
*
*               p_err    Pointer to variable that will receive the return error code from this function:
*
*                            NET_DEV_ERR_NONE    Device GPIO successfully configured.
*
* Return(s)   : none.
*
* Note(s)     : none.
*********************************************************************************************************
*/

static  void  NetDev_CfgGPIO (NET_IF   *p_if, NET_ERR  *p_err)
{
    (void)p_if;                                                 
    
	CPU_INT16U          i;  
    NET_PHY_CFG_ETHER  *pphy_cfg;
    GPIO_InitTypeDef    GPIO_InitStructure;


    pphy_cfg = p_if->Ext_Cfg;                                    /* Obtain pointer to Phy cfg.                           */
    if (pphy_cfg == (NET_PHY_CFG_ETHER *)0u) {
       *p_err = NET_DEV_ERR_INVALID_CFG;
        return;
    }
  
    GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_7;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL ;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource1, GPIO_AF_ETH);
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource2, GPIO_AF_ETH);
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource7, GPIO_AF_ETH);


    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10 | GPIO_Pin_11 | GPIO_Pin_12 | GPIO_Pin_13;
    GPIO_Init(GPIOB, &GPIO_InitStructure);
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource10, GPIO_AF_ETH);	
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource11, GPIO_AF_ETH);
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource12, GPIO_AF_ETH);
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource13, GPIO_AF_ETH);


    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1 | GPIO_Pin_4 | GPIO_Pin_5;
    GPIO_Init(GPIOC, &GPIO_InitStructure);
    GPIO_PinAFConfig(GPIOC, GPIO_PinSource1, GPIO_AF_ETH);
    GPIO_PinAFConfig(GPIOC, GPIO_PinSource4, GPIO_AF_ETH);
    GPIO_PinAFConfig(GPIOC, GPIO_PinSource5, GPIO_AF_ETH);


    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOE, &GPIO_InitStructure);

    GPIO_ResetBits(GPIOE, GPIO_Pin_2);	
    for (i = 0; i < 20000; i++);
    GPIO_SetBits(GPIOE, GPIO_Pin_2);
    for (i = 0; i < 20000; i++);

    *p_err = NET_DEV_ERR_NONE;
}


/*
*********************************************************************************************************
*                                    BSP_NET_Template_ClkFreqGet()
*
* Description : Get device clock frequency.
*
* Argument(s) : p_if     Pointer to network interface to get clock frequency.
*
*               p_err    Pointer to variable that will receive the return error code from this function:
*
*                            NET_DEV_ERR_NONE    Device clock frequency successfully returned.
*
* Return(s)   : Device clock frequency (in Hz).
*
* Note(s)     : none.
*********************************************************************************************************
*/

static  CPU_INT32U  NetDev_ClkFreqGet (NET_IF   *p_if, NET_ERR  *p_err)
{
    CPU_INT32U  clk_freq;


    (void)p_if;                                                 /* Prevent 'variable unused' compiler warning.          */
    (void)clk_freq;

    clk_freq = SystemCoreClock;

    *p_err = NET_DEV_ERR_NONE;

    return (clk_freq);
}


/*
*********************************************************************************************************
*                                    BSP_NET_Template_IntHandler()
*
* Description : BSP-level ISR handler(s) for device interrupts.
*
* Argument(s) : none.
*
* Return(s)   : none.
*
* Note(s)     : none.
*********************************************************************************************************
*/

void  NetDev_IntHandler (void)
{
    NET_ERR  err;
		NetIF_ISR_Handler(Board_IF_Nbr, NET_DEV_ISR_TYPE_UNKNOWN, &err);
}

/*
*********************************************************************************************************
*                                              MODULE END
*********************************************************************************************************
*/

#endif  /* NET_IF_ETHER_MODULE_EN */
