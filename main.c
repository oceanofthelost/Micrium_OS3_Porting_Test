/**
network libarary hasn't been tried.
If you use OS without network, delete remove Network folder,in this case STM32F4xx unnecessary ,remove it.

**/

#include "stm32f4xx.h"                  // Device header
#include "os.h"
#include "main.h"
#include "gpio.h"
#include "utility.h"

#include  <lib_mem.h>
#include  <net.h>
#include  <net_ascii.h>
#include  <net_if.h>
#include  <net_if_ether.h>
#include  <net_ipv4.h>
#include  <net_dev_cfg.h>                      /* See Note #1. */
//#include  <net_dev_ether_template_dma.h> /* See Note #2. */
#include  <net_phy.h>                 /* See Note #3. */
#include  <net_bsp.h>                    /* See Note #4. */
#include "net_dev_gmac.h"
#include "lib_math.h"

#define	BTN_PRESSED   0x01U

void SystemClock_Config(void);
void NVICs_init(void);

void ledTask(void *p);

OS_ERR osErr;
OS_TCB ledCtrlTaskTCB;
CPU_STK ledCtrlTaskSTK[128U];

CPU_INT64U diffTimeF = 0;
CPU_INT64U diffTimeE = 0;
CPU_INT64U diffTimeRes = 0;

OS_SEM  sem;

int main()
{
	LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_SYSCFG);
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_PWR);
	
	NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_4);
	
	SystemClock_Config();
	SysTick_Config(SystemCoreClock / 1000);
	
	
	MX_GPIO_Init();

	CPU_Init();                                                 /* Initialize the uC/CPU services                       */
  Mem_Init();                                                 /* Initialize Memory Mmanagment Module                  */
  Math_Init(); 
	
	OSInit(&osErr);
	
	OSTaskCreate( &ledCtrlTaskTCB,
								"Led Task",
								ledTask,
								(void*)0,
								1,
								ledCtrlTaskSTK,
								0,
								128U,
								0,
								0,
								0,
								OS_OPT_TASK_STK_CHK + OS_OPT_TASK_STK_CLR,
								&osErr
							);
								
	OSSemCreate(&sem,
              "Button Semaphore",
              0,
              &osErr);
	
		OSStart(&osErr);
		
	  NET_IF_NBR      if_nbr;
    NET_ERR         err_net;
		NET_IPv4_ADDR   addr_ipv4;
    NET_IPv4_ADDR   msk_ipv4;
    NET_IPv4_ADDR   gateway_ipv4;
		
		err_net = Net_Init(&NetRxTaskCfg,                   /* See Note #6.                               */
                       &NetTxDeallocTaskCfg,
                       &NetTmrTaskCfg);
											 
    if (err_net != NET_ERR_NONE) 
		{
			//todo:
    }
		
		                                                    /* --------- ADD ETHERNET INTERFACE --------- */
                                                        /* See Note #7.                               */
    if_nbr = NetIF_Add((void *)&NetIF_API_Ether,                /* See Note #7b.                              */
                       (void *)&NetDev_API_GMAC,    /* Device driver API,    See Note #7c.        */
                       (void *)&NetDev_BSP_STM32F4xx,        /* BSP API,              See Note #7d.        */
                       (void *)&NetDev_Cfg_Ether_1,             /* Device configuration, See Note #7e.        */
                       (void *)&NetPhy_API_Generic,             /* PHY driver API,       See Note #7f.        */
                       (void *)&NetPhy_Cfg_Ether_1,             /* PHY configuration,    See Note #7g.        */
                       &err_net);
											 
    if (err_net != NET_IF_ERR_NONE) 
	  {
			   // todo:
    }
		
		                                                    /* -------- START ETHERNET INTERFACE -------- */
    NetIF_Start(if_nbr, &err_net);                      /* See Note #8.                               */
    if (err_net != NET_IF_ERR_NONE) {
			//todo:
		}
			
			                                                    /* ------- CONFIGURE IPV4 STATIC ADDR ------- */
                                                        /* See Note #9                                */
    NetASCII_Str_to_IP("192.168.1.36",                   /* Convert IPv4 string addr to 32 bits addr.  */
                       &addr_ipv4,
                       NET_IPv4_ADDR_SIZE,
                      &err_net);
    NetASCII_Str_to_IP("255.255.255.0",                 /* Convert IPv4 mask string to 32 bits addr.  */
                       &msk_ipv4,
                       NET_IPv4_ADDR_SIZE,
                      &err_net);
    NetASCII_Str_to_IP("192.168.1.1",                    /* Convert Gateway string to 32 bits addr.    */
                       &gateway_ipv4,
                       NET_IPv4_ADDR_SIZE,
                      &err_net);
    NetIPv4_CfgAddrAdd(if_nbr,                          /* Add a statically-configured IPv4 host ...  */
                       addr_ipv4,                       /* ... addr, subnet mask, & default      ...  */
                       msk_ipv4,                        /* ... gateway to the interface. See Note #10.*/
                       gateway_ipv4,
                      &err_net);
											
    if (err_net != NET_IPv4_ERR_NONE) 
			{
        //todo:
      }
}

void btnPressed(void)
{
	OS_SEM_CTR  ctr;
	OS_ERR    err;

	ctr = OSSemPost(&sem,OS_OPT_POST_1 + OS_OPT_POST_NO_SCHED, &err);
	(void)ctr;
}

void ledTask(void *p)
{
	(void)&p;
	
	OS_ERR      err;
	OS_SEM_CTR  ctr;
	CPU_TS      ts;
	
	while(DEF_ON)
	{
//		OSTimeDlyHMSM(0, 0, 0, 500, OS_OPT_TIME_HMSM_STRICT, &osErr);
		
		ctr = OSSemPend(&sem,
                    0,
                    OS_OPT_PEND_BLOCKING,
                    &ts,
                    &err);
		(void)ctr;

			LL_GPIO_TogglePin(GPIOD, LL_GPIO_PIN_14);
		

		
		diffTimeF = CPU_TS32_to_uSec(CPU_TS_Get32());
		diffTimeE = CPU_TS32_to_uSec(CPU_TS_Get32());
	  diffTimeRes = diffTimeE - diffTimeF;
	}
}



void SystemClock_Config(void)
{
  LL_FLASH_SetLatency(LL_FLASH_LATENCY_5);
  while(LL_FLASH_GetLatency()!= LL_FLASH_LATENCY_5)
  {
  }
  LL_PWR_SetRegulVoltageScaling(LL_PWR_REGU_VOLTAGE_SCALE1);
  LL_RCC_HSE_EnableBypass();
  LL_RCC_HSE_Enable();

   /* Wait till HSE is ready */
  while(LL_RCC_HSE_IsReady() != 1)
  {

  }
  LL_RCC_PLL_ConfigDomain_SYS(LL_RCC_PLLSOURCE_HSE, LL_RCC_PLLM_DIV_4, 168, LL_RCC_PLLP_DIV_2);
  LL_RCC_PLL_Enable();

   /* Wait till PLL is ready */
  while(LL_RCC_PLL_IsReady() != 1)
  {

  }
  LL_RCC_SetAHBPrescaler(LL_RCC_SYSCLK_DIV_1);
  LL_RCC_SetAPB1Prescaler(LL_RCC_APB1_DIV_4);
  LL_RCC_SetAPB2Prescaler(LL_RCC_APB2_DIV_2);
  LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_PLL);

   /* Wait till System clock is ready */
  while(LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_PLL)
  {

  }
  LL_Init1msTick(168000000);
  LL_SetSystemCoreClock(168000000);
}

void NVICs_init(void)
{
   
//   NVIC_ClearPendingIRQ(NonMaskableInt_IRQn);
//   NVIC_EnableIRQ(NonMaskableInt_IRQn);
//   
//   NVIC_ClearPendingIRQ(MemoryManagement_IRQn);
//   NVIC_EnableIRQ(MemoryManagement_IRQn);
//   
//   NVIC_ClearPendingIRQ(BusFault_IRQn);
//   NVIC_EnableIRQ(BusFault_IRQn);
//   
//   NVIC_ClearPendingIRQ(UsageFault_IRQn);
//   NVIC_EnableIRQ(UsageFault_IRQn);
//   
//   NVIC_ClearPendingIRQ(SVCall_IRQn);
//   NVIC_EnableIRQ(SVCall_IRQn);
//   
//   NVIC_ClearPendingIRQ(DebugMonitor_IRQn);
//   NVIC_EnableIRQ(DebugMonitor_IRQn);
//   
//   NVIC_ClearPendingIRQ(PendSV_IRQn);
//   NVIC_EnableIRQ(PendSV_IRQn);
	 
	 NVIC_ClearPendingIRQ(SysTick_IRQn);
   NVIC_SetPriority(SysTick_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),15, 0));
   NVIC_EnableIRQ(SysTick_IRQn);
}
