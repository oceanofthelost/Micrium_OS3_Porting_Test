#include "stm32f4xx_it.h"
#include "main.h"
#include "net_bsp.h"




/**
  * @brief This function handles Non maskable interrupt.
  */
void NMI_Handler(void)
{
  while (1)
  {
  }

}

/**
  * @brief This function handles Hard fault interrupt.
  */
void HardFault_Handler(void)
{
  while (1)
  {
  }
}

/**
  * @brief This function handles Memory management fault.
  */
void MemManage_Handler(void)
{
  while (1)
  {
  }
}

/**
  * @brief This function handles Pre-fetch fault, memory access fault.
  */
void BusFault_Handler(void)
{
  while (1)
  {
  }
}

/**
  * @brief This function handles Undefined instruction or illegal state.
  */
void UsageFault_Handler(void)
{

  while (1)
  {
  }
}

/**
  * @brief This function handles System service call via SWI instruction.
  */
void SVC_Handler(void)
{

}

/**
  * @brief This function handles Debug monitor.
  */
void DebugMon_Handler(void)
{

}

/**
  * @brief This function handles Pendable request for system service.
  */
//void PendSV_Handler(void)
//{

//}

volatile uint32_t systick;
/**
  * @brief This function handles System tick timer.
  */
//void SysTick_Handler(void)
//{
//	systick++;
//}

/**
  * @brief This function handles EXTI line0 interrupt.
  */
void EXTI0_IRQHandler(void)
{
  if (LL_EXTI_IsActiveFlag_0_31(LL_EXTI_LINE_0) != RESET)
  {
    LL_EXTI_ClearFlag_0_31(LL_EXTI_LINE_0);
		os_intHandler(btnPressed);
  }

}

void ETH_IRQHandler(void)
{
		if((ETH->DMASR & ETH_DMA_FLAG_R) == ETH_DMA_FLAG_R)
	{
		CLEAR_BIT(ETH->DMASR, ETH_DMA_FLAG_R | ETH_DMA_FLAG_NIS);
		os_intHandler(NetDev_IntHandler);
	}
}

void os_intHandler(CPU_FNCT_VOID  isr)
{
		CPU_SR_ALLOC();
		CPU_CRITICAL_ENTER(); /* Tell the OS that we are starting an ISR            */

    OSIntEnter();

    CPU_CRITICAL_EXIT();
		if (isr)
			isr();

		OSIntExit(); /* Tell the OS that we are leaving the ISR            */
}
