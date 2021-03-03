/*!
 *****************************************************************************
 @file:    ADICUP3029Port.c
 @author:  Neo Xu
 @brief:   The port for ADI's ADICUP3029 board.
 -----------------------------------------------------------------------------

Copyright (c) 2017-2019 Analog Devices, Inc. All Rights Reserved.

This software is proprietary to Analog Devices, Inc. and its licensors.
By using this software you agree to the terms of the associated
Analog Devices Software License Agreement.

*****************************************************************************/

#include "ad5940.h"
#include <nrf52840.h>

////////////////
#include "nrf_drv_spi.h"
#include "app_util_platform.h"
#include "nrf_gpio.h"
#include "nrf_delay.h"
#include "boards.h"
#include "app_error.h"
#include <string.h>
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"


#define SPI_INSTANCE  0 /**< SPI instance index. */
static const nrf_drv_spi_t spi = NRF_DRV_SPI_INSTANCE(SPI_INSTANCE);  /**< SPI instance. */
static volatile bool spi_xfer_done;  /**< Flag used to indicate that SPI instance completed the transfer. */
///////////////////
#define TEST_STRING "Nordic"
static uint8_t       m_tx_buf[] = TEST_STRING;           /**< TX buffer. */
static uint8_t       m_rx_buf[sizeof(TEST_STRING) + 1];    /**< RX buffer. */
static const uint8_t m_length = sizeof(m_tx_buf);        /**< Transfer length. */

    

#define AD5940SPI                          NRF_SPI


void spi_event_handler(nrf_drv_spi_evt_t const * p_event,
                       void *                    p_context)
{
    spi_xfer_done = true;
    NRF_LOG_INFO("Transfer completed.");
    if (m_rx_buf[0] != 0)
    {
        NRF_LOG_INFO(" Received:");
        NRF_LOG_HEXDUMP_INFO(m_rx_buf, strlen((const char *)m_rx_buf));
    }
}
/*
#define AD5940_CLK_ENABLE()                __HAL_RCC_SPI1_CLK_ENABLE()
#define AD5940_SCK_GPIO_CLK_ENABLE()       spi_config.sck_pin
#define AD5940_MISO_GPIO_CLK_ENABLE()      spi_config.miso_pin
#define AD5940_MOSI_GPIO_CLK_ENABLE()      spi_config.mosi_pin
#define AD5940_CS_GPIO_CLK_ENABLE()        spi_config.ss_pin
#define AD5940_RST_GPIO_CLK_ENABLE()       __HAL_RCC_GPIOB_CLK_ENABLE()
#define AD5940_GP0INT_GPIO_CLK_ENABLE()    __HAL_RCC_GPIOA_CLK_ENABLE()

#define AD5940SPI_FORCE_RESET()               __HAL_RCC_SPI1_FORCE_RESET()
#define AD5940SPI_RELEASE_RESET()             __HAL_RCC_SPI1_RELEASE_RESET()
*/

/* Definition for AD5940 Pins */
/*
#define AD5940_SCK_PIN                     GPIO_PIN_5
#define AD5940_SCK_GPIO_PORT               GPIOA
#define AD5940_SCK_AF                      GPIO_AF5_SPI1
#define AD5940_MISO_PIN                    GPIO_PIN_6
#define AD5940_MISO_GPIO_PORT              GPIOA
#define AD5940_MISO_AF                     GPIO_AF5_SPI1
#define AD5940_MOSI_PIN                    GPIO_PIN_7
#define AD5940_MOSI_GPIO_PORT              GPIOA
#define AD5940_MOSI_AF                     GPIO_AF5_SPI1

#define AD5940_CS_PIN                      GPIO_PIN_6
#define AD5940_CS_GPIO_PORT                GPIOB

#define AD5940_RST_PIN                     GPIO_PIN_0   //A3
#define AD5940_RST_GPIO_PORT               GPIOB

#define AD5940_GP0INT_PIN                  GPIO_PIN_10   //A3
#define AD5940_GP0INT_GPIO_PORT            GPIOA
#define AD5940_GP0INT_IRQn                 EXTI15_10_IRQn
*/






#define SYSTICK_MAXCOUNT ((1L<<24)-1) /* we use Systick to complete function Delay10uS(). This value only applies to ADICUP3029 board. */
#define SYSTICK_CLKFREQ   26000000L   /* Systick clock frequency in Hz. This only appies to ADICUP3029 board */
volatile static uint32_t ucInterrupted = 0;       /* Flag to indicate interrupt occurred */

/**
	@brief Using SPI to transmit N bytes and return the received bytes. This function targets to 
         provide a more efficient way to transmit/receive data.
	@param pSendBuffer :{0 - 0xFFFFFFFF}
      - Pointer to the data to be sent.
	@param pRecvBuff :{0 - 0xFFFFFFFF}
      - Pointer to the buffer used to store received data.
	@param length :{0 - 0xFFFFFFFF}
      - Data length in SendBuffer.
	@return None.
**/
void AD5940_ReadWriteNBytes(unsigned char *pSendBuffer,unsigned char *pRecvBuff,unsigned long length)
{               
	              nrf_drv_spi_transfer(&spi, m_tx_buf, m_length, m_rx_buf, m_length);
}

void AD5940_CsClr(void)
{
      NRF_P0->OUTCLR = (1<<10);
}

void AD5940_CsSet(void)
{
      NRF_P0->OUTSET = (1<<10); //p2.6-ADC3-A3
}

void AD5940_RstSet(void)
{
      NRF_P1->OUTSET = 1<<6;
}

void AD5940_RstClr(void)
{
      NRF_P1->OUTCLR = 1<<6;
}

void AD5940_Delay10us(uint32_t time)
{
  if(time==0)return;
  if(time*10<SYSTICK_MAXCOUNT/(SYSTICK_CLKFREQ/1000000)){
    SysTick->LOAD = time*10*(SYSTICK_CLKFREQ/1000000);
    SysTick->CTRL = (1 << 2) | (1<<0);    /* Enable SysTick Timer, using core clock */
    while(!((SysTick->CTRL)&(1<<16)));    /* Wait until count to zero */
    SysTick->CTRL = 0;                    /* Disable SysTick Timer */
  }
  else {
    AD5940_Delay10us(time/2);
    AD5940_Delay10us(time/2 + (time&1));
  }
}

uint32_t AD5940_GetMCUIntFlag(void)
{
   return ucInterrupted;
}

uint32_t AD5940_ClrMCUIntFlag(void)
{
  
   ucInterrupted = 0;
   return 1;
}

/* Functions that used to initialize MCU platform */

uint32_t AD5940_MCUResourceInit(void *pCfg)
{
	
  nrf_drv_spi_config_t spi_config = NRF_DRV_SPI_DEFAULT_CONFIG;
                spi_config.ss_pin   = SPI_SS_PIN;
                spi_config.miso_pin = SPI_MISO_PIN;
                spi_config.mosi_pin = SPI_MOSI_PIN;
                spi_config.sck_pin  = SPI_SCK_PIN;
	
	
	APP_ERROR_CHECK(nrf_drv_spi_init(&spi, &spi_config, spi_event_handler, NULL));

	
	            
 
	
  /* Step1, initialize SPI peripheral and its GPIOs for CS/RST */
  /*
	AD5940_SCK_GPIO_CLK_ENABLE();
  AD5940_MISO_GPIO_CLK_ENABLE();
  AD5940_MOSI_GPIO_CLK_ENABLE();
  AD5940_CS_GPIO_CLK_ENABLE();
  AD5940_RST_GPIO_CLK_ENABLE();
  */
	/* Enable SPI clock */
  /*
	AD5940_CLK_ENABLE(); 
  
  GPIO_InitStruct.Pin       = AD5940_SCK_PIN;
  GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull      = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = AD5940_SCK_AF;
  HAL_GPIO_Init(AD5940_SCK_GPIO_PORT, &GPIO_InitStruct);
  */
  
	/* SPI MISO GPIO pin configuration  */
  /*
	GPIO_InitStruct.Pin = AD5940_MISO_PIN;
  GPIO_InitStruct.Alternate = AD5940_MISO_AF;
  HAL_GPIO_Init(AD5940_MISO_GPIO_PORT, &GPIO_InitStruct);
  */
  /* SPI MOSI GPIO pin configuration  */
 /*
	GPIO_InitStruct.Pin = AD5940_MOSI_PIN;
  GPIO_InitStruct.Alternate = AD5940_MOSI_AF;
  HAL_GPIO_Init(AD5940_MOSI_GPIO_PORT, &GPIO_InitStruct);
  */
	/* SPI CS GPIO pin configuration  */
  /*
	GPIO_InitStruct.Pin = AD5940_CS_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  HAL_GPIO_Init(AD5940_CS_GPIO_PORT, &GPIO_InitStruct);
  */
  /* SPI RST GPIO pin configuration  */
  /*
	GPIO_InitStruct.Pin = AD5940_RST_PIN;
  HAL_GPIO_Init(AD5940_RST_GPIO_PORT, &GPIO_InitStruct);
  
  AD5940_CsSet();
  AD5940_RstSet();
  */
  /* Set the SPI parameters */
  /*
	SpiHandle.Instance               = AD5940SPI;
  SpiHandle.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8; //SPI clock should be < AD5940_SystemClock
  SpiHandle.Init.Direction         = SPI_DIRECTION_2LINES;
  SpiHandle.Init.CLKPhase          = SPI_PHASE_1EDGE;
  SpiHandle.Init.CLKPolarity       = SPI_POLARITY_LOW;
  SpiHandle.Init.DataSize          = SPI_DATASIZE_8BIT;
  SpiHandle.Init.FirstBit          = SPI_FIRSTBIT_MSB;
  SpiHandle.Init.TIMode            = SPI_TIMODE_DISABLE;
  SpiHandle.Init.CRCCalculation    = SPI_CRCCALCULATION_DISABLE;
  SpiHandle.Init.CRCPolynomial     = 7;
  SpiHandle.Init.NSS               = SPI_NSS_SOFT;
  SpiHandle.Init.Mode = SPI_MODE_MASTER;
  HAL_SPI_Init(&SpiHandle);
  */
	
  /* Step 2: Configure external interrupot line */
  /*
	AD5940_GP0INT_GPIO_CLK_ENABLE();
  GPIO_InitStruct.Pin       = AD5940_GP0INT_PIN;
  GPIO_InitStruct.Mode      = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull      = GPIO_PULLUP;
  GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = 0;
  HAL_GPIO_Init(AD5940_GP0INT_GPIO_PORT, &GPIO_InitStruct);
  */
  /* Enable and set EXTI Line0 Interrupt to the lowest priority */
  //HAL_NVIC_EnableIRQ(AD5940_GP0INT_IRQn);
//  HAL_NVIC_SetPriority(AD5940_GP0INT_IRQn, 0, 0);
  return 0;
}

/* MCU related external line interrupt service routine */

//void Ext_Int0_Handler()
//{
//  pADI_XINT0->CLR = BITM_XINT_CLR_IRQ0;
//  ucInterrupted = 1;
 /* This example just set the flag and deal with interrupt in AD5940Main function. It's your choice to choose how to process interrupt. */
//}




