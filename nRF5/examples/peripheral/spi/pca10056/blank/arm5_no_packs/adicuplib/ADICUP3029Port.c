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

#include "nrf_gpiote.h"
#include "nrf_drv_ppi.h"
#include "nrf_drv_timer.h"
#include "nrf_drv_gpiote.h"
#include <stdbool.h>
#include <stdint.h>
#include "nrf.h"
#include "nrf_drv_systick.h"




#define HAL_MAX_DELAY      0xFFFFFFFFU
#define SPI_INSTANCE  0 /**< SPI instance index. */
static const nrf_drv_spi_t spi = NRF_DRV_SPI_INSTANCE(SPI_INSTANCE);  /**< SPI instance. */
static volatile bool spi_xfer_done;  /**< Flag used to indicate that SPI instance completed the transfer. */
///////////////////
#define TEST_STRING "Nordic"
static uint8_t       m_tx_buf[] = TEST_STRING;           /**< TX buffer. */
static uint8_t       m_rx_buf[sizeof(TEST_STRING) + 1];    /**< RX buffer. */
static const uint8_t m_length = sizeof(m_tx_buf);        /**< Transfer length. */


    

#define AD5940SPI                          NRF_SPIM0
#define AD5940_MISO_PIN                    SPI_MISO_PIN

#define AD5940_MOSI_PIN                    SPI_MOSI_PIN

#define AD5940_CS_PIN                      SPI_SS_PIN

#define AD5940_RST_PIN                     SPI_RESET_PIN

#define AD5940_GP0INT_PIN                  SPI_RX_PIN

#define AD5940_GP0INT_IRQn                 SPI_IRQ_PRIORITY




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






#define SYSTICK_MAXCOUNT ((1L<<24)-1) /* we use Systick to complete function Delay10uS(). This value only applies to ADICUP3029 board. */
#define SYSTICK_CLKFREQ  100000000L    /* Systick clock frequency in Hz. This only appies to ADICUP3029 board */
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
      nrf_gpio_pin_clear(AD5940_CS_PIN);
}

void AD5940_CsSet(void)
{
      nrf_gpio_pin_set(AD5940_CS_PIN);
}

void AD5940_RstSet(void)
{     
      nrf_gpio_pin_set(AD5940_RST_PIN);
}

void AD5940_RstClr(void)
{  
      nrf_gpio_pin_clear(AD5940_RST_PIN);
}

void AD5940_Delay10us(uint32_t time)
{
   time/=100;
  if(time == 0) time =1;
 
	uint32_t tickstart= nrf_systick_val_get();
	uint32_t wait= time;
	
	 if (wait < HAL_MAX_DELAY)
  {
    wait += (uint32_t)(1U);
  }

  while((nrf_systick_val_get() - tickstart) < wait)
  {
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
	nrf_drv_gpiote_init();


  nrf_gpio_cfg_output(SPI_SCK_PIN);
	nrf_gpio_cfg_output(SPI_MISO_PIN);
	nrf_gpio_cfg_output(SPI_MOSI_PIN);
	nrf_gpio_cfg_output(SPI_SS_PIN);
	nrf_gpio_cfg_output(SPI_RESET_PIN);
	
	AD5940_CsSet();
  AD5940_RstSet();
    nrf_drv_spi_config_t spi_config = NRF_DRV_SPI_DEFAULT_CONFIG;
                spi_config.ss_pin   = SPI_SS_PIN;
                spi_config.miso_pin = SPI_MISO_PIN;
                spi_config.mosi_pin = SPI_MOSI_PIN;
                spi_config.sck_pin  = SPI_SCK_PIN;
	
	APP_ERROR_CHECK(nrf_drv_spi_init(&spi, &spi_config, spi_event_handler, NULL));

	
	nrf_gpio_cfg_input(AD5940_GP0INT_PIN, NRF_GPIO_PIN_NOPULL);
  nrf_gpiote_event_configure(0,AD5940_GP0INT_PIN,NRF_GPIOTE_POLARITY_TOGGLE);
	NRF_GPIOTE->INTENSET = GPIOTE_INTENSET_IN0_Enabled;
	NVIC_EnableIRQ(GPIOTE_IRQn);
	

  return 0;
}

/* MCU related external line interrupt service routine */
//
void Ext_Int0_Handler()
{
//	NRF_SPIS0
//  pADI_XINT0->CLR = BITM_XINT_CLR_IRQ0;
   ucInterrupted = 1;
 /* This example just set the flag and deal with interrupt in AD5940Main function. It's your choice to choose how to process interrupt. */
   NRF_GPIOTE->EVENTS_IN[0]=0;

}


