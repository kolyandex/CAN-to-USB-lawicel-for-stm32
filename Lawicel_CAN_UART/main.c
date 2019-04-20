#include <stm32f10x.h>

#include <stm32f10x_rcc.h>
#include <stm32f10x_tim.h>
#include <stm32f10x_gpio.h>
#include <stm32f10x_usart.h>
#include <stm32f10x_dma.h>
#include <stm32f10x_can.h>
#include <misc.h>
#include <stdio.h>
#include <string.h>


#define CAN1_ReMap

#ifndef CAN1_ReMap
#define CAN1_GPIO_PORT			GPIOA
#define CAN1_RX_SOURCE			GPIO_Pin_11
#define CAN1_TX_SOURCE			GPIO_Pin_12
#define CAN1_Periph				RCC_APB2Periph_GPIOA
#else
#define CAN1_GPIO_PORT			GPIOB
#define CAN1_RX_SOURCE			GPIO_Pin_8
#define CAN1_TX_SOURCE			GPIO_Pin_9
#define CAN1_Periph				RCC_APB2Periph_GPIOB
#endif

#define USART_RX_SIZE 1024
uint8_t USART_RX_Buff[USART_RX_SIZE];

CanTxMsg TxMessage;

uint8_t interface_state = 0;

#define BUFF_SIZE 4096
uint8_t Buff1[BUFF_SIZE];
uint8_t Buff2[BUFF_SIZE];
uint8_t TxBuf[BUFF_SIZE];

uint8_t Buf1Busy = 0, Buf2Busy = 0;
uint16_t Buf1Idx = 0, Buf2Idx = 0;

void USARTSendDMA(char* pucBuffer)
{
	uint8_t len = strlen(pucBuffer);
	memset(TxBuf, 0x00, BUFF_SIZE);
	strcpy((char*)TxBuf, pucBuffer);

	DMA_Cmd(DMA1_Channel7, DISABLE);
	DMA1_Channel7->CNDTR = len;
	DMA_Cmd(DMA1_Channel7, ENABLE);
}

void AddBuffToTransmit(uint8_t* buf, uint16_t len)
{
	if (!interface_state) return;

	if (!Buf1Busy)
	{
		memcpy(&Buff1[Buf1Idx], buf, len);
		Buf1Idx += len;
		return;
	}
	if (!Buf2Busy)
	{
		memcpy(&Buff2[Buf2Idx], buf, len);
		Buf2Idx += len;
	}
}

void SendAvailableData()
{
	if (Buf1Idx > 0 && !Buf2Busy)
	{
		Buf1Busy = 1;
		memcpy(TxBuf, Buff1, Buf1Idx);
		DMA_Cmd(DMA1_Channel7, DISABLE);
		DMA1_Channel7->CNDTR = Buf1Idx;
		DMA_Cmd(DMA1_Channel7, ENABLE);
		Buf1Idx = 0;
		return;
	}
	if (Buf2Idx > 0 && !Buf1Busy)
	{
		Buf2Busy = 1;
		memcpy(TxBuf, Buff2, Buf2Idx);
		DMA_Cmd(DMA1_Channel7, DISABLE);
		DMA1_Channel7->CNDTR = Buf2Idx;
		DMA_Cmd(DMA1_Channel7, ENABLE);
		Buf2Idx = 0;
	}
}

uint8_t hexascii_to_halfbyte(uint8_t _ascii)
{
	if ((_ascii >= '0') && (_ascii <= '9'))
		return (_ascii - '0');
	if ((_ascii >= 'a') && (_ascii <= 'f'))
		return (_ascii - 'a' + 10);
	if ((_ascii >= 'A') && (_ascii <= 'F'))
		return (_ascii - 'A' + 10);
	return (0xFF);
}

uint8_t halfbyte_to_hexascii(uint8_t _halfbyte)
{
	_halfbyte &= 0x0F;
	if (_halfbyte >= 10)
		return ('A' + _halfbyte - 10);
	else
		return ('0' + _halfbyte);
}

void SetPrescalerAndInitCan(uint8_t presc)
{
	CAN_InitTypeDef CAN_InitStructure;
	GPIO_InitTypeDef GPIO_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
	RCC_APB2PeriphClockCmd(CAN1_Periph, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN1, ENABLE);

	GPIO_InitStructure.GPIO_Pin = CAN1_RX_SOURCE;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(CAN1_GPIO_PORT, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = CAN1_TX_SOURCE;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(CAN1_GPIO_PORT, &GPIO_InitStructure);

#ifdef CAN1_ReMap
	GPIO_PinRemapConfig(GPIO_Remap1_CAN1, ENABLE);
#endif

	CAN_DeInit(CAN1);
	CAN_StructInit(&CAN_InitStructure);

	CAN_InitStructure.CAN_TTCM = DISABLE;
	CAN_InitStructure.CAN_ABOM = DISABLE;
	CAN_InitStructure.CAN_AWUM = ENABLE;
	CAN_InitStructure.CAN_NART = DISABLE;
	CAN_InitStructure.CAN_RFLM = DISABLE;
	CAN_InitStructure.CAN_TXFP = DISABLE;
	CAN_InitStructure.CAN_Mode = CAN_Mode_Normal;
	CAN_InitStructure.CAN_SJW = CAN_SJW_1tq;
	CAN_InitStructure.CAN_BS1 = CAN_BS1_13tq;
	CAN_InitStructure.CAN_BS2 = CAN_BS2_2tq;
	CAN_InitStructure.CAN_Prescaler = presc;      //For 32 MHz APB1 Clock only!

	CAN_Init(CAN1, &CAN_InitStructure);

	CAN_FilterInitTypeDef CAN_FilterInitStructure;
	CAN_FilterInitStructure.CAN_FilterNumber = 1;
	CAN_FilterInitStructure.CAN_FilterMode = CAN_FilterMode_IdMask;
	CAN_FilterInitStructure.CAN_FilterScale = CAN_FilterScale_32bit;
	CAN_FilterInitStructure.CAN_FilterIdHigh = 0x0000;
	CAN_FilterInitStructure.CAN_FilterIdLow = 0x0000;
	CAN_FilterInitStructure.CAN_FilterMaskIdHigh = 0x0000;
	CAN_FilterInitStructure.CAN_FilterMaskIdLow = 0x0000;
	CAN_FilterInitStructure.CAN_FilterFIFOAssignment = CAN_FIFO0;
	CAN_FilterInitStructure.CAN_FilterActivation = ENABLE;
	CAN_FilterInit(&CAN_FilterInitStructure);

	CAN_ITConfig(CAN1, CAN_IT_FMP0, ENABLE);
	
	NVIC_InitStructure.NVIC_IRQChannel = USB_LP_CAN1_RX0_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
}

void SetupPeriph()
{
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	TIM_TimeBaseInitTypeDef TIMER_InitStructure;
	DMA_InitTypeDef DMA_InitStruct;

	///-------------------------------------------------
	///USART2
	///-------------------------------------------------

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);

	//USART2 Tx PA2
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	//USART2 Rx PA3
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	USART_InitStructure.USART_BaudRate = 115200;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl =
		USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	USART_Init(USART2, &USART_InitStructure);
	USART_Cmd(USART2, ENABLE);

	NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	USART_ITConfig(USART2, USART_IT_IDLE, ENABLE);
	USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);

	///-------------------------------------------------
	///TIM2 for timestamp
	///-------------------------------------------------
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);

	TIM_TimeBaseStructInit(&TIMER_InitStructure);
	TIMER_InitStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIMER_InitStructure.TIM_Prescaler = 32000;
	TIMER_InitStructure.TIM_Period = 60000;
	TIM_TimeBaseInit(TIM2, &TIMER_InitStructure);
	TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE);
	TIM_Cmd(TIM2, ENABLE);

	NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	///-------------------------------------------------
	///DMA Tx for USART2
	///-------------------------------------------------

	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);

	DMA_InitStruct.DMA_PeripheralBaseAddr = (uint32_t) &(USART2->DR);
	DMA_InitStruct.DMA_MemoryBaseAddr = (uint32_t)&TxBuf[0];
	DMA_InitStruct.DMA_DIR = DMA_DIR_PeripheralDST;
	DMA_InitStruct.DMA_BufferSize = BUFF_SIZE;
	DMA_InitStruct.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	DMA_InitStruct.DMA_MemoryInc = DMA_MemoryInc_Enable;
	DMA_InitStruct.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
	DMA_InitStruct.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
	DMA_InitStruct.DMA_Mode = DMA_Mode_Normal;
	DMA_InitStruct.DMA_Priority = DMA_Priority_Low;
	DMA_InitStruct.DMA_M2M = DMA_M2M_Disable;
	DMA_Init(DMA1_Channel7, &DMA_InitStruct);

	USART_DMACmd(USART2, USART_DMAReq_Tx, ENABLE);
	DMA_ITConfig(DMA1_Channel7, DMA_IT_TC, ENABLE);
	NVIC_EnableIRQ(DMA1_Channel7_IRQn);

	SetPrescalerAndInitCan(4);
}

void SetupClock()
{
	RCC_DeInit();
	RCC_HSEConfig(RCC_HSE_ON);
	while (RCC_GetFlagStatus(RCC_FLAG_HSERDY) == RESET)
	{
	}

	RCC_HCLKConfig(RCC_SYSCLK_Div1);      // HCLK   = SYSCLK  32MHz
	RCC_PCLK1Config(RCC_HCLK_Div1);      // PCLK1  = HCLK    32MHz
	RCC_PCLK2Config(RCC_HCLK_Div1);      // PCLK2  = HCLK	32MHz
	RCC_ADCCLKConfig(RCC_PCLK2_Div2);      //ADCCLK = PCLK2/2 16MHz

	RCC_PLLConfig(RCC_PLLSource_HSE_Div1, RCC_PLLMul_4);      // 8MHz * 4 = 32MHz
	RCC_PLLCmd(ENABLE);
	while (RCC_GetFlagStatus(RCC_FLAG_PLLRDY) == RESET)
	{
	}

	RCC_SYSCLKConfig(RCC_SYSCLKSource_PLLCLK);
	while (RCC_GetSYSCLKSource() != 0x08)
	{
	}
}

void SetInterfaceState(uint8_t state)
{
	interface_state = state;
}

int8_t USART_RX_Handler(uint8_t* Buf, uint16_t *Len)
{

	uint32_t num_bytes;
	uint8_t i;
	uint8_t tmp_byte;
	uint8_t tmp_buf[128];

	switch (Buf[0])
	{
	case 'V':
		num_bytes = sprintf((char*)tmp_buf, "V0101\r");
		break;

	case 'v':
		num_bytes = sprintf((char*)tmp_buf, "vSTM32\r");
		break;

	case 'N':
		num_bytes = sprintf((char*)tmp_buf, "NAFFF\r");
		break;

	case 'O':
		SetInterfaceState(1);
		TIM2->CNT = 0;
		break;

	case 'C':
		SetInterfaceState(0);
		break;
	case 'S':
		num_bytes = sprintf((char*)tmp_buf, "\r");
		if (*Len <= 3)
		{
			switch (Buf[1])
			{
			case '0':
				SetPrescalerAndInitCan(200);
				break;
			case '1':
				SetPrescalerAndInitCan(100);
				break;
			case '2':
				SetPrescalerAndInitCan(40);
				break;
			case '3':
				SetPrescalerAndInitCan(20);
				break;
			case '4':
				SetPrescalerAndInitCan(16);
				break;
			case '5':
				SetPrescalerAndInitCan(8);
				break;
			case '6':
				SetPrescalerAndInitCan(4);
				break;
			case '7':
				SetPrescalerAndInitCan(24);    //83.3
				break;
			case '8':
				SetPrescalerAndInitCan(2);
				break;
			default:
				num_bytes = sprintf((char*)tmp_buf, "\a");
				break;
			}
		}
		break;

	case 't':
		i = 1;
		TxMessage.StdId = hexascii_to_halfbyte(Buf[i++]);
		TxMessage.StdId = (TxMessage.StdId << 4) + hexascii_to_halfbyte(Buf[i++]);
		TxMessage.StdId = (TxMessage.StdId << 4) + hexascii_to_halfbyte(Buf[i++]);
		TxMessage.DLC = hexascii_to_halfbyte(Buf[i++]);
		tmp_byte = hexascii_to_halfbyte(Buf[i++]);
		tmp_byte = (tmp_byte << 4) + hexascii_to_halfbyte(Buf[i++]);
		TxMessage.Data[0] = tmp_byte;
		tmp_byte = hexascii_to_halfbyte(Buf[i++]);
		tmp_byte = (tmp_byte << 4) + hexascii_to_halfbyte(Buf[i++]);
		TxMessage.Data[1] = tmp_byte;
		tmp_byte = hexascii_to_halfbyte(Buf[i++]);
		tmp_byte = (tmp_byte << 4) + hexascii_to_halfbyte(Buf[i++]);
		TxMessage.Data[2] = tmp_byte;
		tmp_byte = hexascii_to_halfbyte(Buf[i++]);
		tmp_byte = (tmp_byte << 4) + hexascii_to_halfbyte(Buf[i++]);
		TxMessage.Data[3] = tmp_byte;
		tmp_byte = hexascii_to_halfbyte(Buf[i++]);
		tmp_byte = (tmp_byte << 4) + hexascii_to_halfbyte(Buf[i++]);
		TxMessage.Data[4] = tmp_byte;
		tmp_byte = hexascii_to_halfbyte(Buf[i++]);
		tmp_byte = (tmp_byte << 4) + hexascii_to_halfbyte(Buf[i++]);
		TxMessage.Data[5] = tmp_byte;
		tmp_byte = hexascii_to_halfbyte(Buf[i++]);
		tmp_byte = (tmp_byte << 4) + hexascii_to_halfbyte(Buf[i++]);
		TxMessage.Data[6] = tmp_byte;
		tmp_byte = hexascii_to_halfbyte(Buf[i++]);
		tmp_byte = (tmp_byte << 4) + hexascii_to_halfbyte(Buf[i++]);
		TxMessage.Data[7] = tmp_byte;

		CAN_Transmit(CAN1, &TxMessage);

		num_bytes = sprintf((char*)tmp_buf, "\r");
		break;

	default:
		num_bytes = sprintf((char*)tmp_buf, "\r");
		break;
	}
	if (interface_state)
	{		
		AddBuffToTransmit(tmp_buf, num_bytes);
	}
	else
	{
		USARTSendDMA((char*)tmp_buf);
	}
	return 0;
}

void CanRxHandler(CanRxMsg* RxMessage)
{
	if (RxMessage->IDE == CAN_Id_Standard)
	{
		uint8_t num_bytes;
		uint8_t buf[40];
		static uint32_t time;

		time = TIM2->CNT;
		num_bytes = 0;
		buf[num_bytes++] = 't';
		buf[num_bytes++] = halfbyte_to_hexascii((RxMessage->StdId) >> 8);
		buf[num_bytes++] = halfbyte_to_hexascii((RxMessage->StdId) >> 4);
		buf[num_bytes++] = halfbyte_to_hexascii((RxMessage->StdId));
		buf[num_bytes++] = halfbyte_to_hexascii((RxMessage->DLC));
		buf[num_bytes++] = halfbyte_to_hexascii((RxMessage->Data[0]) >> 4);
		buf[num_bytes++] = halfbyte_to_hexascii((RxMessage->Data[0]));
		if (RxMessage->DLC > 1)
		{			
			buf[num_bytes++] = halfbyte_to_hexascii((RxMessage->Data[1]) >> 4);
			buf[num_bytes++] = halfbyte_to_hexascii((RxMessage->Data[1]));
			if (RxMessage->DLC > 2)
			{
				buf[num_bytes++] = halfbyte_to_hexascii((RxMessage->Data[2]) >> 4);
				buf[num_bytes++] = halfbyte_to_hexascii((RxMessage->Data[2]));
				if (RxMessage->DLC > 3)
				{
					buf[num_bytes++] = halfbyte_to_hexascii((RxMessage->Data[3]) >> 4);
					buf[num_bytes++] = halfbyte_to_hexascii((RxMessage->Data[3]));
					if (RxMessage->DLC > 4)
					{
						buf[num_bytes++] = halfbyte_to_hexascii((RxMessage->Data[4]) >> 4);
						buf[num_bytes++] = halfbyte_to_hexascii((RxMessage->Data[4]));
						if (RxMessage->DLC > 5)
						{
							buf[num_bytes++] = halfbyte_to_hexascii((RxMessage->Data[5]) >> 4);
							buf[num_bytes++] = halfbyte_to_hexascii((RxMessage->Data[5]));
							if (RxMessage->DLC > 6)
							{
								buf[num_bytes++] = halfbyte_to_hexascii((RxMessage->Data[6]) >> 4);
								buf[num_bytes++] = halfbyte_to_hexascii((RxMessage->Data[6]));
								if (RxMessage->DLC > 7)
								{
									buf[num_bytes++] = halfbyte_to_hexascii((RxMessage->Data[7]) >> 4);
									buf[num_bytes++] = halfbyte_to_hexascii((RxMessage->Data[7]));
								}
							}
						}
					}
				}
			}			
		}	
		
		buf[num_bytes++] = halfbyte_to_hexascii((time) >> 12);
		buf[num_bytes++] = halfbyte_to_hexascii((time) >> 8);
		buf[num_bytes++] = halfbyte_to_hexascii((time) >> 4);
		buf[num_bytes++] = halfbyte_to_hexascii((time) >> 0);

		buf[num_bytes++] = '\r';
		AddBuffToTransmit(buf, num_bytes);
	}
}

uint16_t d_idx = 0;
void USART2_IRQHandler(void)
{
	uint16_t sr = USART2->SR;
	uint8_t dr = USART2->DR;
	if ((sr & USART_FLAG_IDLE) != (u16)RESET)
	{
		USART_RX_Handler(USART_RX_Buff, &d_idx);
		d_idx = 0;
	}

	if ((sr & USART_FLAG_RXNE) != (u16)RESET)
	{
		if (d_idx < USART_RX_SIZE)
		{
			USART_RX_Buff[d_idx++] = dr;
		}
	}
}
void TIM2_IRQHandler(void)
{
	if (TIM_GetITStatus(TIM2, TIM_IT_Update) != RESET)
	{
		TIM_ClearITPendingBit(TIM2, TIM_IT_Update);
		TIM2->CNT = 0;
	}
}
void DMA1_Channel7_IRQHandler(void)
{
	if (DMA_GetITStatus(DMA1_IT_TC7) != RESET)
	{
		DMA_ClearITPendingBit(DMA1_IT_TC7);
		DMA_Cmd(DMA1_Channel7, DISABLE);

		if (Buf1Busy)
		{
			Buf1Busy = 0;
			return;
		}
		if (Buf2Busy)
		{
			Buf2Busy = 0;
		}
	}
}

void USB_LP_CAN1_RX0_IRQHandler(void)
{
	CanRxMsg RxMessage;

	memset(&RxMessage.Data, 0x00, 8);

	if (CAN_GetITStatus(CAN1, CAN_IT_FMP0) != RESET)
	{
		CAN_Receive(CAN1, CAN_FIFO0, &RxMessage);
		CanRxHandler(&RxMessage);
	}
	if (CAN_GetITStatus(CAN1, CAN_IT_ERR) != RESET)
	{
		CAN_ClearITPendingBit(CAN1, CAN_IT_ERR);
	}
}

int main(void)
{
	
	SetupClock();
	SetupPeriph();

	while (1)
	{
		SendAvailableData();
	}
}
