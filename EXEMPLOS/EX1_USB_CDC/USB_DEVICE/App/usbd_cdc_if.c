/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : usbd_cdc_if.c
  * @version        : v2.0_Cube
  * @brief          : Usb device for Virtual Com Port.
    ******************************************************************************
  * Aluno:        Ruan Robert Bispo dos Santos                            2019.2
  * Orientadores: Lucas Molina e Elyson Adan Nunes Carvalho
  *
  * CODIGO DA PLACA DE INTERFACE DO MATLAB COM O STM32F103C8
  *
  * @Resumo: Código teste para USB_CDC
  *
  * @Obs: Função CDC_Receive_FS não pode ser chamada na main, ela é ativada por
  * 	  interrupção nesta configuração para a biblioteca HAL
  * 	  Função CDC_Transmit_FS pode ser chamada em outras funções para enviar
  *
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "usbd_cdc_if.h"

/* USER CODE BEGIN INCLUDE */

/* USER CODE END INCLUDE */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

/* USER CODE END PV */

/** @addtogroup STM32_USB_OTG_DEVICE_LIBRARY
  * @brief Usb device library.
  * @{
  */

/** @addtogroup USBD_CDC_IF
  * @{
  */

/** @defgroup USBD_CDC_IF_Private_TypesDefinitions USBD_CDC_IF_Private_TypesDefinitions
  * @brief Private types.
  * @{
  */

/* USER CODE BEGIN PRIVATE_TYPES */

/* USER CODE END PRIVATE_TYPES */

/**
  * @}
  */

/** @defgroup USBD_CDC_IF_Private_Defines USBD_CDC_IF_Private_Defines
  * @brief Private defines.
  * @{
  */

/* USER CODE BEGIN PRIVATE_DEFINES */
/* Define size for the receive and transmit buffer over CDC */
/* It's up to user to redefine and/or remove those define */
#define APP_RX_DATA_SIZE  1000
#define APP_TX_DATA_SIZE  1000
/* USER CODE END PRIVATE_DEFINES */

/**
  * @}
  */

/** @defgroup USBD_CDC_IF_Private_Macros USBD_CDC_IF_Private_Macros
  * @brief Private macros.
  * @{
  */

/* USER CODE BEGIN PRIVATE_MACRO */

/* USER CODE END PRIVATE_MACRO */

/**
  * @}
  */

/** @defgroup USBD_CDC_IF_Private_Variables USBD_CDC_IF_Private_Variables
  * @brief Private variables.
  * @{
  */
/* Create buffer for reception and transmission           */
/* It's up to user to redefine and/or remove those define */
/** Received data over USB are stored in this buffer      */
uint8_t UserRxBufferFS[APP_RX_DATA_SIZE];

/** Data to send over USB CDC are stored in this buffer   */
uint8_t UserTxBufferFS[APP_TX_DATA_SIZE];

/* USER CODE BEGIN PRIVATE_VARIABLES */

/* USER CODE END PRIVATE_VARIABLES */

/**
  * @}
  */

/** @defgroup USBD_CDC_IF_Exported_Variables USBD_CDC_IF_Exported_Variables
  * @brief Public variables.
  * @{
  */

extern USBD_HandleTypeDef hUsbDeviceFS;

/* USER CODE BEGIN EXPORTED_VARIABLES */

/* USER CODE END EXPORTED_VARIABLES */

/**
  * @}
  */

/** @defgroup USBD_CDC_IF_Private_FunctionPrototypes USBD_CDC_IF_Private_FunctionPrototypes
  * @brief Private functions declaration.
  * @{
  */

static int8_t CDC_Init_FS(void);
static int8_t CDC_DeInit_FS(void);
static int8_t CDC_Control_FS(uint8_t cmd, uint8_t* pbuf, uint16_t length);
static int8_t CDC_Receive_FS(uint8_t* pbuf, uint32_t *Len);

/* USER CODE BEGIN PRIVATE_FUNCTIONS_DECLARATION */

void concatenar(char s[], char t[]);

/* USER CODE END PRIVATE_FUNCTIONS_DECLARATION */

/**
  * @}
  */

USBD_CDC_ItfTypeDef USBD_Interface_fops_FS =
{
  CDC_Init_FS,
  CDC_DeInit_FS,
  CDC_Control_FS,
  CDC_Receive_FS
};

/* Private functions ---------------------------------------------------------*/
/**
  * @brief  Initializes the CDC media low layer over the FS USB IP
  * @retval USBD_OK if all operations are OK else USBD_FAIL
  */
static int8_t CDC_Init_FS(void)
{
  /* USER CODE BEGIN 3 */
  /* Set Application Buffers */
  USBD_CDC_SetTxBuffer(&hUsbDeviceFS, UserTxBufferFS, 0);
  USBD_CDC_SetRxBuffer(&hUsbDeviceFS, UserRxBufferFS);
  return (USBD_OK);
  /* USER CODE END 3 */
}

/**
  * @brief  DeInitializes the CDC media low layer
  * @retval USBD_OK if all operations are OK else USBD_FAIL
  */
static int8_t CDC_DeInit_FS(void)
{
  /* USER CODE BEGIN 4 */
  return (USBD_OK);
  /* USER CODE END 4 */
}

/**
  * @brief  Manage the CDC class requests
  * @param  cmd: Command code
  * @param  pbuf: Buffer containing command data (request parameters)
  * @param  length: Number of data to be sent (in bytes)
  * @retval Result of the operation: USBD_OK if all operations are OK else USBD_FAIL
  */
static int8_t CDC_Control_FS(uint8_t cmd, uint8_t* pbuf, uint16_t length)
{
  /* USER CODE BEGIN 5 */
  switch(cmd)
  {
    case CDC_SEND_ENCAPSULATED_COMMAND:

    break;

    case CDC_GET_ENCAPSULATED_RESPONSE:

    break;

    case CDC_SET_COMM_FEATURE:

    break;

    case CDC_GET_COMM_FEATURE:

    break;

    case CDC_CLEAR_COMM_FEATURE:

    break;

  /*******************************************************************************/
  /* Line Coding Structure                                                       */
  /*-----------------------------------------------------------------------------*/
  /* Offset | Field       | Size | Value  | Description                          */
  /* 0      | dwDTERate   |   4  | Number |Data terminal rate, in bits per second*/
  /* 4      | bCharFormat |   1  | Number | Stop bits                            */
  /*                                        0 - 1 Stop bit                       */
  /*                                        1 - 1.5 Stop bits                    */
  /*                                        2 - 2 Stop bits                      */
  /* 5      | bParityType |  1   | Number | Parity                               */
  /*                                        0 - None                             */
  /*                                        1 - Odd                              */
  /*                                        2 - Even                             */
  /*                                        3 - Mark                             */
  /*                                        4 - Space                            */
  /* 6      | bDataBits  |   1   | Number Data bits (5, 6, 7, 8 or 16).          */
  /*******************************************************************************/
    case CDC_SET_LINE_CODING:

    break;

    case CDC_GET_LINE_CODING:

    break;

    case CDC_SET_CONTROL_LINE_STATE:

    break;

    case CDC_SEND_BREAK:

    break;

  default:
    break;
  }

  return (USBD_OK);
  /* USER CODE END 5 */
}

/**
  * @brief  Data received over USB OUT endpoint are sent over CDC interface
  *         through this function.
  *
  *         @note
  *         This function will block any OUT packet reception on USB endpoint
  *         untill exiting this function. If you exit this function before transfer
  *         is complete on CDC interface (ie. using DMA controller) it will result
  *         in receiving more data while previous ones are still not sent.
  *
  * @param  Buf: Buffer of data to be received
  * @param  Len: Number of data received (in bytes)
  * @retval Result of the operation: USBD_OK if all operations are OK else USBD_FAIL
  */
static int8_t CDC_Receive_FS(uint8_t* Buf, uint32_t *Len)
{
  /* USER CODE BEGIN 6 */
  USBD_CDC_SetRxBuffer(&hUsbDeviceFS, &Buf[0]); //Padrão da função de recebimento
  USBD_CDC_ReceivePacket(&hUsbDeviceFS); 		//Padrão da função de recebimento


  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  // VARIÁVEIS UTILIZADAS
  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  	  static uint8_t cont = 0;    			   //Contador de 8 bits para testar comunicação (static garante que só é inicializado uma vez)
  	  float var_1 = -41;			   		   //Ex2: variável 1 de teste, atribuido qualquer valor
  	  int var_2 = 902;				   		   //Ex2: variável 2 de teste, atribuido qualquer valor
  	  char BufferAux[32];					   //Buffer auxiliar para concatenar
  	  	  	  	  	  	  	  	  	  	  	   //UserTxBufferFS[1000] é a variável padrão do buffer

  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  // EX1: ENVIA DADOS DO CONTADOR ASSIM QUE RECEBE SINAL DO PC (ECO)
  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	  /* Segure "Ctrl" e aperte "/" no código abaixo para descomentar o exemplo 1 (comente o exemplo 2) */

//	  cont = cont + 1;						   //Incrementa contador (conta de 0 a 255 e reinicia)
//  	  itoa(cont,UserTxBufferFS,10);            //Converte de inteiro para string - ex:589 = ["5","8","9"]
//	  uint16_t tam = strlen(UserTxBufferFS);   //Salva o tamanho da string
//	  UserTxBufferFS[tam] = '\n';			   //Adiciona quebra de linha
//	  UserTxBufferFS[tam+1] = '\0';			   //Adiciona o Terminator da string informando que a palavra acabou
//	  CDC_Transmit_FS(UserTxBufferFS, tam+1);  //Transmite dados de volta ao computador


  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  // EX2: CONCATENA E ENVIA DADOS DE DUAS VARIÁVEIS ASSIM QUE RECEBE SINAL DO PC - SEPARADO POR SIMBOLO "+"
  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	  /* Segure "Ctrl" e aperte "/" no código abaixo para descomentar o exemplo 2 (comente o exemplo 1) */

	  itoa(var_1,UserTxBufferFS,10);		   //Converte de inteiro para string - ex:589 = ["5","8","9"]
	  uint16_t tam = strlen(UserTxBufferFS);   //Salva o tamanho da string

	  UserTxBufferFS[tam] = '+';			   //Adiciona um simbolo base para servir como referencia para separar no PC/Código do MATLAB
	  UserTxBufferFS[tam+1] = '\0';			   //Adiciona o Terminator da string informando que a palavra acabou
	  itoa(var_2,BufferAux,10);				   //Converte de inteiro para string - ex:589 = ["5","8","9"]
	  concatenar(UserTxBufferFS,BufferAux);	   //Concatena os dados de BufferAux no final de UserTxBufferFS (função implementada)

	  tam = strlen(UserTxBufferFS);			   //Salva o novo tamanho da string
	  UserTxBufferFS[tam] = '\n';			   //Adiciona quebra de linha
	  UserTxBufferFS[tam+1] = '\0';			   //Adiciona o Terminator da string informando que a palavra acabou
	  CDC_Transmit_FS(UserTxBufferFS, tam+1);  //Transmite dados de volta ao computador


  return (USBD_OK);
  /* USER CODE END 6 */
}

/**
  * @brief  CDC_Transmit_FS
  *         Data to send over USB IN endpoint are sent over CDC interface
  *         through this function.
  *         @note
  *
  *
  * @param  Buf: Buffer of data to be sent
  * @param  Len: Number of data to be sent (in bytes)
  * @retval USBD_OK if all operations are OK else USBD_FAIL or USBD_BUSY
  */
uint8_t CDC_Transmit_FS(uint8_t* Buf, uint16_t Len)
{
  uint8_t result = USBD_OK;
  /* USER CODE BEGIN 7 */
  USBD_CDC_HandleTypeDef *hcdc = (USBD_CDC_HandleTypeDef*)hUsbDeviceFS.pClassData;
  if (hcdc->TxState != 0){
    return USBD_BUSY;
  }
  USBD_CDC_SetTxBuffer(&hUsbDeviceFS, Buf, Len);
  result = USBD_CDC_TransmitPacket(&hUsbDeviceFS);
  /* USER CODE END 7 */
  return result;
}

/* USER CODE BEGIN PRIVATE_FUNCTIONS_IMPLEMENTATION */
// Função que concatena 2 strings, assim a string "s" é aumentada e no seu
// final é adicionada a string "t". a saída é uma nova string "s".
void concatenar(char s[], char t[]){
    int i = 0, j = 0;

    while (s[i] != '\0') {
        i++;
    }

    while ((s[i++] = t[j++]) != '\0');
}
/* USER CODE END PRIVATE_FUNCTIONS_IMPLEMENTATION */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
