/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "crc.h"
#include "dma.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"
#define ARM_MATH_CM4
#include "arm_math.h"
#include "network.h"
#include "network_config.h"
#include "network_data.h"
#include "network_data_params.h"
#include "ai_datatypes_defines.h"
#include "ai_platform.h"

/* Include arm_math.h mathematic functions */

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define DIM 8000
#define SAMPLES  512
#define FFT_SIZE (SAMPLES/2)
#define Fs  20000
#define passo_f  (Fs/2)/(FFT_SIZE);
#define ARM_MATH_CM4

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

float32_t fft_in_buf_realX [SAMPLES];
float32_t fft_in_buf_realY [SAMPLES];
float32_t fft_in_buf_realZ [SAMPLES];
float32_t tempX[FFT_SIZE];
float32_t tempY[FFT_SIZE];
float32_t tempZ[FFT_SIZE];
float32_t fft_out_buf_realX[FFT_SIZE];
float32_t fft_out_buf_realY[FFT_SIZE];
float32_t fft_out_buf_realZ[FFT_SIZE];
arm_rfft_fast_instance_f32 fft_handler;
arm_cfft_radix4_instance_f32 fft_handler_cplx;
float32_t massimo;
float32_t valMassimiX[10];
float32_t indX[10];
float32_t valMassimiY[10];
float32_t indY[10];
float32_t valMassimiZ[10];
float32_t indZ[10];
int indmax;
char  buffer_out[100];
uint8_t spiSndX[1];			//Buffer di trasmissione X MSB
uint8_t spiSndY[1];			//Buffer di trasmissione Y MSB
uint8_t spiSndZ[1];			//Buffer di trasmissione Z MSB
uint8_t spiSndXLSB[1];		//Buffer di trasmissione X LSB
uint8_t spiSndYLSB[1];		//Buffer di trasmissione Y LSB
uint8_t spiSndZLSB[1];		//Buffer di trasmissione Z LSB
uint8_t spiRcv[2];			//Buffer di ricezione LSB
uint8_t spiRcvMSB[2];		//Buffer di ricezione MSB
uint8_t spiSnd[2];			//Buffer invio comandi SPI
uint16_t buffer[50]={0};	//Buffer usato per UART5
uint16_t buffer2[300];
uint16_t buffer3[300];
uint8_t a;
float Vettx[DIM];
float Vetty[DIM];
float Vettz[DIM];
volatile uint8_t flag_elapsed = 0;
ai_buffer *ai_input;
ai_buffer *ai_output;

void uprintf(char* str){
	HAL_UART_Transmit(&huart5, (uint8_t*)str, strlen(str), 100);
}


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

		spiSndX[0]=0x29|0x80;
	    spiSndY[0]=0x2B|0x80;
		spiSndZ[0]=0x2D|0x80;
		spiSndXLSB[0]=0x28|0x80;
		spiSndYLSB[0]=0x2A|0x80;
		spiSndZLSB[0]=0x2C|0x80;
		uint16_t x[DIM];
		uint16_t y[DIM];
		uint16_t z[DIM];
		int16_t k=0;
		float acc;
		float typ=0.488;
		int i=0;

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_SPI1_Init();
  MX_USART2_UART_Init();
  MX_UART5_Init();
  MX_TIM2_Init();
  MX_CRC_Init();
  /* USER CODE BEGIN 2 */

  HAL_UART_MspInit(&huart5);
  HAL_GPIO_WritePin(GPIOG, GPIO_PIN_2, GPIO_PIN_RESET);
  spiSnd[0]=0x10;
  spiSnd[1]=0xA4;
  HAL_SPI_Transmit(&hspi1, spiSnd, 2, 100);
  HAL_GPIO_WritePin(GPIOG, GPIO_PIN_2, GPIO_PIN_SET);
  HAL_TIM_Base_Start(&htim2);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (i<DIM)
   {
 	 if((flag_elapsed=1))											//La funzione di interrupt abilita il flag per ogni n conteggi selezionati
 	 {
 		 flag_elapsed=0;										//flag=0 in modo che non rientro ma aspetto che sia l'interrupt ad alzarlo

 		//Asse X
 		HAL_GPIO_WritePin(GPIOG, GPIO_PIN_2, GPIO_PIN_RESET);	//Abilito SPI1
 		HAL_SPI_Transmit(&hspi1, spiSndX, 1, 100);				//Trasmetto Indirizzo dove leggere istruzione
 		HAL_SPI_Receive(&hspi1, &spiRcv[0], 1, 100);			//Ricevo Risultato di misura MSB (8bit)
 		HAL_GPIO_WritePin(GPIOG, GPIO_PIN_2, GPIO_PIN_SET);		//Chiudo la comunicazione SPI1
 		HAL_GPIO_WritePin(GPIOG, GPIO_PIN_2, GPIO_PIN_RESET);	//Abilito la comunicazione SPI1 (NON POSSO FARE PIU OPERAZIONI CON UN'UNICA ATTIVAZIONE)
 		HAL_SPI_Transmit(&hspi1, spiSndXLSB, 1, 100);			//Trasmetto Indirizzo dove leggere istruzione
 		HAL_SPI_Receive(&hspi1, &spiRcv[1], 1, 100);			//Ricevo Risultato di misura LSB (8bit)
 		x[i]=((int16_t)spiRcv[0] << 8) | spiRcv[1];				//Unisco MSB e LSB attraverso un'operazione di shift su 16bit
 		HAL_GPIO_WritePin(GPIOG, GPIO_PIN_2, GPIO_PIN_SET);		//Chiudo la comunicazione SPI1
 		k=(int16_t)x[i];										//Casting da uint a int (abilitazione del segno, complemento)
 		acc=(k*typ)/1000;										//Da datasheet, il dato letto, lo converto in scala g attraverso questo calcolo
 		Vettx[i]=acc;											//Aggiungo la misura al vettore delle letture
 		//HAL_UART_Transmit(&huart5,(uint8_t*)"\n \r", 3, HAL_MAX_DELAY);			//Vado da capo


 		spiRcv[0]=0;
 		spiRcv[1]=0;
 		//Asse Y
 		HAL_GPIO_WritePin(GPIOG, GPIO_PIN_2, GPIO_PIN_RESET);	//Abilito SPI1
 		HAL_SPI_Transmit(&hspi1, spiSndY, 1, 100);				//Trasmetto Indirizzo dove leggere istruzione
 		HAL_SPI_Receive(&hspi1, &spiRcv[0], 1, 100);			//Ricevo Risultato di misura MSB (8bit)
 		HAL_GPIO_WritePin(GPIOG, GPIO_PIN_2, GPIO_PIN_SET);		//Chiudo la comunicazione SPI1
 		HAL_GPIO_WritePin(GPIOG, GPIO_PIN_2, GPIO_PIN_RESET);	//Abilito la comunicazione SPI1 (NON POSSO FARE PIU OPERAZIONI CON UN'UNICA ATTIVAZIONE)
 		HAL_SPI_Transmit(&hspi1, spiSndYLSB, 1, 100);			//Trasmetto Indirizzo dove leggere istruzione
 		HAL_SPI_Receive(&hspi1, &spiRcv[1], 1, 100);			//Ricevo Risultato di misura LSB (8bit)
 		y[i]=((int16_t)spiRcv[0] << 8) | spiRcv[1];				//Unisco MSB e LSB attraverso un'operazione di shift su 16bit
 		HAL_GPIO_WritePin(GPIOG, GPIO_PIN_2, GPIO_PIN_SET);		//Chiudo la comunicazione SPI1
 		k=(int16_t)y[i];										//Casting da uint a int (abilitazione del segno, complemento)
 		acc=(k*typ)/1000;										//Da datasheet, il dato letto, lo converto in scala g attraverso questo calcolo
 		Vetty[i]=acc;											//Aggiungo la misura al vettore delle letture
 		//HAL_UART_Transmit(&huart5,(uint8_t*)"\n \r", 3, HAL_MAX_DELAY);			//Vado da capo

 		spiRcv[0]=0;
 		spiRcv[1]=0;
 		//Asse Z												//Si ripetono gli stessi commenti
 		HAL_GPIO_WritePin(GPIOG, GPIO_PIN_2, GPIO_PIN_RESET);
 		HAL_SPI_Transmit(&hspi1, spiSndZ, 1, 100);
 		HAL_SPI_Receive(&hspi1, &spiRcv[0], 1, 100);
 		HAL_GPIO_WritePin(GPIOG, GPIO_PIN_2, GPIO_PIN_SET);
 		HAL_GPIO_WritePin(GPIOG, GPIO_PIN_2, GPIO_PIN_RESET);
 		HAL_SPI_Transmit(&hspi1, spiSndZLSB, 1, 100);
 		HAL_SPI_Receive(&hspi1, &spiRcv[1], 1, 100);
 		z[i]=((int16_t)spiRcv[0] << 8) | spiRcv[1];
 		HAL_GPIO_WritePin(GPIOG, GPIO_PIN_2, GPIO_PIN_SET);
 		k=(int16_t)z[i];
 		acc=(k*typ)/1000;
 		Vettz[i]=acc;
 		//HAL_UART_Transmit(&huart5,(uint8_t*)"\n \r", 3, HAL_MAX_DELAY);			//Vado da capo

 	    i=i+1;															//Incremento la i per spostare le misure nel buffer
 	  }
   }

  /* USER CODE END WHILE */


  /* USER CODE BEGIN 3 */

  HAL_TIM_Base_Stop(&htim2);											//Completate le n=DIM misure, fermo il timer
     HAL_UART_Transmit(&huart5,(uint8_t*)"\n \r", 3, HAL_MAX_DELAY);		//Trasmetto uno spazio per ordine

     //Scrivo le misure relative all'asse X
     /*uprintf("Misure Asse X \n \r");
     for(int j=0; j<DIM; j++){
  	   sprintf(buffer2,"%f \n \r",Vettx[j]); // @suppress("Float formatting support")? No, ho aggiunto manualmente la funzione nel Linker
  	   uprintf(buffer2);
     }

     //Scrivo le misure relative all'asse Y
     uprintf("Misure Asse Y \n \r");
     for(int j=0; j<DIM; j++){
  	  sprintf(buffer2,"%f \n \r",Vetty[j	]); // @suppress("Float formatting support")? No, ho aggiunto manualmente la funzione nel Linker
  	  uprintf(buffer2);
     }

     //Scrivo le misure relative all'asse Z
     uprintf("Misure Asse Z \n \r");
     for(int j=0; j<DIM; j++){
  	  sprintf(buffer2,"%f \n \r",Vettz[j]); // @suppress("Float formatting support")? No, ho aggiunto manualmente la funzione nel Linker
  	  uprintf(buffer2);
     }*/

     //FFT reale



        	  /*Initialize the RFFT*/
        	  arm_rfft_fast_init_f32(&fft_handler, FFT_SIZE);
        	  /* Initialize the CFFT/CIFFT module, intFlag = 0, doBitReverse = 1 */
        	  arm_cfft_radix4_init_f32(&fft_handler_cplx, FFT_SIZE, 0, 1);

        	  //Asse X

        	  for (int j = 0; j < FFT_SIZE; j++) {
        		  		fft_in_buf_realX[j]=((float32_t)((float32_t)Vettx[j]));
        		  		}
        	  arm_rfft_fast_f32(&fft_handler, fft_in_buf_realX, fft_out_buf_realX, 0);
        	  for (int j = 0; j < SAMPLES; j += 2) {
        		  		  			fft_in_buf_realX[j]=(float32_t)((float32_t)Vettx[j/2]);
        		  		  		    //parte immaginaria
        		  		  			fft_in_buf_realX[(j + 1)] = 0;
          	 	 	 	 	 }
          arm_cfft_f32(&fft_handler, fft_in_buf_realX, 0, 1);
          arm_cmplx_mag_f32(fft_in_buf_realX, fft_out_buf_realX, FFT_SIZE);


          //Stampa sulla com
          for(int j = 0; j < FFT_SIZE;j++){

         	  		 //sprintf(buffer3,"%f \n \r",fft_out_buf_realX[j]);
         	  		 //uprintfX(buffer3);
          }

          //Asse Y


          for (int j = 0; j < FFT_SIZE; j++) {
             		  		fft_in_buf_realY[j]=((float32_t)((float32_t)Vetty[j]));
             		  		}
             	  arm_rfft_fast_f32(&fft_handler, fft_in_buf_realX, fft_out_buf_realY, 0);
             	  for (int j = 0; j < SAMPLES; j += 2) {
             		  		  			fft_in_buf_realY[j]=(float32_t)((float32_t)Vetty[j/2]);
             		  		  		    //parte immaginaria
             		  		  			fft_in_buf_realY[(j + 1)] = 0;
               	 	 	 	 	 }
               arm_cfft_f32(&fft_handler, fft_in_buf_realY, 0, 1);
               arm_cmplx_mag_f32(fft_in_buf_realY, fft_out_buf_realY, FFT_SIZE);

           //Asse Z

            for (int j = 0; j < FFT_SIZE; j++) {
                  		  		fft_in_buf_realZ[j]=((float32_t)((float32_t)Vettz[j]));
                  		  		}
                  	  arm_rfft_fast_f32(&fft_handler, fft_in_buf_realZ, fft_out_buf_realZ, 0);
                  	  for (int j = 0; j < SAMPLES; j += 2) {
                  		  		  			fft_in_buf_realZ[j]=(float32_t)((float32_t)Vettz[j/2]);
                  		  		  		    //parte immaginaria
                  		  		  			fft_in_buf_realZ[(j + 1)] = 0;
                    	 	 	 	 	 }
                    arm_cfft_f32(&fft_handler, fft_in_buf_realZ, 0, 1);
                    arm_cmplx_mag_f32(fft_in_buf_realZ, fft_out_buf_realZ, FFT_SIZE);



          //Prendo le 10 armoniche a magnitude maggiore di X

          for(int j = 0; j<FFT_SIZE;j++){
         	 tempX[j] = (float32_t)fft_out_buf_realX[j];
          }

          for(int j = 0; j<5;j++){

         	 arm_max_f32(tempX, FFT_SIZE, &massimo, &indmax);
         	 valMassimiX[j] = massimo;
         	 indX[j] = indmax*passo_f;
         	 tempX[indmax] = 0;
         	 sprintf(buffer3,"%f \n \r",valMassimiX[j]);
         	 uprintf(buffer3);
         	 sprintf(buffer2,"%f \n \r",indX[j]);
         	 uprintf(buffer2);
          }

          //Prendo le 10 armoniche a magnitude maggiore di Y

          for(int j = 0; j<FFT_SIZE;j++){
              	 tempY[j] = (float32_t)fft_out_buf_realY[j];
               }

               for(int j = 0; j<10;j++){

              	 arm_max_f32(tempY, FFT_SIZE, &massimo, &indmax);
              	 valMassimiY[j] = massimo;
              	 indY[j] = indmax*passo_f;
              	 tempY[indmax] = 0;
              	 sprintf(buffer3,"%f \n \r",valMassimiY[j]);
              	 //uprintf(buffer3);
              	 sprintf(buffer2,"%f \n \r",indY[j]);
              	 //uprintf(buffer2);
               }

            //Prendo le 10 armoniche a magnitude maggiore di Z

            for(int j = 0; j<FFT_SIZE;j++){
                   	 tempZ[j] = (float32_t)fft_out_buf_realZ[j];
                    }

                    for(int j = 0; j<10;j++){

                   	 arm_max_f32(tempZ, FFT_SIZE, &massimo, &indmax);
                   	 valMassimiZ[j] = massimo;
                   	 indZ[j] = indmax*passo_f;
                   	 tempZ[indmax] = 0;
                   	 sprintf(buffer3,"%f \n \r",valMassimiZ[j]);
                   	 //uprintf(buffer3);
                   	 sprintf(buffer2,"%f \n \r",indZ[j]);
                   	 //uprintf(buffer2);
                    }


  //Definizione parametri per il NN

  AI_ALIGNED(32) ai_u32 activations[AI_NETWORK_DATA_ACTIVATIONS_SIZE];

  //Ricavo i pesi e i bias dal modello

  ai_network_params ai_params = {
		  AI_NETWORK_DATA_WEIGHTS(ai_network_data_weights_get()),
		  AI_NETWORK_DATA_ACTIVATIONS(activations)
  };

  //Creo i vettori buffer per raccogliere gli input e gli output del mio NN

  AI_ALIGNED(32) ai_i8 input_data[AI_NETWORK_IN_1_SIZE_BYTES];
  AI_ALIGNED(32) ai_i8 output_data[AI_NETWORK_OUT_1_SIZE_BYTES];

  //Puntatore al modello

  ai_handle  network = AI_HANDLE_NULL;


  //Creo un'istanza del NN

  ai_error ai_err;
  ai_err = ai_network_create(&network,AI_NETWORK_DATA_CONFIG);

  //Verifico che il NN sia stato istanziato ed inizializzato correttamente

  if(ai_err.type != AI_ERROR_NONE){
	  sprintf(buffer2,"Errore creazione NN");
  	  uprintf(buffer2);
  }

  //Verifico che il NN sia inizializzato correttamente

   if(!ai_network_init(network, &ai_params)){

	   sprintf(buffer2,"Errore inizializzazione NN");
	   uprintf(buffer2);
   }

   //

   ai_input = ai_network_inputs_get(network, NULL);
   ai_output = ai_network_outputs_get(network, NULL);

   ai_input->data = AI_HANDLE_PTR(input_data);
   ai_output->data = AI_HANDLE_PTR(output_data);

   //Riempio il buffer di input

   for(int j=0;j<AI_NETWORK_IN_1_SIZE;j++){

	  ((ai_float*)input_data)[j] = (ai_float)5;
	  sprintf(buffer2,"%f \n \r",((ai_float*)input_data)[j]);
	  uprintf(buffer2);
   }

   ai_i32 inference = ai_network_run(network,ai_input,ai_output);

   if(inference != 1){

   	  	  sprintf(buffer2,"Errore nell'eseguire l'inferenza");
   	  	  uprintf(buffer2);
   }

   sprintf(buffer2,"%f \n \r",((ai_float*)output_data)[0]);
   uprintf(buffer2);




  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 12;
  RCC_OscInitStruct.PLL.PLLN = 180;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Activate the Over-Drive mode
  */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
