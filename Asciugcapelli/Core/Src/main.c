/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
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


#define ARM_MATH_CM4
#include "main.h"
#include "crc.h"
#include "dma.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"
#include "arm_math.h"
#include "network.h"
#include "network_data.h"
#include "ai_datatypes_defines.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define DIM 5000			//Aumento la scalabilità del codice utilizzando questa macro
#define N 5


/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

uint16_t buffer[50]={0};	//Buffer usato per UART5
uint16_t buffer2[300];		//Buffer usato per UART5


//Ho creato una funzione che mi semplifica la scrittura sulla COM di messaggi
void uprintf(char* str){
	HAL_UART_Transmit(&huart5, (uint8_t*)str, strlen(str), 100);
}

//Funzioni di stampa
void printAmpFreq(float32_t a[N],float32_t b[N],float32_t c[N],float32_t d[N],float32_t e[N],float32_t f[N]){



	for(int j=0;j<N;j++){
			   for(int k=0;k<6;k++){
			   		   if(k==0){
			   		   sprintf(buffer2,"%f",a[j]);
			   		   uprintf(buffer2);
			   		   }
			   		   else if(k==1){
			   		   sprintf(buffer2,"%f",b[j]);
			   		   uprintf(buffer2);
			   		   }
			   		   else if(k==2){
			   		   sprintf(buffer2,"%f",c[j]);
			   		   uprintf(buffer2);
			   		   }
			   		   else if(k==3){
	     	           sprintf(buffer2,"%f",d[j]);
	     	           uprintf(buffer2);
			   		   }
			   		   else if(k==4){
			   		   sprintf(buffer2,"%f",e[j]);
			   		   uprintf(buffer2);
			   		   }
			   		   else if(k==5){
			   		   sprintf(buffer2,"%f",f[j]);
			   		   uprintf(buffer2);
			   		   }
			   		   if(k!=5){
			   		   sprintf(buffer2,"\t");
			   		   uprintf(buffer2);
			   		   }
			   	   }
			   	   sprintf(buffer2,"\n \r");
			   	   uprintf(buffer2);
			      }

}
void printAcc(float a[DIM],float b[DIM],float c[DIM]){

	for(int j=0;j<DIM;j++){
		   for(int k=0;k<3;k++){
			   if(k==0){
			   sprintf(buffer2,"%f",a[j]);
			   uprintf(buffer2);
			   }
			   else if(k==1){
			   sprintf(buffer2,"%f",b[j]);
			   uprintf(buffer2);
			   }
			   else if(k==2){
			   sprintf(buffer2,"%f",c[j]);
			   uprintf(buffer2);
			   }
			   sprintf(buffer2,"\t");
			   uprintf(buffer2);
		   }
		   sprintf(buffer2,"\n \r");
		   uprintf(buffer2);
	   }
	   }



const int SAMPLES = 8192;
const int FFT_SIZE = 4096;
const float Fs = 5000;
const float passo_f = (Fs/2)/(FFT_SIZE/2);
float32_t fft_in_buf[8192]; //Buffer di input della FFT
float32_t fft_out_buf[4096]; //Vettore di output della FFT
float32_t temp[2048];  //Vettori e variabili usate per trovare i valori massimi
float32_t valMassimiX[N];
float32_t valMassimiY[N];
float32_t valMassimiZ[N];
float32_t indX[N];
float32_t indY[N];
float32_t indZ[N];
float32_t massimo;
float32_t matriceInput[N][6];
int indmax;
uint8_t spiSndX[1];			//Buffer di trasmissione X MSB
uint8_t spiSndY[1];			//Buffer di trasmissione Y MSB
uint8_t spiSndZ[1];			//Buffer di trasmissione Z MSB
uint8_t spiSndXLSB[1];		//Buffer di trasmissione X LSB
uint8_t spiSndYLSB[1];		//Buffer di trasmissione Y LSB
uint8_t spiSndZLSB[1];		//Buffer di trasmissione Z LSB
uint8_t spiRcv[2];			//Buffer di ricezione LSB
uint8_t spiRcvMSB[2];		//Buffer di ricezione MSB
uint8_t spiSnd[2];			//Buffer invio comandi SPI
uint8_t a;					//Variabile di appoggio usata per snprintf (si può eliminare se non la si usa)
float Vettx[DIM];			//Vettore float (con virgola) della misura Accelerazione Asse X
float Vetty[DIM];			//Vettore float (con virgola) della misura Accelerazione Asse Y
float Vettz[DIM];			//Vettore float (con virgola) della misura Accelerazione Asse Z
volatile uint8_t flag_elapsed = 0;	//Flag proveniente dall'interrupt function del timer (si alza dopo i 2250-1 conteggi settati)

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
	spiSndX[0]=0x29|0x80;			//Indirizzo Lettura X MSB
    spiSndY[0]=0x2B|0x80;			//Indirizzo Lettura Y MSB
	spiSndZ[0]=0x2D|0x80;			//Indirizzo Lettura Z MSB
	spiSndXLSB[0]=0x28|0x80;		//Indirizzo Lettura X LSB
	spiSndYLSB[0]=0x2A|0x80;		//Indirizzo Lettura Y LSB
	spiSndZLSB[0]=0x2C|0x80;		//Indirizzo Lettura Z LSB
	uint16_t x[DIM];				//Vettore intero 16bit misura Accelerazione Asse X
	uint16_t y[DIM];				//Vettore intero 16bit misura Accelerazione Asse Y
	uint16_t z[DIM];				//Vettore intero 16bit misura Accelerazione Asse Z
	int16_t k=0;					//Variabile usata per il casting
	float acc;						//Accelerazione istantanea ad asse
	float typ=0.244;
	int i=0;						//Variabile di appoggio while
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
    HAL_UART_MspInit(&huart5);			//Inizializzo UART5

    //Messaggio di Boot COM
    //uprintf("/******  Misuratore Assiale di Accelerazione  ******/ \n \r");
    //uprintf("/****   Programma Realizzato da Salvatore Granata    ****/ \n \r");
    //uprintf("/****   Ingegneria dell'Automazione   ****/ \n \r");
    //uprintf("Sensore Avviato e pronto per misure \n \r");


   /*1. Abilito lo slave portando la linea CS/SS Bassa*/
    HAL_GPIO_WritePin(GPIOG, GPIO_PIN_2, GPIO_PIN_RESET);	   /*1. Abilita lo Slave settando CS Basso lo stato dell'IIS3DWB */

   /*2. Trasmetto indirizzo registro e dati */
    spiSnd[0]=0x10;												//Indirizzo Registro CTRL1_XL (Abilita l'accelerometro)
    spiSnd[1]=0xAC;												//Scrivo 101 all'inizio del registro per abilitarlo e poi tutti zero (accelerometro scala +-2g)
    HAL_SPI_Transmit(&hspi1, spiSnd, 2, 100);					//Invio messaggio allo slave per effettuare l'abilitazione abilitazione

   /*3. Disabilito lo slave portando la linea CS/SS Alta*/		//Inviata l'istruzione di abilitazione dell'accelerometro,
    HAL_GPIO_WritePin(GPIOG, GPIO_PIN_2, GPIO_PIN_SET);			//Chiudo la comunicazione

    HAL_TIM_Base_Start(&htim2);									//Faccio partire il timer
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */


  while (i<DIM)
  {
	 if((flag_elapsed)=1)											//La funzione di interrupt abilita il flag per ogni n conteggi selezionati
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
   i=0;

   //Stampo le misure di accelerazione
   //printAcc(Vettx,Vetty,Vettz);

   //Eseguo la FFT

   arm_cfft_radix2_instance_f32 S;
   /* Inizializzo il modulo CFFT/CIFFT, intFlag = 0, doBitReverse = 1 */
   arm_cfft_radix2_init_f32(&S, FFT_SIZE, 0, 1);

   //Asse X
   for(int j=0;j<SAMPLES;j+=2){
	   fft_in_buf[j] = ((float32_t)((float32_t)Vettx[j/2]));
	   fft_in_buf[j+1] = 0;
   }
   /*Processo i dati attraverso il modulo CFFT/CIFFT */
   arm_cfft_radix2_f32(&S, fft_in_buf);
   /*Processo i dati attraverso il modulo Complex Magnitude*/
   arm_cmplx_mag_f32(fft_in_buf, fft_out_buf, FFT_SIZE);

   //Prendo le N componenti frequenziali ad ampiezza maggiore dello spettro di X

      for(int j = 0; j<((FFT_SIZE/2));j++){
          temp[j] = (float32_t)fft_out_buf[j+1];
        }
      for(int j=0; j<N;j++){
   	   arm_max_f32(temp, FFT_SIZE/2, &massimo, &indmax);
   	   valMassimiX[j] = 2*massimo;
   	   indX[j] = (float)(indmax)*passo_f;
   	   temp[indmax] = 0;
     }


   //Asse Y
   for(int j=0;j<SAMPLES;j+=2){
  	   fft_in_buf[j] = ((float32_t)((float32_t)Vetty[j/2]));
  	   fft_in_buf[j+1] = 0;
     }
   /*Processo i dati attraverso il modulo CFFT/CIFFT */
   arm_cfft_radix2_f32(&S, fft_in_buf);
   /* Processo i dati attraverso il modulo Complex Magnitude*/
   arm_cmplx_mag_f32(fft_in_buf, fft_out_buf, FFT_SIZE);

   //Prendo le N componenti frequenziali ad ampiezza maggiore dello spettro di Y

      for(int j = 0; j<((FFT_SIZE/2));j++){
          temp[j] = (float32_t)fft_out_buf[j+1];
        }
         for(int j=0; j<N;j++){
      	   arm_max_f32(temp, FFT_SIZE/2, &massimo, &indmax);
      	   valMassimiY[j] = 2*massimo;
      	   indY[j] = (float)(indmax*passo_f);
      	   temp[indmax] = 0;
        }


   //Asse Z
   for(int j=0;j<SAMPLES;j+=2){
       fft_in_buf[j] = ((float32_t)((float32_t)Vettz[j/2]));
       fft_in_buf[j+1] = 0;
     }
   /*Processo i dati attraverso il modulo CFFT/CIFFT */
   arm_cfft_radix2_f32(&S, fft_in_buf);
   /* Processo i dati attraverso il modulo Complex Magnitude*/
   arm_cmplx_mag_f32(fft_in_buf, fft_out_buf, FFT_SIZE);

   //Prendo le N componenti frequenziali ad ampiezza maggiore dello spettro di Z

   for(int j = 0; j<((FFT_SIZE/2));j++){
       temp[j] = (float32_t)fft_out_buf[j+1];
     }
      for(int j=0; j<N;j++){
   	   arm_max_f32(temp, FFT_SIZE/2, &massimo, &indmax);
   	   valMassimiZ[j] = 2*massimo;
   	   indZ[j] = (float)(indmax*passo_f);
   	   temp[indmax] = 0;
     }

   //Stampo le coppie ampiezza-frequenza sui 3 assi

    printAmpFreq(valMassimiX,indX,valMassimiY,indY,valMassimiZ,indZ);

    //Preparo la matrice con gli input da dare al mio NN(riempio la matrice per colonne)

    //Preparo la matrice con gli input da dare al mio NN(riempio la matrice per colonne)

      for(int j=0;j<AI_NETWORK_IN_1_SIZE;j++){

    	  for(int k=0;k<N;k++){

    		  if(j==0)
    			  matriceInput[k][j] = (float32_t)valMassimiX[k];
    		  if(j==1)
    			  matriceInput[k][j] = (float32_t)indX[k];
    		  if(j==2)
    			  matriceInput[k][j] = (float32_t)valMassimiY[k];
    		  if(j==3)
    			  matriceInput[k][j] = (float32_t)indY[k];
    		  if(j==4)
    			  matriceInput[k][j] = (float32_t)valMassimiZ[k];
    		  if(j==5)
    			  matriceInput[k][j] = (float32_t)indZ[k];
    	  }
      }

      // Chunk of memory used to hold intermediate values for neural network

        	AI_ALIGNED(4) ai_u8 activations[AI_NETWORK_DATA_ACTIVATIONS_SIZE];
          ai_i32 inferenza;

        // Buffer per memorizzare i tensori di input e di output

        	AI_ALIGNED(4) ai_i8 in_data[AI_NETWORK_IN_1_SIZE_BYTES];
          AI_ALIGNED(4) ai_i8 out_data[AI_NETWORK_OUT_1_SIZE_BYTES];

          //Puntatore al modello

          ai_handle network = AI_HANDLE_NULL;

          //Inizializzo le struct che conterranno i puntatori ai dati

              ai_buffer ai_input[AI_NETWORK_IN_NUM] = AI_NETWORK_IN;
              ai_buffer ai_output[AI_NETWORK_OUT_NUM] = AI_NETWORK_OUT;

              //Ricavo i pesi e i bias del modello

                 ai_network_params ai_params = {
                 AI_NETWORK_DATA_WEIGHTS(ai_network_data_weights_get()),
                 AI_NETWORK_DATA_ACTIVATIONS(activations)
               };

               //Setto le struct

                 ai_input[0].n_batches = 1;
                 ai_input[0].data = AI_HANDLE_PTR(in_data);
                 ai_output[0].n_batches = 1;
                 ai_output[0].data = AI_HANDLE_PTR(out_data);

                 //Creo un'istanza del NN

                      ai_error ai_err;
                      ai_err = ai_network_create(&network, AI_NETWORK_DATA_CONFIG);
                      if (ai_err.type != AI_ERROR_NONE)
                      {
                     	 sprintf(buffer2,"Errore");
                     	 uprintf(buffer2);

                      }

                      //Verifico che il NN sia inizializzato correttamente

                      if (!ai_network_init(network, &ai_params))
                      {
                     	 sprintf(buffer2,"Errore");
                     	 uprintf(buffer2);

                      }
                      //Riempio il buffer di input

                          	 for (uint32_t i = 0; i < AI_NETWORK_IN_1_SIZE; i++)
                              {
                                ((ai_float *)in_data)[i] = (ai_float)matriceInput[2][i];
                                //sprintf(buffer2,"%f \n \r",((ai_float*)in_data)[i]);
                                //uprintf(buffer2);
                              }

                          	//Eseguo l'inferenza


                          	             inferenza = ai_network_run(network, &ai_input[0], &ai_output[0]);
                          	             if (inferenza != 1) {

                          	            	 sprintf(buffer2,"Errore");
                          	            	 uprintf(buffer2);

                          	             }

                          	             //Visualizzo il risultato dell'inferenza

                          	             float y_val;
                          	             for(int j=0;j<3;j++){
                          	             y_val = ((float *)out_data)[j];
                          	             sprintf(buffer2,"%f \n \r",y_val);
                          	             uprintf(buffer2);
                          	             }



    //Fine main

  }




  /* USER CODE END 3 */


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
