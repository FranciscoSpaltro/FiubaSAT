#include "fatfs.h"
#include "fatfs_sd.h"
#include <string.h>
#include <stdio.h>

/*
SPI_HandleTypeDef hspi1;

UART_HandleTypeDef huart1;

*/

/* USER CODE BEGIN PV */
FATFS fs;  				// file system
FIL fil; 					// File
FILINFO fno;
FRESULT fresult;  // result
UINT br, bw;  		// File read/write count

/**** microSD capacity related *****/
FATFS *pfs;
DWORD fre_clust;

char buffer[128];	//buffer to write/read in microSD

/* USER CODE BEGIN PFP */
static void bufclear(char *buf, uint16_t len);

void uart_puts(const char *str) {
    while (*str) {
        uart_putc(*str); // Envía el carácter actual
        str++;           // Avanza al siguiente carácter en la cadena
    }
}
static void microSD_init(void);
static void microSD_getSize(void);
static void microSD_put(char *name);
static void microSD_get(char *name);


void sd_example(void){
  
  MX_FATFS_Init();
  uart_puts("MX_FATFS_Init...\r\n");   
  /* USER CODE BEGIN 2 */
	microSD_init();
    uart_puts("microSD_init...\r\n");    
	microSD_getSize();
    uart_puts("microSD_getSizet...\r\n"); 
	microSD_put("prueba1.txt");
	microSD_get("prueba1.txt");
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1) {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	}
  /* USER CODE END 3 */
}


/* USER CODE BEGIN 4 */
//Funcion para limpiar el buffer
static void bufclear(char *buf, uint16_t len) {
	for (int i = 0; i < len; i++) {
		buf[i] = '\0';
	}
}
//------------------------------------------------------------

static void send_uart(char *string) {
	uint8_t len = strlen(string);
	uart_puts(string);
}
//------------------------------------------------------------

//Montamos el sistema de archivos
static void microSD_init(void) {
	fresult = f_mount(&fs, "/", 1);
	if (fresult != FR_OK)
		uart_puts("ERROR!!! montaje tarjeta microSD fallido...\n");
	else
		uart_puts("microSD CARD montada con exito\n");
}
//------------------------------------------------------------

//leemos la capacidad y el espacio libre
static void microSD_getSize(void) {
	f_getfree("", &fre_clust, &pfs);	//check free space

	//get microSD size in KB
	uint32_t total = (uint32_t) ((pfs->n_fatent - 2) * pfs->csize * 0.5);
	sprintf(buffer, "SD CARD Total Size: \t%luKB\n", total);
	uart_puts(buffer);
	uint16_t len = strlen(buffer);
	bufclear(buffer, len);

	//get free space in KB
	uint32_t freeSpace;
	freeSpace = (uint32_t) (fre_clust * pfs->csize * 0.5);
	sprintf(buffer, "SD CARD Free Space: \t%luKB\n\n", freeSpace);
	uart_puts(buffer);
	len = strlen(buffer);
	bufclear(buffer, len);
}
//------------------------------------------------------------

//guardamos informacion con PUTS
static void microSD_put(char *name) {
	//Open file to write/ create a file if it doesn't exist
	fresult = f_open(&fil, name, FA_OPEN_ALWAYS | FA_READ | FA_WRITE);

	//Write data
	f_puts("Prueba PUTS, suscribete Prro!\n", &fil);

	//Close file
	fresult = f_close(&fil);

	if (fresult == FR_OK) {
		sprintf(buffer, "%s%s", name, " creado y los datos fueron escritos\n");
		uart_puts(buffer);
	}

	uint8_t len = strlen(buffer);
	bufclear(buffer, len);
}
//------------------------------------------------------------

//guardamos informacion con PUTS
static void microSD_get(char *name) {
	//Open file to read
	fresult = f_open(&fil, name, FA_READ);

	if (fresult == FR_OK) {
		//Read data from the file */
		f_gets(buffer, f_size(&fil), &fil);

		char msg[50];
		sprintf(msg, "%s%s", name,
				" Fue abierto y contiene los siguientes datos:\n");
		uart_puts(msg);
		uart_puts(buffer);
		send_uart("\n");

		/* Close file */
		f_close(&fil);
		uint8_t len = strlen(buffer);
		bufclear(buffer, len);
	}
}
//------------------------------------------------------------
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
	while (1) {
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