#include "fatfs.h"
#include "fatfs_sd.h"
#include <string.h>
#include <stdio.h>
#include "sd.h"

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
extern void microSD_init(void) {
	fresult = f_mount(&fs, "/", 1);
	if (fresult != FR_OK)
		uart_puts("ERROR!!! montaje tarjeta microSD fallido...\r\n");
	else
		uart_puts("microSD CARD montada con exito\r\n");
}
//------------------------------------------------------------

//leemos la capacidad y el espacio libre
extern void microSD_getSize(void) {
	f_getfree("", &fre_clust, &pfs);	//check free space

	//get microSD size in KB
	uint32_t total = (uint32_t) ((pfs->n_fatent - 2) * pfs->csize * 0.5);
	sprintf(buffer, "SD CARD Total Size: \t%luKB\r\n", total);
	uart_puts(buffer);
	uint16_t len = strlen(buffer);
	bufclear(buffer, len);

	//get free space in KB
	uint32_t freeSpace;
	freeSpace = (uint32_t) (fre_clust * pfs->csize * 0.5);
	sprintf(buffer, "SD CARD Free Space: \t%luKB\r\n", freeSpace);
	uart_puts(buffer);
	len = strlen(buffer);
	bufclear(buffer, len);
}
//------------------------------------------------------------

//guardamos informacion con PUTS
extern void microSD_put(char *name, char *string) {
	//Open file to write/ create a file if it doesn't exist
	fresult = f_open(&fil, name, FA_OPEN_ALWAYS | FA_READ | FA_WRITE);
	if (fresult == FR_INVALID_OBJECT) uart_puts("Retorno FR_INVALID_OBJECT\r\n");

	//Write data
	f_puts(string, &fil);
	
	//Close file
	fresult = f_close(&fil);

	if (fresult == FR_OK) {
		sprintf(buffer, "%s%s", name, " creado y los datos fueron escritos\r\n");
		uart_puts(buffer);
	}

	uint8_t len = strlen(buffer);
	bufclear(buffer, len);
}
//------------------------------------------------------------

//guardamos informacion con PUTS
extern void microSD_get(char *name) {
	//Open file to read
	fresult = f_open(&fil, name, FA_READ);

	if (fresult == FR_OK) {
		//Read data from the file */
		f_gets(buffer, f_size(&fil), &fil);

		char msg[50];
		sprintf(msg, "%s%s", name,
				" Fue abierto y contiene los siguientes datos:\r\n");
		uart_puts(msg);
		uart_puts(buffer);
		send_uart("\r\n");

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
		uart_puts("Error_Handler...\r\n");
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
     ex: printf("Wrong parameters value: file %s on line %d\r\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */