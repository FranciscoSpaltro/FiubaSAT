#ifndef SD_H
#define SH_H

void sd_example(void);
extern void microSD_init(void);
extern void microSD_getSize(void);
extern void microSD_put(char *name, char *string); 
extern void microSD_get(char *name);
void uart_puts(const char *str);

#endif /* SD_H */
