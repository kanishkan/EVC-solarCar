#include <stdio.h>
#include <unistd.h>			//Used for UART
#include <fcntl.h>			//Used for UART

int openPort(void);
void setupSerial(int uart0_filestream);
void sendCommand(unsigned char* command,int uart0_filestream);
