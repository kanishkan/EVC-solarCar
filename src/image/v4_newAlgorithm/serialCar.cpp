#include <termios.h>		//Used for UART
#include "serialCar.h"
#include "string.h" 

//int main(void){
//    unsigned char command[5]    = {'0','0','1','T','+'};
//   
//    // Serial Communicaion 
//    int uart0_filestream        = openPort();
//    setupSerial(uart0_filestream);
//    sendCommand(command,uart0_filestream); 
//
//    return(0);
//}

int openPort(){
    int uart0_filestream = -1;
    uart0_filestream = open("/dev/ttyACM0", O_RDWR | O_NOCTTY | O_NDELAY);        //Open in non blocking read/write mode
    if (uart0_filestream == -1) {
        printf("Error - Unable to open UART.  Ensure it is not in use by another application\n");
    }
    return(uart0_filestream); 
}

void setupSerial(int uart0_filestream){
    struct termios options;
    tcgetattr(uart0_filestream, &options);
    options.c_cflag = B9600 | CS8 | CLOCAL | CREAD;        //<Set baud rate
    options.c_iflag = IGNPAR;
    options.c_oflag = 0;
    options.c_lflag = 0;
    tcflush(uart0_filestream, TCIFLUSH);
    tcsetattr(uart0_filestream, TCSANOW, &options);
}

void sendCommand(unsigned char* command,int uart0_filestream){
    if (uart0_filestream != -1) {
        //int count = write(uart0_filestream, &tx_buffer[0], (p_tx_buffer - &tx_buffer[0]));
        int count = write(uart0_filestream,command , 5);
        if (count < 0) {
            printf("UART TX error\n");
        }
    }
}
 
