 /*
 * Copyright (c) 2015-2016, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */




/*
 *  ======== empty.c ========
 */
/* XDCtools Header files */
#include <xdc/std.h>
#include <xdc/runtime/System.h>


#include <ti/drivers/GPIO.h>
#include <ti/drivers/I2C.h>
#include "iic_lib.h"


/*
 *  ======== httpget.c ========
 *  HTTP Client GET example application
 */
#include <string.h>

/* XDCtools Header files */
#include <xdc/runtime/Error.h>
#include <xdc/runtime/System.h>
#include <time.h>
/* TI-RTOS Header files */
#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/knl/Task.h>
#include <ti/sysbios/knl/Swi.h>
#include <ti/sysbios/knl/Queue.h>
#include <ti/sysbios/knl/Event.h>
#include <ti/sysbios/knl/Idle.h>
#include <ti/sysbios/knl/Mailbox.h>
#include <ti/sysbios/knl/Clock.h>
#include <ti/sysbios/knl/Semaphore.h>
#include <ti/drivers/GPIO.h>
#include <ti/net/http/httpcli.h>

/* Example/Board Header file */
#include "Board.h"

#include <sys/socket.h>

#define HOSTNAME          "api.openweathermap.org"
#define REQUEST_URI       "/data/2.5/forecast/?id=315202&APPID=b9bdaf75a7b1e96362a172ec83cb9303"
#define USER_AGENT        "HTTPCli (ARM; TI-RTOS)"
#define SOCKETTEST_IP     "192.168.1.22"
#define TASKSTACKSIZE     4096


#define device_ID 0x1D
#define BMP180_ID 0x77


extern Semaphore_Handle semaphore0;
extern Mailbox_Handle mailbox0;
extern Mailbox_Handle mailbox1;
extern Mailbox_Handle mailbox2;
extern Mailbox_Handle mailbox3;

Task_Struct task0Struct;
Char task0Stack[TASKSTACKSIZE];


I2C_Handle      i2c;
I2C_Params      i2cParams;
I2C_Transaction i2cTransaction;

uint8_t         txBuffer[4];
uint8_t         rxBuffer[30];

short AC1, AC2, AC3, B1, B2, MB, MC, MD;    // calibration register(BMP180 datasheet page 15)
unsigned short AC4;               // calibration register(BMP180 datasheet page 15)
long UP;    //pressure
float B3, B4, B6, B7, X1t, X1p, X2t, X2p, X3p, B5, B5p, Altitude;
int ctr, ctr_Move = 0; // every number taken 10 times
int average = 0;

char   tempstr[30];
char buffer[80];                    //time data

int MoveX = 0, MoveY = 0, MoveZ = 0; // X, Y and Z axis of MMA7455



Void Hwi1(UArg arg1){
    Semaphore_post(semaphore0);
}




void printError(char *errString, int code)
{

    System_printf("Error! code = %d, desc = %s\n", code, errString);
    BIOS_exit(code);
}


/*
 *  ======== httpTask ========
 *  Makes a HTTP GET request
 */




Void socketTask(UArg arg0, UArg arg1)
{
    System_printf("working socketTask\n");
    System_flush();
    while(1) {
          char data[40];
          System_printf("working socketTask\n");
          System_flush();
          Mailbox_pend(mailbox3, &data, BIOS_WAIT_FOREVER);
          System_flush();
          int sockfd;
          struct sockaddr_in serverAddr;

          sockfd = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
          if (sockfd == -1) {
              System_printf("Socket not created");
              BIOS_exit(-1);
          }

          memset(&serverAddr, 0, sizeof(serverAddr));  /* clear serverAddr structure */
          serverAddr.sin_family = AF_INET;
          serverAddr.sin_port = htons(5011);     /* convert port # to network order */
          inet_pton(AF_INET, SOCKETTEST_IP, &(serverAddr.sin_addr));

          int connStat = connect(sockfd, (struct sockaddr *)&serverAddr, /* connecting….*/
                        sizeof(serverAddr));
          if(connStat < 0) {
              System_printf("Error while connecting to server\n");
              if (sockfd > 0)
                  close(sockfd);
              BIOS_exit(-1);
          }
          get_socketTime();
          strcat(data," ");
          strcat(data, buffer);

          strcat(data,"\n");
          int numSend = send(sockfd, data, strlen(data), 0);       /* send data to the server*/
          if(numSend < 0) {
              System_printf("Error while sending data to server\n");
              if (sockfd > 0) close(sockfd);
              BIOS_exit(-1);
          }

          if (sockfd > 0) close(sockfd);;
    }

}
/*
 *  ======== netIPAddrHook ========
 *  This function is called when IP Addr is added/deleted
 */

void netIPAddrHook(unsigned int IPAddr, unsigned int IfIdx, unsigned int fAdd)
{

    static Task_Handle taskHandle2;
    Task_Params taskParams;
    Error_Block eb;
    if (fAdd  && !taskHandle2) {
       Error_init(&eb);
    }

    Task_Params_init(&taskParams);
    taskParams.stackSize = TASKSTACKSIZE;
    taskParams.priority = 4;
    taskHandle2 = Task_create((Task_FuncPtr)socketTask, &taskParams, &eb);

    if (taskHandle2 == NULL) {
       printError("netIPAddrHook: Failed to create HTTP and Socket Tasks\n", -1);
    }
    System_printf("netIPAddrHook end\n");
    System_flush();

}


void get_socketTime(void)
{
    char time[4];
    int fd, connStat;
    struct sockaddr_in serverAddr;

    fd = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP); //create socket
    if (fd == -1) {
       System_printf("Socket not created");
       BIOS_exit(-1);
    }

    memset(&serverAddr, 0, sizeof(serverAddr));  // clear serverAddr structure
    serverAddr.sin_family = AF_INET;
    serverAddr.sin_port = htons(37);     // convert port # to network order
    inet_pton(AF_INET, "132.163.96.5" , &(serverAddr.sin_addr));

    connStat = connect(fd, (struct sockaddr *)&serverAddr, sizeof(serverAddr));
    if(connStat < 0) {
       System_printf("sendData2Server::Error while connecting to server\n");
       if(fd>0) close(fd);
       BIOS_exit(-1);

    }


    recv(fd, time, 4, 0);       //read time data
    if(fd>0) close(fd);


    unsigned long int seconds= time[0]*16777216 +  time[1]*65536 + time[2]*256 + time[3];   // total seconds since 01.01.1900



    time_t rawtime =(seconds+10800); // +3 hours to set gmt +3
    struct tm  time2;


    // Format time, "ddd yyyy-mm-dd hh:mm:ss zzz"
    time2 = *localtime(&rawtime);
    strftime(buffer, sizeof(buffer), "%Y.%m.%d %H:%M:%S", &time2);

}



int getFreeFall(void){
    char buf[2];
    IIC_writeReg(device_ID, 0x16, 0x12); //case 4
    IIC_writeReg(device_ID, 0x18, 0x40); //case 4
    IIC_writeReg(device_ID, 0x19, 0x01); //case 4
    IIC_writeReg(device_ID, 0x1A, 0x07); //case 4
    //IIC_writeReg(device_ID, 0x1C, 0xAB); //pulse duration AB*0.5ms

    IIC_readReg(device_ID,0x19, 1, buf);
    System_printf("Freefall situation: %d \n", buf[0]);
    System_flush();
    return buf[0];
}

void setCalibration(void)
{
    txBuffer[0] = 0xAA;

    i2cTransaction.slaveAddress = BMP180_ID;
    i2cTransaction.writeBuf = txBuffer;
    i2cTransaction.writeCount = 1;
    i2cTransaction.readBuf = rxBuffer;
    i2cTransaction.readCount = 22;

    if (I2C_transfer(i2c, &i2cTransaction)) {

           /* all register defined in below used to calibration of pressure data
            * Detail information check BMP180 data sheet page 15
            */
           AC1 = rxBuffer[0]<<8 | rxBuffer[1]; //0xAA, 0xAB
           AC2 = rxBuffer[2]<<8 | rxBuffer[3]; //0xAC, 0xAD
           AC3 = rxBuffer[4]<<8 | rxBuffer[5]; //0xAE, 0xAF
           AC4 = rxBuffer[6]<<8 | rxBuffer[7]; //0xB0, 0xB1
           B1 = rxBuffer[12]<<8 | rxBuffer[13]; //0xB6, 0xB7
           B2 = rxBuffer[14]<<8 | rxBuffer[15]; //0xB8, 0xB9
           MB = rxBuffer[16]<<8 | rxBuffer[17]; //0xBA, 0xBB
    }

}


void setPressure(void)
{
    txBuffer[0] = 0xf4;         //control register value
    txBuffer[1] = 0xB4;         //13.5 ms max. conversion time
    i2cTransaction.slaveAddress = BMP180_ID;
    i2cTransaction.writeBuf = txBuffer;
    i2cTransaction.writeCount = 2;
    i2cTransaction.readBuf = rxBuffer;
    i2cTransaction.readCount = 0;
}

int getMoveX(){
    char buf[2];
    IIC_readReg(device_ID,0x06, 1, buf);
    System_flush();
    return buf[0];
}

int getMoveY(){
    char buf[2];
    IIC_readReg(device_ID,0x07, 1, buf);
    System_flush();
    return buf[0];
}

int getMoveZ(){
    char buf[2];
    IIC_readReg(device_ID,0x08, 1, buf);
    System_flush();
    return buf[0];
}



float getPressure(void)
{
    float pressure;

    txBuffer[0] = 0xf6;                                 // MSB of pressure value
    i2cTransaction.slaveAddress = BMP180_ID;
    i2cTransaction.writeBuf = txBuffer;
    i2cTransaction.writeCount = 1;
    i2cTransaction.readBuf = rxBuffer;
    i2cTransaction.readCount = 2;

    if (I2C_transfer(i2c, &i2cTransaction)) {
       //System_printf("Pressure value acquired\n");
    }


    UP = rxBuffer[0]<<8 | rxBuffer[1];

    /*
     * Calculate true pressure all equations taken from BMP180 data sheet figure 4
     */
    B6 = B5 - 4000;
    X1p = (B2 * (B6 * B6 / 4096)) / 2048;
    X2p = AC2 * B6 / 2048;
    X3p = X1p = X2p;
    B3 = ((((long)AC1 * 4 + X3p)) + 2) / 4;
    X1p = AC3 * B6 / 8192;
    X2p = (B1 * (B6 * B6 / 4096)) / 65536;
    X3p = ((X1p + X2p) + 2) / 4;
    B4 = AC4 * (unsigned long)(X3p + 32768) / 32768;
    B7 = ((unsigned long)UP - B3) * (50000);
    if (B7 < 0x80000000) {
        pressure = (B7 * 2) / B4;
    }
    else {
        pressure = (B7 / B4) * 2;
    }
    X1p = (pressure / 256) * (pressure / 256);
    X1p = (X1p * 3038) / 65536;
    X2p = (-7357 * pressure) / 65536;
    pressure = pressure + (X1p + X2p + 3791) / 16;
    pressure = pressure / 100.0f ; // Pa to mbar
    return pressure;
}



Void taskFxn(UArg arg0, UArg arg1)
{

    IIC_OpenComm();
    /*
    setCalibration();
    System_flush();
    BMP180_startPressureAcquisition();
    */
    while(1){
        Semaphore_pend(semaphore0, BIOS_WAIT_FOREVER); //posted in Hwý with period 1 sec.
        setCalibration();
        setPressure();
        int pressure = getPressure();

        ctr++;

        average += pressure;
        //Mailbox_post(mailbox0, &average, BIOS_NO_WAIT);


        if(ctr % 3 == 0){

            average = (average / 3);
            System_flush();
            Mailbox_post(mailbox0, &average, BIOS_NO_WAIT);
            average = 0;
        }


        Task_sleep(1000);
        }


}

Void taskFxn2(UArg arg0, UArg arg1){
    while(1){

        //float altitude, altitudeBasic;

        Mailbox_pend(mailbox0, &average, BIOS_WAIT_FOREVER);

        //altitude = 44330.0f * (1.0f - powf(average / 101325.0f, 1 / 5.255f));
        //altitudeBasic = 44330.0f * (1.0f - powf(850 / 101325.0f, 1 / 5.255f));
        System_printf("I set pressure change function always 1(For this experiment) because it is not changing in the home condition  \n");
        System_flush();
        if(abs(average) < 644,7)){ //644,7 calculated by formula in BMP180 data sheep page 16
                                   //if the pressure value less than 644,7 it mean falling more than 3 meter. For Eskisehir/Turkey
            int boolean = 1;
            Mailbox_post(mailbox1, &boolean, BIOS_NO_WAIT);
        }
        else{
            int boolean = 0;
            Mailbox_post(mailbox1, &boolean, BIOS_NO_WAIT);
        }

        //System_printf("average %d %d\n", altitude, altitudeBasic);
        System_flush();

        average = 0;
    }
}


Void taskIsmove(UArg arg0, UArg arg1){
    int boolean;
    while(1){

        Mailbox_pend(mailbox1, &boolean, BIOS_WAIT_FOREVER);
        int x = getMoveX();
        int y = getMoveY();
        int z = getMoveZ();

        if(boolean){
            ctr_Move++;
            MoveX = MoveX + abs(x);
            MoveY = MoveY + abs(y);
            MoveZ = MoveZ + abs(z);
            if(ctr_Move % 5 == 0){
                int Average_Move = abs(abs(MoveX + MoveY + MoveZ)/5 - (abs(x) + abs(y) + abs(z)));
                System_printf("\n MoveX + MoveY + MoveZ /5 - (abs(x) + abs(y) + abs(z) %d\n", Average_Move);

                if(Average_Move < 10){      // if the axis not change then object is not moving
                    System_printf("The falling object is not moving, post True for next steps. \n", average);
                    System_flush();
                    boolean = 1;
                    Mailbox_post(mailbox2, &boolean, BIOS_NO_WAIT);
                    MoveX = 0;
                    MoveY = 0;
                    MoveZ = 0;
                }
                else{       // if the axis change then object moving
                    System_printf("The falling object is moving, post false for next steps. \n", average);
                    System_flush();
                    boolean = 0;
                    Mailbox_post(mailbox2, &boolean, BIOS_NO_WAIT);
                    MoveX = 0;
                    MoveY = 0;
                    MoveZ = 0;
                }
            }

        }


    }


}

Void taskFxn3(UArg arg0, UArg arg1){
    char data[30] = "FreeFall";
    while(1){

        int boolean;

        Mailbox_pend(mailbox2, &boolean, BIOS_WAIT_FOREVER);

        if(boolean){
            boolean = getFreeFall();
            if(boolean){
                System_printf("Free Fall detected write it on hercules. \n", average);
                System_flush();
                Mailbox_post(mailbox3, &data, BIOS_NO_WAIT);
            }
        }
        average = 0;
    }
}




/*
 *  ======== main ========
 */

int main(void)
{
    /* Call board init functions */
    Board_initGeneral();
    Board_initGPIO();
    Board_initEMAC();
    Board_initI2C();
    /* Turn on user LED */
    GPIO_write(Board_LED0, Board_LED_ON);

    System_printf("Starting the HTTP GET example\nSystem provider is set to "
            "SysMin. Halt the target to view any SysMin contents in ROV.\n");
    /* SysMin will only print to the console when you call flush or exit */

    System_flush();


    /* Start BIOS */
    BIOS_start();

    System_flush();
    return (0);
}
