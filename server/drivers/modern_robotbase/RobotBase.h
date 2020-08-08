//
// Created by raysuner on 19-7-10.
//

#ifndef CARTOGRAPHER_SUPERBUILD_Robot_H
#define CARTOGRAPHER_SUPERBUILD_Robot_H

#include <chrono>
#include <iostream>
#include <assert.h>
#include <termios.h>
#include <sys/ioctl.h> // ioctl
#include <unistd.h> // close(2),fcntl(2),getpid(2),usleep(3),execvp(3),fork(2)
#include <netdb.h> // for gethostbyname(3)
#include <netinet/in.h>  // for struct sockaddr_in, htons(3)
#include <sys/types.h>  // for socket(2)
#include <sys/socket.h>  // for socket(2)
#include <signal.h>  // for kill(2)
#include <fcntl.h>  // for fcntl(2)
#include <string.h>  // for strncpy(3),memcpy(3)
#include <stdlib.h>  // for atexit(3),atoi(3)
#include <pthread.h>  // for pthread stuff
#include <math.h>

#include "player_ekf_fusion.h"



#define MESSAGE_ERROR					0
#define MESSAGE_INFO				    1
#define MESSAGE_DEBUG					2

using TimePoint = std::chrono::steady_clock::time_point;

//串口结构
typedef struct{
    char prompt;  //prompt after reciving data
    int  baudrate;  //baudrate
    char databit;  //data bits, 5, 6, 7, 8
    char  debug;  //debug mode, 0: none, 1: debug
    char  echo;   //echo mode, 0: none, 1: echo
    char fctl;   //flow control, 0: none, 1: hardware, 2: software
    const char *port = "/dev/ttyUSB0";   //port
    char parity;  //parity 0: none, 1: odd, 2: even
    char stopbit;  //stop bits, 1, 2
    int reserved; //reserved, must be zero
}portinfo_t;

class RobotBase : public ThreadedDriver{
private:
    int fd_;
    const char *port_ ;
    portinfo_t portinfo_;
    TimePoint current_time_, last_time_;
    unsigned char recvbuf_[8];
    double rate_;
    double recrate_;
    double last_imu_v_;
    double last_vth_;
    double pure_imu=0;
    double pure_odo=0;
    player_position2d_geom_t pos_geom;



    player_pose2d_t current_position_;
    player_position2d_data_t position_data_;
    player_devaddr_t position_addr_; //position2d interface

    double clock_sum;
    bool addimu_;
    Fusion fusion;


    void openPort(){
        fd_ = open(port_, O_RDWR | O_NOCTTY | O_NONBLOCK);
    }

    void setPort(){
        struct termios termios_old, termios_new;
        int     baudrate, tmp;
        char    databit, stopbit, parity, fctl;

        bzero(&termios_old, sizeof(termios_old));
        bzero(&termios_new, sizeof(termios_new));
        cfmakeraw(&termios_new);
        tcgetattr(fd_, &termios_old);         //get the serial port attributions
        /*------------设置端口属性----------------*/
        cfsetispeed(&termios_new, B115200);        //填入串口输入端的波特率
        cfsetospeed(&termios_new, B115200);        //填入串口输出端的波特率
        termios_new.c_cflag |= CLOCAL;          //控制模式，保证程序不会成为端口的占有者
        termios_new.c_cflag |= CREAD;           //控制模式，使能端口读取输入的数据

        // 控制模式，flow control
        fctl = portinfo_.fctl;
        switch(fctl){
            case '0':{
                termios_new.c_cflag &= ~CRTSCTS;        //no flow control
            }break;
            case '1':{
                termios_new.c_cflag |= CRTSCTS;         //hardware flow control
            }break;
            case '2':{
                termios_new.c_iflag |= IXON | IXOFF |IXANY; //software flow control
            }break;
        }

        //控制模式，data bits
        termios_new.c_cflag &= ~CSIZE;      //控制模式，屏蔽字符大小位
        databit = portinfo_.databit;
        switch(databit){
            case '5':
                termios_new.c_cflag |= CS5;
            case '6':
                termios_new.c_cflag |= CS6;
            case '7':
                termios_new.c_cflag |= CS7;
            default:
                termios_new.c_cflag |= CS8;
        }

        //控制模式 parity check
        parity = portinfo_.parity;
        switch(parity){
            case '0':{
                termios_new.c_cflag &= ~PARENB;     //no parity check
            }break;
            case '1':{
                termios_new.c_cflag |= PARENB;      //odd check
                termios_new.c_cflag &= ~PARODD;
            }break;
            case '2':{
                termios_new.c_cflag |= PARENB;      //even check
                termios_new.c_cflag |= PARODD;
            }break;
        }

        //控制模式，stop bits
        stopbit = portinfo_.stopbit;
        if(stopbit == '2'){
            termios_new.c_cflag |= CSTOPB;  //2 stop bits
        }
        else{
            termios_new.c_cflag &= ~CSTOPB; //1 stop bits
        }

        //other attributions default
        termios_new.c_oflag &= ~OPOST;          //输出模式，原始数据输出
        termios_new.c_cc[VMIN]  = 1;            //控制字符, 所要读取字符的最小数量
        termios_new.c_cc[VTIME] = 1;            //控制字符, 读取第一个字符的等待时间    unit: (1/10)second

        tcflush(fd_, TCIFLUSH);               //溢出的数据可以接收，但不读
        tmp = tcsetattr(fd_, TCSANOW, &termios_new);  //设置新属性，TCSANOW：所有改变立即生效    tcgetattr(fdcom, &termios_old);
    }

    void init(){
        openPort();
        if(fd_ < 0){
            printf("Error: open serial port error.\n");
            exit(-1);
        }
        else{
            setPort();
        }
    }

    int sendPort(unsigned char *data, int datalen){
        int len = 0;
        len = write(fd_, data, datalen);
        if(len == datalen){
            return len;
        }
        else{
            tcflush(fd_, TCOFLUSH);
            return -1;
        }
    }

    int recvPort(unsigned char *data, int datalen)
    {
        int readlen, fs_sel;
        fd_set  fs_read;
        struct timeval tv_timeout;

        FD_ZERO(&fs_read);
        FD_SET(fd_, &fs_read);
        //tv_timeout.tv_sec = TIMEOUT_SEC(datalen, baudrate);
        tv_timeout.tv_sec=0;
        tv_timeout.tv_usec = 100;

        fs_sel = select(fd_+1, &fs_read, nullptr, nullptr, &tv_timeout);
        if(fs_sel){
            readlen = read(fd_, data, datalen);
            return(readlen);
        }
        else {
            return (-1);
        }
    }



public:

    RobotBase(ConfigFile *cf, int section);


    virtual int MainSetup();

    virtual void MainQuit();

    void Main();

    int ProcessMessage(QueuePointer &resp_queue, player_msghdr_t* hdr, void* data);

    void ProcessOdometry(double dt);

    void WriteMotorVelocity(double vx, double vth);
};


#endif 
