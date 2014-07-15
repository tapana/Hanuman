#ifndef _I2C_INTERFACE_H_
#define _I2C_INTERFACE_H_

#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <stdint.h>

class I2cInterface{

public:
    I2cInterface(){
        portfd = -1;
    }


    //port : i2c port name eg: /dev/i2c-4
    int openPort(char* portName){

        if(portfd != -1){
            close(portfd);
            portfd = -1;
        }

        portfd = open(portName, O_RDWR);
        //ortfd = open(portName, O_RDWR | O_NOCTTY | O_NDELAY);

        if(portfd ==-1){
            perror("unable to open given i2c port");
            return 1;
        }

        //disable 10 bit address
        ioctl(portfd,I2C_TENBIT,0);


    }


    int writeReg(int address, uint8_t reg, uint8_t value){

        ioctl(portfd,I2C_SLAVE,address);
        return i2c_smbus_write_byte_data(portfd, reg,value);
    }

    int readReg(int address, uint8_t reg){
        ioctl(portfd,I2C_SLAVE,address);
        return i2c_smbus_read_byte_data(portfd,reg);
    }

    int writeRegBuf(int address, uint8_t reg, uint8_t* buff, int length){
        ioctl(portfd,I2C_SLAVE,address);
        return i2c_smbus_write_i2c_block_data(portfd, reg,length,buff);
    }

    int readRegBuf(int address, uint8_t reg, uint8_t* buff, int length){
        ioctl(portfd,I2C_SLAVE,address);
        return i2c_smbus_read_i2c_block_data(portfd, reg,length, buff);
    }


private:
    int portfd;

};


#endif
