#ifndef _IMU_H_
#define _IMU_H_

#include <time.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <iostream>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <string.h>

#include "i2cInterface.h"
#include "MadgwickAHRS.h"

#define M_PI 3.14159265359f

#define ITG3200_ADDRESS 0x68
#define ITG3200_DLPF_CFG 2  //digtal low pass filter: value 0 to 6 higher is more smooth
#define ITG3200_GAIN 0.00121414208834388 //convert to rad/sec  to_rad(1/lsb) // (1/14.375)/180*pi

#define ADXL345_ADDRESS 0x53
#define ADXL345_GAIN 0.004 //convert to G //  lsb = 4 mg/LSB  4*1e-3

#define HMC5883_ADDRESS 0x1e
//#define HMC5883_GAIN 1 //not sure have to check


#define GYRO_INIT() ITG3200_init()
#define GYRO_READ() ITG3200_read()

#define ACC_INIT()  ADXL345_init()
#define ACC_READ()  ADXL345_read()
#define ACC_GAIN    ADXL345_GAIN

#define MAG_INIT()  HMC5883_init()
#define MAG_READ()  HMC5883_read()


class IMU
{

private :

    timespec time1, time2;

    timespec timeDiff(timespec start, timespec end)
    {
        timespec temp;
        if ((end.tv_nsec-start.tv_nsec)<0)
        {
            temp.tv_sec = end.tv_sec-start.tv_sec-1;
            temp.tv_nsec = 1000000000+end.tv_nsec-start.tv_nsec;
        }
        else
        {
            temp.tv_sec = end.tv_sec-start.tv_sec;
            temp.tv_nsec = end.tv_nsec-start.tv_nsec;
        }
        return temp;
    }

    double timerLap(){
        clock_gettime(CLOCK_REALTIME, &time2);
        timespec dt = timeDiff(time1,time2);

        clock_gettime(CLOCK_REALTIME, &time1);
        return  (double)dt.tv_sec + dt.tv_nsec*1e-9;
    }



public:

    I2cInterface i2c;
    MadgwickAHRS ahrs;

    float gyro_val[3],acc_val[3],mag_val[3];
    int16_t acc_raw[3],gyro_raw[3],mag_raw[3];
    int16_t gyro_offset[3],acc_offset[3];
    int16_t mag_max[3], mag_min[3];

    //buffer for i2c
    uint8_t rawBuff[32];
    float euler[3];
    float acc_mag;


    void init()
    {
        ahrs.setParam(1./100,0.1);
        i2c.openPort("/dev/i2c-4");
        GYRO_INIT();
        ACC_INIT();
        MAG_INIT();

        for(int i=0; i<3; i++)
        {
            mag_max[i] = 1000;
            mag_min[i] = -1000;
            gyro_offset[i] = 0;
            acc_offset[i] =  0;
        }
        calibrate_acc_gyro();
        timerLap();
    }




    void calibrate_acc_gyro()
    {

        int32_t sum_acc[]= {0,0,0};
        int32_t sum_gyro[]= {0,0,0};

        int iteration = 100;

        for(uint8_t i=0; i<iteration; i++)
        {
            ITG3200_read();
            ADXL345_read();

            for(uint8_t j=0; j<3; j++)
            {
                sum_acc[j] += acc_raw[j];
                sum_gyro[j] += gyro_raw[j];
            }
            usleep(10000);
        }

        for(uint8_t i=0; i<3; i++)
        {
            acc_offset[i] = -sum_acc[i]/iteration;
            gyro_offset[i] = -sum_gyro[i]/iteration;
        }
        acc_offset[2] = 1.0/ACC_GAIN + acc_offset[2];

    }

    void update()
    {
        GYRO_READ();
        ACC_READ();
        MAG_READ();

        ahrs.samplePeriod = timerLap();
        ahrs.update(gyro_val[0],gyro_val[1],gyro_val[2],
                    acc_val[0],acc_val[1],acc_val[2],
                    mag_val[0],mag_val[1],mag_val[2]);
        //ahrs.update(0,0,0,acc_val[0],acc_val[1],acc_val[2]);
        //ahrs.update(gyro_val[0],gyro_val[1],gyro_val[2]);

        quaternionToEuler(ahrs.quaternion, euler);

        return;
    }


private:

    void ITG3200_init()
    {
        i2c.writeReg(ITG3200_ADDRESS, 0x3E, 0x80);
        i2c.writeReg(ITG3200_ADDRESS, 0x16, 0x18 + ITG3200_DLPF_CFG); //register: DLPF_CFG - low pass filter configuration
        i2c.writeReg(ITG3200_ADDRESS, 0x3E, 0x03); //register: Power Management  --  value: PLL with Z Gyro reference
    }

    void ITG3200_read()
    {
        i2c.readRegBuf(ITG3200_ADDRESS,0X1D,rawBuff,6);
        gyro_raw[0] = (rawBuff[0]<<8) | rawBuff[1];
        gyro_raw[1] = (rawBuff[2]<<8) | rawBuff[3];
        gyro_raw[2] = (rawBuff[4]<<8) | rawBuff[5];
        gyro_val[0] = (gyro_raw[0] + gyro_offset[0]) * ITG3200_GAIN;
        gyro_val[1] = (gyro_raw[1] + gyro_offset[1]) * ITG3200_GAIN;
        gyro_val[2] = (gyro_raw[2] + gyro_offset[2]) * ITG3200_GAIN;
    }

    void ADXL345_init()
    {
        i2c.writeReg(ADXL345_ADDRESS,0x2D,1<<3); //  register: Power CTRL  -- value: Set measure bit 3 on
        i2c.writeReg(ADXL345_ADDRESS,0x31,0x0B); //  register: DATA_FORMAT -- value: Set bits 3(full range) and 1 0 on (+/- 16g-range)
        i2c.writeReg(ADXL345_ADDRESS,0x2C,0x0A); //  register: BW_RATE     -- value: rate=100hz, bw=50hz
    }

    void ADXL345_read()
    {
        i2c.readRegBuf(ADXL345_ADDRESS,0x32,rawBuff,6);
        acc_raw[0] = (rawBuff[1]<<8) | rawBuff[0];
        acc_raw[1] = (rawBuff[3]<<8) | rawBuff[2];
        acc_raw[2] = (rawBuff[5]<<8) | rawBuff[4];
        acc_val[0] = (float)(acc_raw[0] + acc_offset[0])* ADXL345_GAIN;
        acc_val[1] = (float)(acc_raw[1] + acc_offset[1])* ADXL345_GAIN;
        acc_val[2] = (float)(acc_raw[2] + acc_offset[2])* ADXL345_GAIN;
    }

    void  HMC5883_init()
    {
        //Configuration Register A  -- 0 11 110 00  num samples: 8 ; output rate: 75Hz ; normal measurement mode
        i2c.writeReg(HMC5883_ADDRESS ,0x00 ,0x78 );
        //Configuration Register B  -- 001 00000    configuration gain 1.3Ga
        i2c.writeReg(HMC5883_ADDRESS ,0x01 ,0x20 );
        //Mode register             -- 000000 00    continuous Conversion Mode
        i2c.writeReg(HMC5883_ADDRESS ,0x02 ,0x00 );
    }

    void HMC5883_read()
    {
        i2c.readRegBuf(HMC5883_ADDRESS,0x03,rawBuff,6);
        mag_raw[1] = (rawBuff[0]<<8) | rawBuff[1];
        mag_raw[2] = (rawBuff[2]<<8) | rawBuff[3];
        mag_raw[0] = (rawBuff[4]<<8) | rawBuff[5];
        mag_val[0] = (float)mag_raw[0] / ((mag_raw[0] > 0)?mag_max[0]:-mag_min[0]) ;
        mag_val[1] = -(float)mag_raw[1] / ((mag_raw[1] > 0)?mag_max[1]:-mag_min[1]) ;
        mag_val[2] = (float)mag_raw[2] / ((mag_raw[2] > 0)?mag_max[2]:-mag_min[2]) ;
    }


    void quaternionToEuler(float* q, float* e){
        float sqw = q[0]*q[0];
        float sqx = q[1]*q[1];
        float sqy = q[2]*q[2];
        float sqz = q[3]*q[3];
 
	e[0] = atan2(2.0 * (q[2]*q[3] + q[1]*q[0]) , -sqx - sqy + sqz + sqw );
        e[1] = asin(-2.0 * (q[1]*q[3] - q[2]*q[0]) );
        e[2] = atan2(-2.0 * (q[1]*q[2] + q[3]*q[0]) ,  sqx - sqy - sqz + sqw );

    }



};





#endif
