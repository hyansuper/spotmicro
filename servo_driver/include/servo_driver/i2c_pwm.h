#ifndef __SM_I2C_PWM__
#define __SM_I2C_PWM__

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sstream>
#include <math.h>
#include <fcntl.h>
#include <errno.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <linux/i2c-dev.h>
#include <ros/ros.h>


namespace servo_driver{

class I2CPWM{

    /**
    most codes in this class came from https://gitlab.com/bradanlane/ros-i2cpwmboard
    **/

    #define _BASE_ADDR   0x40

    enum pwm_regs {
        // Registers/etc.
        __MODE1              = 0x00,
        __MODE2              = 0x01,
        __SUBADR1            = 0x02,      // enable sub address 1 support
        __SUBADR2            = 0x03,      // enable sub address 2 support
        __SUBADR3            = 0x04,      // enable sub address 2 support
        __PRESCALE           = 0xFE,
        __CHANNEL_ON_L       = 0x06,
        __CHANNEL_ON_H       = 0x07,
        __CHANNEL_OFF_L      = 0x08,
        __CHANNEL_OFF_H      = 0x09,
        __ALL_CHANNELS_ON_L  = 0xFA,
        __ALL_CHANNELS_ON_H  = 0xFB,
        __ALL_CHANNELS_OFF_L = 0xFC,
        __ALL_CHANNELS_OFF_H = 0xFD,
        __RESTART            = 0x80,
        __SLEEP              = 0x10,      // enable low power mode
        __ALLCALL            = 0x01,
        __INVRT              = 0x10,      // invert the output control logic
        __OUTDRV             = 0x04
    };

    int io_handle;
    int active_board;

    public:

    void set_pwm_freq(int freq)
    {
        int prescale;
        char oldmode, newmode;
        int res;
        
        ROS_DEBUG("_set_pwm_frequency prescale");
        float prescaleval = 25000000.0; // 25MHz
        prescaleval /= 4096.0;
        prescaleval /= (float)freq;
        prescaleval -= 1.0;
        //ROS_INFO("Estimated pre-scale: %6.4f", prescaleval);
        prescale = floor(prescaleval + 0.5);
        // ROS_INFO("Final pre-scale: %d", prescale);

        ROS_INFO("Setting PWM frequency to %d Hz", freq);

        nanosleep ((const struct timespec[]){{1, 000000L}}, NULL);

        oldmode = i2c_smbus_read_byte_data (io_handle, __MODE1);
        newmode = (oldmode & 0x7F) | 0x10; // sleep

        if (0 > i2c_smbus_write_byte_data (io_handle, __MODE1, newmode)) // go to sleep
            ROS_ERROR("Unable to set PWM controller to sleep mode"); 

        if (0 >  i2c_smbus_write_byte_data(io_handle, __PRESCALE, (int)(floor(prescale))))
            ROS_ERROR("Unable to set PWM controller prescale"); 

        if (0 > i2c_smbus_write_byte_data(io_handle, __MODE1, oldmode))
            ROS_ERROR("Unable to set PWM controller to active mode"); 

        nanosleep((const struct timespec[]){{0, 5000000L}}, NULL);   //sleep 5microsec,

        if (0 > i2c_smbus_write_byte_data(io_handle, __MODE1, oldmode | 0x80))
            ROS_ERROR("Unable to restore PWM controller to active mode");        
    }

    void set_pwm_interval(int channel,int start,int end)
    {
        if (0 > i2c_smbus_write_byte_data (io_handle, __CHANNEL_ON_L+4*channel, start & 0xFF))
            ROS_ERROR ("Error setting PWM start low byte on servo %d on board %d", channel, active_board);
        if (0 >  i2c_smbus_write_byte_data (io_handle, __CHANNEL_ON_H+4*channel, start  >> 8))
            ROS_ERROR ("Error setting PWM start high byte on servo %d on board %d", channel, active_board);
        if (0 > i2c_smbus_write_byte_data (io_handle, __CHANNEL_OFF_L+4*channel, end & 0xFF))
            ROS_ERROR ("Error setting PWM end low byte on servo %d on board %d", channel, active_board);
        if (0 > i2c_smbus_write_byte_data (io_handle, __CHANNEL_OFF_H+4*channel, end >> 8))
            ROS_ERROR ("Error setting PWM end high byte on servo %d on board %d", channel, active_board);
    }

    void set_pwm_interval_all(int start ,int end)
    {
        if (0 > i2c_smbus_write_byte_data (io_handle, __ALL_CHANNELS_ON_L, start & 0xFF))
            ROS_ERROR ("Error setting PWM start low byte for all servos on board %d", active_board);
        if (0 >  i2c_smbus_write_byte_data (io_handle, __ALL_CHANNELS_ON_H, start  >> 8))
            ROS_ERROR ("Error setting PWM start high byte for all servos on board %d", active_board);
        if (0 > i2c_smbus_write_byte_data (io_handle, __ALL_CHANNELS_OFF_L, end & 0xFF))
            ROS_ERROR ("Error setting PWM end low byte for all servos on board %d", active_board);
        if (0 > i2c_smbus_write_byte_data (io_handle, __ALL_CHANNELS_OFF_H, end >> 8))
            ROS_ERROR ("Error setting PWM end high byte for all servos on board %d", active_board);
    }

    void set_active_board(int board) 
    {
        if (active_board != board) {
            active_board = board;   // save to global        
            if (0 > ioctl (io_handle, I2C_SLAVE, (_BASE_ADDR+(board)))) {
                ROS_FATAL ("Failed to acquire bus access and/or talk to I2C slave at address 0x%02X", (_BASE_ADDR+board));
            }
        }
    }

    void init_board()
    {
        char mode1res;
        /* this is guess but I believe the following needs to be done on each board only once */        
        if (0 > i2c_smbus_write_byte_data (io_handle, __MODE2, __OUTDRV))
            ROS_ERROR ("Failed to enable PWM outputs for totem-pole structure");

        if (0 > i2c_smbus_write_byte_data (io_handle, __MODE1, __ALLCALL))
            ROS_ERROR ("Failed to enable ALLCALL for PWM channels");

        nanosleep ((const struct timespec[]){{0, 5000000L}}, NULL);   //sleep 5microsec, wait for osci

        mode1res = i2c_smbus_read_byte_data (io_handle, __MODE1);
        mode1res = mode1res & ~__SLEEP; //                 # wake up (reset sleep)

        if (0 > i2c_smbus_write_byte_data (io_handle, __MODE1, mode1res))
            ROS_ERROR ("Failed to recover from low power mode");

        nanosleep((const struct timespec[]){{0, 5000000L}}, NULL);   //sleep 5microsec, wait for osci

        // the first time we activate a board, we mark it and set all of its servo channels to 0
        set_pwm_interval_all (0, 0);        
    }

    bool open_i2c(const char* i2c_dev_file)
    {
        io_handle=open(i2c_dev_file,O_RDWR);
        return io_handle>=0;
    } 

    void close_i2c()
    {
        close(io_handle);        
        io_handle=-1;
    }

    bool i2c_opened(){
        return io_handle>=0;
    }

    I2CPWM():io_handle(-1),active_board(-1) {}

    ~I2CPWM()
    {
        if(i2c_opened())
            close_i2c();
    }

};

}

#endif