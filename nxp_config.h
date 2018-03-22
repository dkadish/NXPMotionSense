//
// Created by David Kadish on 07/03/2018.
//

#ifndef NXPMOTIONSENSE_CONFIG_H
#define NXPMOTIONSENSE_CONFIG_H

#ifdef CORE_TEENSY
    #define USE_I2C_T3
    #include <i2c_t3.h>
    typedef i2c_t3 wire_t;
#else
    #include <Wire.h>
    typedef TwoWire wire_t;
#endif

#endif //NXPMOTIONSENSE_CONFIG_H
