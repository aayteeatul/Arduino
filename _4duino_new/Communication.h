#ifndef COMMUNICATION_H
#define COMMUNICATION_H


typedef enum {
    I2C_ERROR,
    I2C_SUCCESS,
    I2C_RESET,
    I2C_HOME,
    I2C_CONFIGURATION_1,
    I2C_CONFIGURATION_2,
    I2C_CONFIGURATION_3,
    I2C_CONFIGURATION_4,
    I2C_CONFIGURATION_5,
    I2C_CONFIGURATION_6,
    I2C_CONFIGURATION_7,
    I2C_CONFIGURATION_8,
    I2C_CONFIGURATION_9,
    I2C_CONFIGURATION_10,
    I2C_CONFIGURATION_11,
    I2C_CONFIGURATION_12,
    I2C_CONFIGURATION_13,
    I2C_CONFIGURATION_14,
    I2C_CONFIGURATION_15,
    I2C_CONFIGURATION_16,
    I2C_CONFIGURATION_17,
    I2C_CONFIGURATION_18,
    I2C_CONFIGURATION_19,
    I2C_BACK_MODE,
} I2Cmsg;


typedef enum {
    STARTUP,
    NOT_HOMED,
    CURRENTLY_HOMING,
    HOMED,
    CHOOSE,
    PLATFORM,
    GIUNGLA,
    BOARDING,
    ESCAVATORE,
    AEROPLANO,
    PESCA,
    SCALATA,
    DIAMANTE,
    PLATFORM_READY,
    GIUNGLA_READY,
    BOARDING_READY,
    ESCAVATORE_READY,
    AEROPLANO_READY,
    PESCA_READY,
    SCALATA_READY,
    DIAMANTE_READY,
    READY,
    INITIALIZING,
    INITIALIZED,
    PHICUBE_ERROR,
} State;


typedef enum {
    CONFIGURATION_1,
    CONFIGURATION_2,
    CONFIGURATION_3,
    CONFIGURATION_4,
    CONFIGURATION_5,
    CONFIGURATION_6,
    CONFIGURATION_7,
    CONFIGURATION_8,
    CONFIGURATION_9,
    CONFIGURATION_10,
    CONFIGURATION_11,
    CONFIGURATION_12,
    CONFIGURATION_13,
    CONFIGURATION_14,
    CONFIGURATION_15,
    CONFIGURATION_16,
    CONFIGURATION_17,
    CONFIGURATION_18,
    CONFIGURATION_19,
    BACK_MODE,
    NO_MODE,
} Mode;


#endif // COMMUNICATION_H

