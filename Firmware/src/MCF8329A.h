/*

MCF8329A Motor Controller IC I2C Driver

Author: Bernard Jin (bljin@mit.edu)
Created: 1 January 2025

*/

#include <stdint.h>

#ifndef MCF8329A
#define MCF8329A

/*

defines

*/

#define MCF8329A_I2C_ADDR           0x01
#define MCF8329A_READ_WRITE_ADDR    0x0000EA
#define MCF8329A_READ               0x40000000
#define MCF8329A_WRITE              0x8A500000
#define MCF8329A_SHADOW_MEM_SEC     0x00
#define MCF8329A_SHADOW_MEM_PAGE    0x00

/*

EEPROM shadow register addresses (p. 81)
note: when actually sending these, we only use the lowest 12 bits.

*/

#define MCF8329A_REG_ISD_CONFIG         0x000080
#define MCF8329A_REG_REV_DRIVE_CONFIG   0x000082
#define MCF8329A_REG_MOTOR_STARTUP1     0x000084
#define MCF8329A_REG_MOTOR_STARTUP2     0x000086
#define MCF8329A_REG_CLOSED_LOOP1       0x000088
#define MCF8329A_REG_CLOSED_LOOP2       0x00008A
#define MCF8329A_REG_CLOSED_LOOP3       0x00008C
#define MCF8329A_REG_CLOSED_LOOP4       0x00008E
#define MCF8329A_REG_FAULT_CONFIG1      0x000090
#define MCF8329A_REG_FAULT_CONFIG2      0x000092
#define MCF8329A_REG_SPEED_PROFILES1    0x000094
#define MCF8329A_REG_SPEED_PROFILES2    0x000096
#define MCF8329A_REG_SPEED_PROFILES3    0x000098
#define MCF8329A_REG_SPEED_PROFILES4    0x00009A
#define MCF8329A_REG_SPEED_PROFILES5    0x00009C
#define MCF8329A_REG_SPEED_PROFILES6    0x00009E
#define MCF8329A_REG_INT_ALGO_1         0x0000A0
#define MCF8329A_REG_INT_ALGO_2         0x0000A2
#define MCF8329A_REG_PIN_CONFIG1        0x0000A4
#define MCF8329A_REG_DEVICE_CONFIG1     0x0000A6
#define MCF8329A_REG_DEVICE_CONFIG2     0x0000A8
#define MCF8329A_REG_PERI_CONFIG2       0x0000AA
#define MCF8329A_REG_GD_CONFIG1         0x0000AC
#define MCF8329A_REG_GD_CONFIG2         0x0000AE

/* 

function stuff 

*/

struct MCF8329A_CONTROL_WORD {
    bool RW;
    bool CRC_EN;
    uint8_t DLEN;
    uint8_t MEM_SEC;
    uint8_t MEM_PAGE;
    uint16_t MEM_ADDR;
} ;

struct MCF8329A_PACKET {
    uint8_t controlWord[3];
    uint8_t data[4];
};

int assembleControlWord(MCF8329A_PACKET *, MCF8329A_CONTROL_WORD);

int assembleControlWord(MCF8329A_PACKET packet, MCF8329A_CONTROL_WORD word) {
    uint8_t out[3] = {0x00, 0x00, 0x00};
    if (word.RW) {
        out[0] = out[0] | 0x01 << 7;
    }
    if (word.CRC_EN) {
        out[0] = out[0] | 0x01 << 6;
    }
    out[0] = out[0] | word.DLEN << 4 | word.MEM_SEC;
    out[1] = word.MEM_SEC << 4 | (word.MEM_ADDR & 0xF00) >> 8;
    out[2] = word.MEM_ADDR & 0xFF;

    packet.controlWord[0] = out[0];
    packet.controlWord[1] = out[1];
    packet.controlWord[2] = out[2];

    return 0;
}

#endif