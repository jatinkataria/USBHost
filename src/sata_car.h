
#ifndef SATA_CAR_H
#define SATA_CAR_H

#include <stdint.h>


#define SATA_ARB_ID 0x62f


typedef struct
{
    uint8_t cyl_num;       // which cyl to power
    uint8_t rpm;            // RPM speed of that specific cyl
    uint16_t pad0;
} SATA_CAR __attribute((packed));

#endif

