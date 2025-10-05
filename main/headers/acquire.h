#ifndef ACQUIRE_H
#define ACQUIRE_H

#include "common.h"
//#include "bmp390.h"
#include "icm20948.h"
#include "icm20948_i2c.h"

#define G 9.80665
#define ICM_SCALE_16G 2048.0f
#define ICM_SCALE_2000DPS 16.4f


#define FLYING_THRESHOLD 10 * G // Acceleration threshold to consider rocket flying
#define CUTOFF_THRESHOLD 3 * G // Acceleration threshold to consider motor cutoff
#define LANDED_THRESHOLD 2     // Altitude threshold to consider rocket landed

#endif

void task_acquire(void *pvParameters);