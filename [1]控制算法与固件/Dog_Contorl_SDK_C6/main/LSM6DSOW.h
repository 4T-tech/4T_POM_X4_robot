#ifndef __LSM6DSOW_H__
#define __LSM6DSOW_H__

void lsm6dsow_init(void);
void lsm6dsow_read_accel_gyro(int16_t accel[3], int16_t gyro[3]);

#endif  // __LSM6DSOW_H__