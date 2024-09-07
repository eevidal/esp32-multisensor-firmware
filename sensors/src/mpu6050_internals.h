 
/**This register specifies the divider from the gyroscope output rate used to generate the Sample Rate
for the MPU-60X0.
The sensor register output, FIFO output, and DMP sampling are all based on the Sample Rate.
R/W */
#define SMPLRT_DIV   0x19

/**This register configures the external Frame Synchronization (FSYNC) pin sampling and the Digital
Low Pass Filter (DLPF) setting for both the gyroscopes and accelerometers.*/
#define CONFIG       0x1A
/**This register is used to trigger gyroscope self-test and configure the gyroscopes’ full scale range.*/
#define GYRO_CONFIG  0x1B

/**This register is used to trigger accelerometer self test and configure the accelerometer full scale
range. This register also configures the Digital High Pass Filter (DHPF).*/
#define ACCEL_CONFIG 0x1C
/**This register determines which sensor measurements are loaded into the FIFO buffer.*/
#define FIFO_EN      0x23

/**This register enables interrupt generation by interrupt sources.*/
#define INT_ENABLE   0x38
/**This register shows the interrupt status of each interrupt generation source. Each bit will clear after
the register is read*/
#define INT_STATUS   0x3A

/**Data Registers*/
/**The measurements are written to these registers at the Sample Rate as defined in Register
0x19.*/
/**These registers store the most recent accelerometer measurements*/ 
#define ACCEL_XOUT_H 0x3B
#define ACCEL_XOUT_L 0x3C
#define ACCEL_YOUT_H 0x3D
#define ACCEL_YOUT_L 0x3E
#define ACCEL_ZOUT_H 0x3F
#define ACCEL_ZOUT_L 0x40

/**These registers store the most recent temperature sensor measurement.*/
#define TEMP_OUT_H   0x41
#define TEMP_OUT_L   0x42

/**These registers store the most recent gyroscope measurements*/
#define GYRO_XOUT_H  0x43
#define GYRO_XOUT_L  0x44
#define GYRO_YOUT_H  0x45
#define GYRO_YOUT_L  0x46
#define GYRO_ZOUT_H  0x47
#define GYRO_ZOUT_L  0x48

/**This register allows the user to enable and disable the FIFO buffer, I2C Master Mode, and primary
I2C interface. The FIFO buffer, I2C Master, sensor signal paths and sensor registers can also be
reset using this register.*/
#define USER_CTRL    0x6A

/**This register allows the user to configure the power mode and clock source. It also provides a bit for
resetting the entire device, and a bit for disabling the temperature sensor.*/
#define PWR_MGMT_1   0x6B

/**This register allows the user to configure the frequency of wake-ups in Accelerometer Only Low
Power Mode. This register also allows the user to put individual axes of the accelerometer and
gyroscope into standby mode.*/
#define PWR_MGMT_2   0x6C

/**These registers keep track of the number of samples currently in the FIFO buffer.
These registers shadow the FIFO Count value. Both registers are loaded with the current sample
count when FIFO_COUNT_H (Register 72) is read*/
#define FIFO_COUNT_H 0x72
#define FIFO_COUNT_L 0x73

/**This register is used to read and write data from the FIFO buffer*/
#define FIFO_DATA    0x74

/**This register is used to verify the identity of the device. The default value of the register is 0x68.*/
#define WHO_AM_I     0x75

#define MPU6050_I2C_ADDRESS    0x68 
#define MAX_CLK      100000  //400KHz