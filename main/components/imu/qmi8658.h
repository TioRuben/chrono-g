#ifndef QMI8658_H
#define QMI8658_H

// I2C Configuration
#define QMI8658_I2C_ADDR_LOW    0x6A  // When SA0 is pulled high or left floating (default)
#define QMI8658_I2C_ADDR_HIGH   0x6B  // When SA0 is pulled low
#define QMI8658_I2C_SPEED_HZ    400000

// Device Identification
#define QMI8658_WHO_AM_I_VALUE  0x05
#define QMI8658_REVISION_ID     0x79

// Register Map
typedef enum {
    // General Purpose Registers
    QMI8658_REG_WHO_AM_I        = 0x00,
    QMI8658_REG_REVISION_ID     = 0x01,
    
    // Setup and Control Registers
    QMI8658_REG_CTRL1           = 0x02,
    QMI8658_REG_CTRL2           = 0x03,
    QMI8658_REG_CTRL3           = 0x04,
    QMI8658_REG_CTRL4           = 0x05,
    QMI8658_REG_CTRL5           = 0x06,
    QMI8658_REG_CTRL6           = 0x07,
    QMI8658_REG_CTRL7           = 0x08,
    QMI8658_REG_CTRL8           = 0x09,
    QMI8658_REG_CTRL9           = 0x0A,
    
    // Host Controlled Calibration Registers
    QMI8658_REG_CAL1_L          = 0x0B,
    QMI8658_REG_CAL1_H          = 0x0C,
    QMI8658_REG_CAL2_L          = 0x0D,
    QMI8658_REG_CAL2_H          = 0x0E,
    QMI8658_REG_CAL3_L          = 0x0F,
    QMI8658_REG_CAL3_H          = 0x10,
    QMI8658_REG_CAL4_L          = 0x11,
    QMI8658_REG_CAL4_H          = 0x12,
    
    // FIFO Registers
    QMI8658_REG_FIFO_WTM_TH     = 0x13,
    QMI8658_REG_FIFO_CTRL       = 0x14,
    QMI8658_REG_FIFO_SMPL_CNT   = 0x15,
    QMI8658_REG_FIFO_STATUS     = 0x16,
    QMI8658_REG_FIFO_DATA       = 0x17,
    
    // Status Registers
    QMI8658_REG_STATUSINT       = 0x2D,
    QMI8658_REG_STATUS0         = 0x2E,
    QMI8658_REG_STATUS1         = 0x2F,
    
    // Timestamp Registers
    QMI8658_REG_TIMESTAMP_LOW   = 0x30,
    QMI8658_REG_TIMESTAMP_MID   = 0x31,
    QMI8658_REG_TIMESTAMP_HIGH  = 0x32,
    
    // Data Output Registers
    QMI8658_REG_TEMP_L          = 0x33,
    QMI8658_REG_TEMP_H          = 0x34,
    QMI8658_REG_AX_L            = 0x35,
    QMI8658_REG_AX_H            = 0x36,
    QMI8658_REG_AY_L            = 0x37,
    QMI8658_REG_AY_H            = 0x38,
    QMI8658_REG_AZ_L            = 0x39,
    QMI8658_REG_AZ_H            = 0x3A,
    QMI8658_REG_GX_L            = 0x3B,
    QMI8658_REG_GX_H            = 0x3C,
    QMI8658_REG_GY_L            = 0x3D,
    QMI8658_REG_GY_H            = 0x3E,
    QMI8658_REG_GZ_L            = 0x3F,
    QMI8658_REG_GZ_H            = 0x40,
    
    // Quaternion Increment Registers
    QMI8658_REG_dQW_L           = 0x49,
    QMI8658_REG_dQW_H           = 0x4A,
    QMI8658_REG_dQX_L           = 0x4B,
    QMI8658_REG_dQX_H           = 0x4C,
    QMI8658_REG_dQY_L           = 0x4D,
    QMI8658_REG_dQY_H           = 0x4E,
    QMI8658_REG_dQZ_L           = 0x4F,
    QMI8658_REG_dQZ_H           = 0x50,
    
    // Velocity Increment Registers
    QMI8658_REG_dVX_L           = 0x51,
    QMI8658_REG_dVX_H           = 0x52,
    QMI8658_REG_dVY_L           = 0x53,
    QMI8658_REG_dVY_H           = 0x54,
    QMI8658_REG_dVZ_L           = 0x55,
    QMI8658_REG_dVZ_H           = 0x56,
    
    // AttitudeEngine Registers
    QMI8658_REG_AE_REG1         = 0x57,
    QMI8658_REG_AE_REG2         = 0x58,
    
    // Reset Register
    QMI8658_REG_RESET           = 0x60,
} qmi8658_reg_t;

// CTRL1 Register Bits
#define QMI8658_CTRL1_SIM           (1 << 7)  // 0: 4-wire SPI, 1: 3-wire SPI
#define QMI8658_CTRL1_ADDR_AI       (1 << 6)  // Address auto-increment
#define QMI8658_CTRL1_BE            (1 << 5)  // Big Endian
#define QMI8658_CTRL1_SENSOR_DISABLE (1 << 0) // Sensor disable

// CTRL2 - Accelerometer Settings
#define QMI8658_CTRL2_aST           (1 << 7)  // Accel Self Test

// Accelerometer Full Scale
typedef enum {
    QMI8658_ACC_RANGE_2G  = 0x00,
    QMI8658_ACC_RANGE_4G  = 0x10,
    QMI8658_ACC_RANGE_8G  = 0x20,
    QMI8658_ACC_RANGE_16G = 0x30,
} qmi8658_acc_range_t;

// Accelerometer ODR (when Accel only)
typedef enum {
    QMI8658_ACC_ODR_8000_HZ    = 0x00,  // High-res only
    QMI8658_ACC_ODR_4000_HZ    = 0x01,  // High-res only
    QMI8658_ACC_ODR_2000_HZ    = 0x02,  // High-res only
    QMI8658_ACC_ODR_1000_HZ    = 0x03,  // High-res
    QMI8658_ACC_ODR_500_HZ     = 0x04,  // High-res
    QMI8658_ACC_ODR_250_HZ     = 0x05,  // High-res
    QMI8658_ACC_ODR_125_HZ     = 0x06,  // High-res
    QMI8658_ACC_ODR_62_5_HZ    = 0x07,  // High-res
    QMI8658_ACC_ODR_31_25_HZ   = 0x08,  // High-res
    QMI8658_ACC_ODR_128_HZ_LP  = 0x0C,  // Low-power
    QMI8658_ACC_ODR_21_HZ_LP   = 0x0D,  // Low-power
    QMI8658_ACC_ODR_11_HZ_LP   = 0x0E,  // Low-power
    QMI8658_ACC_ODR_3_HZ_LP    = 0x0F,  // Low-power
} qmi8658_acc_odr_t;

// When 6DOF mode (Accel + Gyro enabled), ODR frequencies are:
// 7520, 3760, 1880, 940, 470, 235, 117.5, 58.75, 29.375 Hz

// CTRL3 - Gyroscope Settings
#define QMI8658_CTRL3_gST           (1 << 7)  // Gyro Self Test

// Gyroscope Full Scale
typedef enum {
    QMI8658_GYR_RANGE_16_DPS    = 0x00,
    QMI8658_GYR_RANGE_32_DPS    = 0x10,
    QMI8658_GYR_RANGE_64_DPS    = 0x20,
    QMI8658_GYR_RANGE_128_DPS   = 0x30,
    QMI8658_GYR_RANGE_256_DPS   = 0x40,
    QMI8658_GYR_RANGE_512_DPS   = 0x50,
    QMI8658_GYR_RANGE_1024_DPS  = 0x60,
    QMI8658_GYR_RANGE_2048_DPS  = 0x70,
} qmi8658_gyr_range_t;

// Gyroscope ODR
typedef enum {
    QMI8658_GYR_ODR_7520_HZ   = 0x00,
    QMI8658_GYR_ODR_3760_HZ   = 0x01,
    QMI8658_GYR_ODR_1880_HZ   = 0x02,
    QMI8658_GYR_ODR_940_HZ    = 0x03,
    QMI8658_GYR_ODR_470_HZ    = 0x04,
    QMI8658_GYR_ODR_235_HZ    = 0x05,
    QMI8658_GYR_ODR_117_5_HZ  = 0x06,
    QMI8658_GYR_ODR_58_75_HZ  = 0x07,
    QMI8658_GYR_ODR_29_375_HZ = 0x08,
} qmi8658_gyr_odr_t;

// CTRL5 - Low Pass Filter Settings
typedef enum {
    QMI8658_LPF_MODE_0 = 0x00,  // 2.66% of ODR
    QMI8658_LPF_MODE_1 = 0x01,  // 3.63% of ODR
    QMI8658_LPF_MODE_2 = 0x02,  // 5.39% of ODR
    QMI8658_LPF_MODE_3 = 0x03,  // 13.37% of ODR
} qmi8658_lpf_mode_t;

#define QMI8658_CTRL5_gLPF_EN       (1 << 4)  // Gyro LPF Enable
#define QMI8658_CTRL5_aLPF_EN       (1 << 0)  // Accel LPF Enable

// CTRL6 - AttitudeEngine Settings
#define QMI8658_CTRL6_sMoD          (1 << 7)  // Motion on Demand

// AttitudeEngine ODR
typedef enum {
    QMI8658_AE_ODR_1_HZ  = 0x00,
    QMI8658_AE_ODR_2_HZ  = 0x01,
    QMI8658_AE_ODR_4_HZ  = 0x02,
    QMI8658_AE_ODR_8_HZ  = 0x03,
    QMI8658_AE_ODR_16_HZ = 0x04,
    QMI8658_AE_ODR_32_HZ = 0x05,
    QMI8658_AE_ODR_64_HZ = 0x06,
} qmi8658_ae_odr_t;

// CTRL7 - Enable Sensors
#define QMI8658_CTRL7_syncSmpl      (1 << 7)  // Sync sample (locking mechanism)
#define QMI8658_CTRL7_sys_hs        (1 << 6)  // High-speed internal clock
#define QMI8658_CTRL7_gSN           (1 << 4)  // Gyro Snooze mode
#define QMI8658_CTRL7_sEN           (1 << 3)  // AttitudeEngine enable
#define QMI8658_CTRL7_gEN           (1 << 1)  // Gyroscope enable
#define QMI8658_CTRL7_aEN           (1 << 0)  // Accelerometer enable

// CTRL8 - Motion Detection Control
#define QMI8658_CTRL8_CTRL9_HOST_CMD    (1 << 7)  // CTRL9 handshake type
#define QMI8658_CTRL8_INT_MOTION        (1 << 6)  // Motion detection INT pin
#define QMI8658_CTRL8_PEDOMETER_EN      (1 << 4)
#define QMI8658_CTRL8_SIG_MOTION_EN     (1 << 3)
#define QMI8658_CTRL8_NO_MOTION_EN      (1 << 2)
#define QMI8658_CTRL8_ANY_MOTION_EN     (1 << 1)
#define QMI8658_CTRL8_TAP_EN            (1 << 0)

// CTRL9 Commands
typedef enum {
    QMI8658_CTRL9_CMD_NOP                       = 0x00,
    QMI8658_CTRL9_CMD_GYRO_BIAS                 = 0x01,
    QMI8658_CTRL9_CMD_REQ_SDI                   = 0x03,
    QMI8658_CTRL9_CMD_RST_FIFO                  = 0x04,
    QMI8658_CTRL9_CMD_REQ_FIFO                  = 0x05,
    QMI8658_CTRL9_CMD_WRITE_WOM_SETTING         = 0x08,
    QMI8658_CTRL9_CMD_ACCEL_HOST_DELTA_OFFSET   = 0x09,
    QMI8658_CTRL9_CMD_GYRO_HOST_DELTA_OFFSET    = 0x0A,
    QMI8658_CTRL9_CMD_COPY_USID                 = 0x10,
    QMI8658_CTRL9_CMD_SET_RPU                   = 0x11,
    QMI8658_CTRL9_CMD_AHB_CLOCK_GATING          = 0x12,
    QMI8658_CTRL9_CMD_ON_DEMAND_CALIBRATION     = 0xA2,
} qmi8658_ctrl9_cmd_t;

// STATUSINT Register Bits
#define QMI8658_STATUSINT_CTRL9_DONE    (1 << 7)
#define QMI8658_STATUSINT_LOCKED        (1 << 1)
#define QMI8658_STATUSINT_AVAIL         (1 << 0)

// STATUS0 Register Bits
#define QMI8658_STATUS0_sDA             (1 << 3)  // AE data available
#define QMI8658_STATUS0_gDA             (1 << 1)  // Gyro data available
#define QMI8658_STATUS0_aDA             (1 << 0)  // Accel data available

// STATUS1 Register Bits
#define QMI8658_STATUS1_SIG_MOTION      (1 << 7)
#define QMI8658_STATUS1_NO_MOTION       (1 << 6)
#define QMI8658_STATUS1_ANY_MOTION      (1 << 5)
#define QMI8658_STATUS1_PEDOMETER       (1 << 4)
#define QMI8658_STATUS1_TAP             (1 << 1)

// FIFO Control Bits
#define QMI8658_FIFO_CTRL_RD_MODE       (1 << 7)

// FIFO Size
typedef enum {
    QMI8658_FIFO_SIZE_16   = 0x00,
    QMI8658_FIFO_SIZE_32   = 0x04,
    QMI8658_FIFO_SIZE_64   = 0x08,
    QMI8658_FIFO_SIZE_128  = 0x0C,
} qmi8658_fifo_size_t;

// FIFO Mode
typedef enum {
    QMI8658_FIFO_MODE_BYPASS  = 0x00,
    QMI8658_FIFO_MODE_FIFO    = 0x01,
    QMI8658_FIFO_MODE_STREAM  = 0x02,
} qmi8658_fifo_mode_t;

// FIFO Status Bits
#define QMI8658_FIFO_STATUS_FULL        (1 << 7)
#define QMI8658_FIFO_STATUS_WTM         (1 << 6)
#define QMI8658_FIFO_STATUS_OVERFLOW    (1 << 5)
#define QMI8658_FIFO_STATUS_NOT_EMPTY   (1 << 4)

// Reset Value
#define QMI8658_RESET_VALUE             0xB0

// Sensitivity Values (LSB per unit)
#define QMI8658_ACC_SENSITIVITY_2G      16384   // LSB/g
#define QMI8658_ACC_SENSITIVITY_4G      8192
#define QMI8658_ACC_SENSITIVITY_8G      4096
#define QMI8658_ACC_SENSITIVITY_16G     2048

#define QMI8658_GYR_SENSITIVITY_16DPS   2048    // LSB/dps
#define QMI8658_GYR_SENSITIVITY_32DPS   1024
#define QMI8658_GYR_SENSITIVITY_64DPS   512
#define QMI8658_GYR_SENSITIVITY_128DPS  256
#define QMI8658_GYR_SENSITIVITY_256DPS  128
#define QMI8658_GYR_SENSITIVITY_512DPS  64
#define QMI8658_GYR_SENSITIVITY_1024DPS 32
#define QMI8658_GYR_SENSITIVITY_2048DPS 16

#define QMI8658_TEMP_SENSITIVITY        256     // LSB/°C

// Timing Constants
#define QMI8658_SYSTEM_STARTUP_TIME_MS  150
#define QMI8658_ACCEL_STARTUP_TIME_MS   3
#define QMI8658_GYRO_STARTUP_TIME_MS    60

#endif // QMI8658_H