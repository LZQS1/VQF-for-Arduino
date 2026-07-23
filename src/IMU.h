#ifndef IMU_H
#define IMU_H

#include <Arduino.h>
#include <SPI.h>
#include <Preferences.h>
#include "vqf.hpp" // dlaidig/vqf

typedef struct {
    float ax;
    float ay;
    float az;
    float gx;
    float gy;
    float gz;
    float mx;
    float my;
    float mz;
} rawIMUData;

typedef struct {
    float ax;
    float ay;
    float az;
    float gx;
    float gy;
    float gz;
    float mx;
    float my;
    float mz;
} PreProcessedIMUData;

class IMU {
public:
    typedef struct {
        float pitch;
        float roll;
        float yaw;
        VQFState stationary;
        float biasX;
        float biasY;
        float biasZ;
    } imuEulerData;

    typedef struct {
        float gx;
        float gy;
        float gz;
        float accOx;
        float accOy;
        float accOz;
        float accSx;
        float accSy;
        float accSz;
    } nvsCalibData;

    typedef struct
    {
        uint8_t ACCEL_FS      = 2;
        uint8_t ACCEL_DLPF    = 0;
        uint8_t GYRO_FS       = 3;
        uint8_t GYRO_DLPF     = 0;
        uint8_t IMU_CS_PIN    = 1;
        uint8_t IMU_SCK_PIN   = SCK;
        uint8_t IMU_MISO_PIN  = MISO;
        uint8_t IMU_MOSI_PIN  = MOSI;
        uint32_t IMU_SPI_FREQ = 7000000UL;
        uint16_t IMU_SAMPLE_RATE_GYRO = 1000;
        uint16_t IMU_SAMPLE_RATE_ACCEL = 1000;
        uint16_t IMU_SAMPLE_RATE_MAG = 100;
        bool magEnable        = false;
        bool nvsEnable        = true;
        bool useBuiltinICM20948 = true;
        bool rawDataIsPhysicalUnits = false;
    } imuConfiguration;

    PreProcessedIMUData ppData;
    nvsCalibData nvsSaved;
    VQFParams vqfParams;
    VQF vqf;

    IMU();

    /**
    * @brief Initialize the VQF and ICM20948(if enable in imuConfiguration)
    * @return None
    */
    void imuInit(imuConfiguration &config);
    void imuInit(void);

    /**
    * @brief Get ICM20948's Raw Data(if enable in imuConfiguration)
    * @return None
    */
    void imuGetRawData(rawIMUData *result);

    /*
    * @brief Process the data likes
    */
    void imuPreProcessData(rawIMUData input, PreProcessedIMUData *result);
    void imuReadEuler(PreProcessedIMUData input, imuEulerData *result);
    void imuResetYaw(void);
    void imuCalibrateAccel(void);

private:
    imuConfiguration cfg;

    uint32_t lastNVSSaveInvertal = 0;
    uint32_t lastGyrAccUs = 0;
    uint32_t lastMagUs = 0;
    float yawOffset = 0.0f;
    bool yawZeroed = false;
    float lastYawRaw = 0.0f;
    float yawContinuous = 0.0f;
    bool yawInitialized = false;
    bool stationaryOffsetToNVS = false;

    Preferences prefs;
    nvsCalibData pendingBiasSave;

    void imuWrite(uint8_t reg, uint8_t data);
    uint8_t imuRead(uint8_t reg);
    void imuReadBytes(uint8_t reg, uint8_t *buf, uint8_t len);
    void imuBank(uint8_t bank);
    void imuReset(void);
    void imuWake(void);
    uint8_t imuWhoAmI(void);
    void imuMagStart(void);
    void imuMagInit(void);
    void quatToEuler(vqf_real_t *q, float &roll, float &pitch, float &yaw);
    float unwrapYaw(float newYawDeg);
};

#endif