// IMU.cpp

#include "VQF_Arduino.h"

SPIClass imuSpi(FSPI);

IMU::IMU() : vqf(vqfParams, 0.001, 0.01) {}

void IMU::imuInit(imuConfiguration &config) {
    cfg = config;

    if (cfg.useBuiltinICM20948) {
        pinMode(cfg.IMU_CS_PIN, OUTPUT);
        digitalWrite(cfg.IMU_CS_PIN, HIGH);
        imuSpi.begin(cfg.IMU_SCK_PIN, cfg.IMU_MISO_PIN, cfg.IMU_MOSI_PIN, -1);
        if (imuWhoAmI() != 0xEA) {
            Serial.println("ICM20948 WhoAmI failed! Initialize interrupted!");
            return;
        }

        imuReset();
        imuWake();
        imuWrite(0x03, 0x30);
        imuBank(2);

        uint8_t gyro_cfg = ((cfg.GYRO_DLPF & 0x07) << 3) | ((cfg.GYRO_FS   & 0x03) << 1) | 0x01;
        imuWrite(0x01, gyro_cfg);
        uint8_t accel_cfg = ((cfg.ACCEL_DLPF & 0x07) << 3) | ((cfg.ACCEL_FS   & 0x03) << 1) | 0x01;
        imuWrite(0x14, accel_cfg);

        imuWrite(0x00, 0x00);
        imuWrite(0x10, 0x00);
        imuWrite(0x11, 0x00);

        imuMagStart();
        imuMagInit();

        imuBank(0);
        Serial.println("ICM20948 inited");
        imuBank(2);
        uint8_t rcfg = imuRead(0x01);
        Serial.printf("GYRO_CONFIG_1=%02X\n",rcfg);
        imuBank(0);
    }

    prefs.begin("imu", true);
    nvsSaved.gx    = prefs.getFloat("gx", 0.0f);
    nvsSaved.gy    = prefs.getFloat("gy", 0.0f);
    nvsSaved.gz    = prefs.getFloat("gz", 0.0f);
    nvsSaved.accOx = prefs.getFloat("accOx", 0.0f);
    nvsSaved.accOy = prefs.getFloat("accOy", 0.0f);
    nvsSaved.accOz = prefs.getFloat("accOz", 0.0f);
    nvsSaved.accSx = prefs.getFloat("accSx", 1.0f);
    nvsSaved.accSy = prefs.getFloat("accSy", 1.0f);
    nvsSaved.accSz = prefs.getFloat("accSz", 1.0f);
    prefs.end();

    pendingBiasSave = nvsSaved;

    Serial.printf("Gyro Offset From NVS: gx:%.4f gy:%.4f gz:%.4f\n", nvsSaved.gx, nvsSaved.gy, nvsSaved.gz);
    Serial.printf("Accel Calib From NVS: ox:%.6f oy:%.6f oz:%.6f sx:%.6f sy:%.6f sz:%.6f\n",
                  nvsSaved.accOx, nvsSaved.accOy, nvsSaved.accOz, nvsSaved.accSx, nvsSaved.accSy, nvsSaved.accSz);

    lastGyrAccUs = micros();
    lastMagUs = micros();
}
void IMU::imuInit(void) {
    imuConfiguration defaultConfig;
    imuInit(defaultConfig);
}

void IMU::imuGetRawData(rawIMUData *result) {
    rawIMUData temp = {0};
    uint8_t buf[18];
    uint8_t mag[9];
    imuReadBytes(0x2D,buf,12);
    imuReadBytes(0x3B, mag, 9);

    int16_t ax=(buf[0]<<8)|buf[1];
    int16_t ay=(buf[2]<<8)|buf[3];
    int16_t az=(buf[4]<<8)|buf[5];

    int16_t gx=(buf[6]<<8)|buf[7];
    int16_t gy=(buf[8]<<8)|buf[9];
    int16_t gz=(buf[10]<<8)|buf[11];

    int16_t mx = 0, my = 0, mz = 0;
    if (!(mag[8] & 0x08)) {
        mx = (mag[2] << 8) | mag[1];
        my = (mag[4] << 8) | mag[3];
        mz = (mag[6] << 8) | mag[5];
    }
    
    temp.ax = ax;
    temp.ay = ay;
    temp.az = az;

    temp.gx = gx;
    temp.gy = gy;
    temp.gz = gz;

    temp.mx = mx;
    temp.my = my;
    temp.mz = mz;

    *result = temp;
}

void IMU::imuPreProcessData(rawIMUData input, PreProcessedIMUData *result) {
    PreProcessedIMUData temp;

    if (cfg.rawDataIsPhysicalUnits) {
        temp.ax = input.ax;
        temp.ay = input.ay;
        temp.az = input.az;

        temp.gx = input.gx;
        temp.gy = input.gy;
        temp.gz = input.gz;

        temp.mx = input.mx;
        temp.my = input.my;
        temp.mz = input.mz;

        if (cfg.nvsEnable) {
            temp.ax = (temp.ax - nvsSaved.accOx) * nvsSaved.accSx;
            temp.ay = (temp.ay - nvsSaved.accOy) * nvsSaved.accSy;
            temp.az = (temp.az - nvsSaved.accOz) * nvsSaved.accSz;

            temp.gx += nvsSaved.gx;
            temp.gy += nvsSaved.gy;
            temp.gz += nvsSaved.gz;
        }

        *result = temp;
        ppData = temp;
        return;
    }

    float gyroScale = 0.0f;
    float accelScale = 0.0f;
    float magScale = 0.15f;

    int8_t gxSign = 1, gySign = 1, gzSign = 1;

    if (cfg.GYRO_FS == 0) gyroScale = 131.0f;
    else if (cfg.GYRO_FS == 1) gyroScale = 65.5f;
    else if (cfg.GYRO_FS == 2) gyroScale = 32.8f;
    else if (cfg.GYRO_FS == 3)gyroScale = 16.4f;

    if (cfg.ACCEL_FS == 0) accelScale = 16384.0f;
    else if (cfg.ACCEL_FS == 1) accelScale = 8192.0f;
    else if (cfg.ACCEL_FS == 2) accelScale = 4096.0f;
    else if (cfg.ACCEL_FS == 3)accelScale = 2048.0f;

    float ax_g, ay_g, az_g;

    if (cfg.nvsEnable) {
        ax_g = ((input.ax / accelScale) - nvsSaved.accOx) * nvsSaved.accSx * 9.80665f;
        ay_g = ((input.ay / accelScale) - nvsSaved.accOy) * nvsSaved.accSy * 9.80665f;
        az_g = ((input.az / accelScale) - nvsSaved.accOz) * nvsSaved.accSz * 9.80665f;
    } else {
        ax_g = (input.ax / accelScale) * 9.80665f;
        ay_g = (input.ay / accelScale) * 9.80665f;
        az_g = (input.az / accelScale) * 9.80665f;
    }

    float gx_dps = (input.gx / gyroScale) * DEG_TO_RAD;
    float gy_dps = (input.gy / gyroScale) * DEG_TO_RAD;
    float gz_dps = (input.gz / gyroScale) * DEG_TO_RAD;

    gx_dps = (gxSign * gx_dps) - (0.0630f * gxSign);
    gy_dps = (gySign * gy_dps) - (0.0110f * gySign);
    gz_dps = (gzSign * gz_dps) - (0.0056f * gzSign);

    if (cfg.nvsEnable) {
        gx_dps += (nvsSaved.gx * gxSign);
        gy_dps += (nvsSaved.gy * gySign);
        gz_dps += (nvsSaved.gz * gzSign);
    }

    temp.ax = ax_g;
    temp.ay = ay_g;
    temp.az = az_g;

    temp.gx = gx_dps;
    temp.gy = gy_dps;
    temp.gz = gz_dps;

    temp.mx = input.mx * magScale;
    temp.my = input.my * magScale;
    temp.mz = input.mz * magScale;

    *result = temp;
    ppData = temp;
}

void IMU::imuReadEuler(PreProcessedIMUData input, imuEulerData *result) {
    uint32_t now = micros();
    uint32_t gyrAccIntervalUs = 1000000UL / cfg.IMU_SAMPLE_RATE_GYRO;

    if (now - lastGyrAccUs >= gyrAccIntervalUs) {
        lastGyrAccUs = now;

        vqf_real_t gyr[3] = { input.gx, input.gy, input.gz };
        vqf_real_t acc[3] = { input.ax, input.ay, input.az };
        vqf_real_t mag[3] = { input.mx, input.my, input.mz };
        vqf.updateGyr(gyr);
        vqf.updateAcc(acc);

        if (cfg.magEnable) {
            vqf.updateMag(mag);
        }

        vqf_real_t quat[4];
        vqf.getQuat9D(quat);

        float roll, pitch, yaw;
        this->quatToEuler(quat, roll, pitch, yaw);

        float yawUnwrapped = (this->unwrapYaw(yaw));

        if (!yawZeroed) {
            yawOffset = yawUnwrapped;
            yawZeroed = true;
        }

        float yawRelative = -(yawUnwrapped - yawOffset);

        VQFState state = vqf.getState();
        vqf_real_t bias[3];
        vqf.getBiasEstimate(bias);

        if (state.restDetected && cfg.nvsEnable) {
            if (((now - lastNVSSaveInvertal) > 17.5 * 1000 * 1000) && !stationaryOffsetToNVS) {
                stationaryOffsetToNVS = true;

                pendingBiasSave.gx = nvsSaved.gx - bias[0];
                pendingBiasSave.gy = nvsSaved.gy - bias[1];
                pendingBiasSave.gz = nvsSaved.gz - bias[2];

                prefs.begin("imu", false);
                prefs.putFloat("gx", pendingBiasSave.gx);
                prefs.putFloat("gy", pendingBiasSave.gy);
                prefs.putFloat("gz", pendingBiasSave.gz);
                prefs.end();
                Serial.println("Saved");
            }
        }
        else {
            stationaryOffsetToNVS = false;
            lastNVSSaveInvertal = now;
        }

        result->pitch      = pitch;
        result->roll       = roll;
        result->yaw        = yawRelative;
        result->stationary = state;
        result->biasX      = bias[0];
        result->biasY      = bias[1];
        result->biasZ      = bias[2];
    }
}

void IMU::imuResetYaw(void) {
    yawOffset = yawContinuous;
}

void IMU::imuCalibrateAccel(void) {
    if (!cfg.useBuiltinICM20948) {
        Serial.println("imuCalibrateAccel only supports the built-in ICM20948 SPI path.");
        return;
    }

    const char* axisPrompt[6] = {
        "+X (right side up)",
        "-X (left side up)",
        "+Y (front side up)",
        "-Y (back side up)",
        "+Z (top side up)",
        "-Z (bottom side up)"
    };

    float expected[6][3] = {
        { 1, 0, 0},
        {-1, 0, 0},
        { 0, 1, 0},
        { 0,-1, 0},
        { 0, 0, 1},
        { 0, 0,-1}
    };

    float measured[6][3];

    Serial.println("=== Accelerometer Calibration Start ===");
    Serial.println("Type 'next' in Serial to Continue");

    for (int i = 0; i < 6; i++) {
        Serial.printf("\n[%d/6] Place IMU to: %s\n", i + 1, axisPrompt[i]);
        Serial.println("Type 'next' to continue");

        while (Serial.available()) Serial.read();
        while (true) {
            if (Serial.available()) {
                String cmd = Serial.readStringUntil('\n');
                cmd.trim();
                cmd.toLowerCase();
                if (cmd == "next") {
                    break;
                } else if (cmd.length() > 0) {
                    Serial.println("Invalid Command");
                }
            }
            delay(10);
        }

        delay(300);

        const int N = 200;
        float sx = 0, sy = 0, sz = 0;

        for (int s = 0; s < N; s++) {
            rawIMUData raw;
            imuGetRawData(&raw);

            float accelScale = 0.0f;
            if (cfg.ACCEL_FS == 0) accelScale = 16384.0f;
            else if (cfg.ACCEL_FS == 1) accelScale = 8192.0f;
            else if (cfg.ACCEL_FS == 2) accelScale = 4096.0f;
            else if (cfg.ACCEL_FS == 3) accelScale = 2048.0f;
            float ax = (raw.ax / accelScale);
            float ay = (raw.ay / accelScale);
            float az = (raw.az / accelScale);

            sx += ax;
            sy += ay;
            sz += az;
            delay(5);
        }

        measured[i][0] = sx / N;
        measured[i][1] = sy / N;
        measured[i][2] = sz / N;

        Serial.printf("Final Result: X=%.4f Y=%.4f Z=%.4f\n",
                    measured[i][0], measured[i][1], measured[i][2]);
    }

    float offset[3];
    float scale[3];

    for (int axis = 0; axis < 3; axis++) {
        float posVal = measured[axis * 2][axis];
        float negVal = measured[axis * 2 + 1][axis];

        offset[axis] = (posVal + negVal) / 2.0f;
        scale[axis]  = 2.0f / (posVal - negVal);
    }

    Serial.println("\n=== Calibrated Result ===");
    Serial.printf("Offset: X=%.6f  Y=%.6f  Z=%.6f\n", offset[0], offset[1], offset[2]);
    Serial.printf("Scale : X=%.6f  Y=%.6f  Z=%.6f\n", scale[0], scale[1], scale[2]);

    nvsSaved.accOx = offset[0];
    nvsSaved.accOy = offset[1];
    nvsSaved.accOz = offset[2];
    nvsSaved.accSx = scale[0];
    nvsSaved.accSy = scale[1];
    nvsSaved.accSz = scale[2];

    prefs.begin("imu", false);
    prefs.putFloat("accOx", nvsSaved.accOx);
    prefs.putFloat("accOy", nvsSaved.accOy);
    prefs.putFloat("accOz", nvsSaved.accOz);
    prefs.putFloat("accSx", nvsSaved.accSx);
    prefs.putFloat("accSy", nvsSaved.accSy);
    prefs.putFloat("accSz", nvsSaved.accSz);
    prefs.end();

    Serial.println("\nAccel calibration data saved to NVS.");
    Serial.println("Calibration Ended. Please Remove imuCalibrationAccel and upload program !");

    while (true) {
        delay(1000);
    }
}

void IMU::imuWrite(uint8_t reg, uint8_t data) {
    imuSpi.beginTransaction(SPISettings(cfg.IMU_SPI_FREQ, MSBFIRST, SPI_MODE0));
    digitalWrite(cfg.IMU_CS_PIN, LOW);
    imuSpi.transfer(reg & 0x7F);
    imuSpi.transfer(data);
    digitalWrite(cfg.IMU_CS_PIN, HIGH);
    imuSpi.endTransaction();
}

uint8_t IMU::imuRead(uint8_t reg) {
    imuSpi.beginTransaction(SPISettings(cfg.IMU_SPI_FREQ, MSBFIRST, SPI_MODE0));
    digitalWrite(cfg.IMU_CS_PIN, LOW);
    imuSpi.transfer(reg | 0x80);
    uint8_t v = imuSpi.transfer(0x00);
    digitalWrite(cfg.IMU_CS_PIN, HIGH);
    imuSpi.endTransaction();
    return v;
}

void IMU::imuReadBytes(uint8_t reg, uint8_t *buf, uint8_t len) {
    imuSpi.beginTransaction(SPISettings(cfg.IMU_SPI_FREQ, MSBFIRST, SPI_MODE0));
    digitalWrite(cfg.IMU_CS_PIN, LOW);
    imuSpi.transfer(reg | 0x80);

    while (len--)
        *buf++ = imuSpi.transfer(0);

    digitalWrite(cfg.IMU_CS_PIN, HIGH);
    imuSpi.endTransaction();
}

void IMU::imuBank(uint8_t bank) {
    imuWrite(0x7F, bank << 4);
}

void IMU::imuReset(void) {
    imuBank(0);
    imuWrite(0x06,0x80);
    delay(100);
}

void IMU::imuWake(void) {
    imuWrite(0x06,0x01);
    delay(10);
}

uint8_t IMU::imuWhoAmI(void) {
    uint8_t id = imuRead(0x00);
    return id;
}

void IMU::imuMagStart(void) {
    imuBank(3);
    imuWrite(0x14, 0x32);
    imuWrite(0x16, 0x01);
    imuWrite(0x15, 0x80 | 1);
    delay(10);

    imuBank(3);
    imuWrite(0x13,0x0C);
    imuWrite(0x14,0x31);
    imuWrite(0x16,0x08);
    imuWrite(0x15,0x80 | 1);
    delay(10);
    
    imuBank(0);
}

void IMU::imuMagInit(void) {
    imuBank(3);

    imuWrite(0x01,0x07);
    imuWrite(0x03,0x8C);
    imuWrite(0x04,0x10);
    imuWrite(0x05,0x89);

    imuBank(0);
}

void IMU::quatToEuler(vqf_real_t *q, float &roll, float &pitch, float &yaw) {
    float w = q[0], x = q[1], y = q[2], z = q[3];

    float sinr_cosp = 2.0f * (w * x + y * z);
    float cosr_cosp = 1.0f - 2.0f * (x * x + y * y);
    roll = atan2(sinr_cosp, cosr_cosp) * RAD_TO_DEG;

    float sinp = 2.0f * (w * y - z * x);
    if (fabs(sinp) >= 1.0f)
        pitch = copysign(90.0f, sinp);
    else
        pitch = asin(sinp) * RAD_TO_DEG;

    float siny_cosp = 2.0f * (w * z + x * y);
    float cosy_cosp = 1.0f - 2.0f * (y * y + z * z);
    yaw = atan2(siny_cosp, cosy_cosp) * RAD_TO_DEG;
}

float IMU::unwrapYaw(float newYawDeg) {
    if (!yawInitialized) {
        lastYawRaw = newYawDeg;
        yawContinuous = newYawDeg;
        yawInitialized = true;
        return yawContinuous;
    }

    float delta = newYawDeg - lastYawRaw;

    if (delta > 180.0f) {
        delta -= 360.0f;
    }
    else if (delta < -180.0f) {
        delta += 360.0f;
    }

    yawContinuous += delta;
    lastYawRaw = newYawDeg;
    return yawContinuous;
}