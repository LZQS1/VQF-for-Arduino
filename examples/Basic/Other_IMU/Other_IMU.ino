/* Example With Using Another IMU, Now using Adafruit MPU6050 as example */

#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <IMU.h>
#include <esp_task_wdt.h>

IMU imu;
IMU::imuEulerData data;
Adafruit_MPU6050 mpu;

void task(void *pvParameter) {
    esp_task_wdt_deinit();
    while (true) {
        rawIMUData raw;
        PreProcessedIMUData processed;

        sensors_event_t accel;
        sensors_event_t gyro;
        sensors_event_t temp;

        mpu.getEvent(&accel, &gyro, &temp);

        raw.ax = accel.acceleration.x;
        raw.ay = accel.acceleration.y;
        raw.az = accel.acceleration.z;

        raw.gx = gyro.gyro.x;
        raw.gy = gyro.gyro.y;
        raw.gz = gyro.gyro.z;

        raw.mx = 0;
        raw.my = 0;
        raw.mz = 0;

        imu.imuPreProcessData(raw, &processed);
        imu.imuReadEuler(processed, &data);
    }
}

void setup() {
    Serial.begin(115200);
    delay(1000);

    IMU::imuConfiguration cfg;
    cfg.useBuiltinICM20948 = false;
    cfg.rawDataIsPhysicalUnits = true;
    cfg.magEnable = false;
    imu.imuInit(cfg);

    if (!mpu.begin()) {
        Serial.println("MPU6050 not found !");
        while (1) {
            delay(100);
        }
    }
    mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
    mpu.setGyroRange(MPU6050_RANGE_2000_DEG);
    mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

    xTaskCreatePinnedToCore(task, "task", 6144, NULL, 10, NULL, 0);
}
uint32_t last = millis();
void loop() {
    if ((millis()-last) > 20) {
        Serial.printf("Roll:%.3f Pitch:%.3f Yaw:%.3f | AX:%.2f AY:%.2f AZ:%.2f | GX:%.3f GY:%.3f GZ:%.3f | MX:%.2f MY:%.2f MZ:%.2f | Bias:%.4f,%.4f,%.4f Resting:%d\n",
                        data.roll, data.pitch, data.yaw,
                        imu.ppData.ax, imu.ppData.ay, imu.ppData.az,
                        imu.ppData.gx, imu.ppData.gy, imu.ppData.gz,
                        imu.ppData.mx, imu.ppData.my, imu.ppData.mz,
                        data.biasX,data.biasY,data.biasZ, data.stationary.restDetected);
        last = millis();
    }
    delay(20);
}