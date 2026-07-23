/* Example ICM20948 with SPI */

#include <IMU.h>
#include <esp_task_wdt.h>

IMU imu;
IMU::imuEulerData data;

void task(void *pvParameter) {
    esp_task_wdt_deinit();
    while (true) {
        rawIMUData raw;
        PreProcessedIMUData processed;
        imu.imuGetRawData(&raw);
        imu.imuPreProcessData(raw, &processed);
        imu.imuReadEuler(processed, &data);
    }
}

void setup() {
    Serial.begin(115200);
    delay(1000);
    imu.imuInit();

    // imu.imuCalibrateAccel(); // Uncomment When Need Accelerometer Calibration

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