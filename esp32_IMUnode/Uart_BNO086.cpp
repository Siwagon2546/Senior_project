#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_BNO08x.h>
#include <Adafruit_BMP280.h>

#define BNO08X_RX_PIN    16
#define BNO08X_TX_PIN    17
#define BNO08X_RESET_PIN 4

Adafruit_BMP280    bmp;
Adafruit_BNO08x    bno08x(BNO08X_RESET_PIN);

// ==========================================
// Binary payload structs
// ==========================================
#pragma pack(push, 1)
struct ImuPayload {
    uint8_t header1  = 0xAA;
    uint8_t header2  = 0xBB;
    uint8_t type     = 0x01;
    float ax, ay, az;
    float gx, gy, gz;
    float qx, qy, qz, qw;
    uint8_t checksum;
    // รวม: 3 + 10*4 + 1 = 44 bytes
};

struct BmpPayload {
    uint8_t header1  = 0xAA;
    uint8_t header2  = 0xBB;
    uint8_t type     = 0x02;
    float temp;
    float pressure;
    uint8_t checksum;
    // รวม: 3 + 2*4 + 1 = 12 bytes
};
#pragma pack(pop)

// ==========================================
// Shared data
// ==========================================
ImuPayload sharedImuData;
BmpPayload sharedBmpData;
bool       newBmpDataFlag = false;

// ✅ แก้ไข: ใช้ FreeRTOS Mutex แทน portMUX
//    portMUX ปิด interrupt ทั้ง chip เหมาะกับ ISR เท่านั้น
//    SemaphoreHandle_t ทำ context-switch ได้ ไม่บล็อก Core อื่น
SemaphoreHandle_t dataMutex;

// ==========================================
// Helper: คำนวณ XOR checksum ตั้งแต่ byte[2] จนถึง byte[len-2]
// ==========================================
static uint8_t calculateChecksum(uint8_t* ptr, size_t length) {
    uint8_t sum = 0;
    for (size_t i = 2; i < length - 1; i++) sum ^= ptr[i];
    return sum;
}

// ==========================================
// Helper: reinit BNO086 (ใช้ตอน recovery)
// ==========================================
static bool reinitBNO() {
    if (!bno08x.begin_UART(&Serial2)) return false;
    bno08x.enableReport(SH2_LINEAR_ACCELERATION,  5000);
    bno08x.enableReport(SH2_GYROSCOPE_CALIBRATED, 5000);
    bno08x.enableReport(SH2_ROTATION_VECTOR,      5000);
    return true;
}

// ==========================================
// TASK 1: อ่าน IMU (Core 1, Priority สูง)
// ==========================================
void ImuReadTask(void *pvParameters) {
    sh2_SensorValue_t sensorValue;

    // นับ tick ที่ไม่มีข้อมูล เพื่อตรวจ sensor หลุด
    int noDataCount = 0;

    while (1) {
        bool gotData = false;

        // ✅ แก้ไข: อ่าน UART *ข้างนอก* lock ทั้งหมด
        //    getSensorEvent() ใช้เวลาอ่าน UART หลาย microsecond
        //    ถ้าครอบด้วย lock จะบล็อก SerialTxTask บน Core 0
        while (bno08x.getSensorEvent(&sensorValue)) {
            gotData = true;
            noDataCount = 0;

            // อ่านค่าออกมา local ก่อน (ยังไม่ต้อง lock)
            switch (sensorValue.sensorId) {

                case SH2_LINEAR_ACCELERATION: {
                    float ax = sensorValue.un.linearAcceleration.x;
                    float ay = sensorValue.un.linearAcceleration.y;
                    float az = sensorValue.un.linearAcceleration.z;
                    // ✅ lock แค่ตอน copy เข้า shared struct (สั้นมาก)
                    if (xSemaphoreTake(dataMutex, pdMS_TO_TICKS(2)) == pdTRUE) {
                        sharedImuData.ax = ax;
                        sharedImuData.ay = ay;
                        sharedImuData.az = az;
                        xSemaphoreGive(dataMutex);
                    }
                    break;
                }

                case SH2_GYROSCOPE_CALIBRATED: {
                    float gx = sensorValue.un.gyroscope.x;
                    float gy = sensorValue.un.gyroscope.y;
                    float gz = sensorValue.un.gyroscope.z;
                    if (xSemaphoreTake(dataMutex, pdMS_TO_TICKS(2)) == pdTRUE) {
                        sharedImuData.gx = gx;
                        sharedImuData.gy = gy;
                        sharedImuData.gz = gz;
                        xSemaphoreGive(dataMutex);
                    }
                    break;
                }

                case SH2_ROTATION_VECTOR: {
                    float qx = sensorValue.un.rotationVector.i;
                    float qy = sensorValue.un.rotationVector.j;
                    float qz = sensorValue.un.rotationVector.k;
                    float qw = sensorValue.un.rotationVector.real;
                    if (xSemaphoreTake(dataMutex, pdMS_TO_TICKS(2)) == pdTRUE) {
                        sharedImuData.qx = qx;
                        sharedImuData.qy = qy;
                        sharedImuData.qz = qz;
                        sharedImuData.qw = qw;
                        xSemaphoreGive(dataMutex);
                    }
                    break;
                }
            }
        }

        if (!gotData) noDataCount++;

        // ✅ แก้ไข: Recovery — ถ้าไม่มีข้อมูล ~500ms ให้ reinit BNO086
        //    แทนการ ESP.restart() ทั้ง chip
        if (noDataCount > 500) {
            noDataCount = 0;
            Serial.println("DEBUG,BNO_TIMEOUT_REINIT");
            reinitBNO();
            vTaskDelay(pdMS_TO_TICKS(100));
        }

        vTaskDelay(pdMS_TO_TICKS(1));
    }
}

// ==========================================
// TASK 2: อ่าน BMP280 (Core 1, Priority ต่ำ)
// ==========================================
void BmpReadTask(void *pvParameters) {
    TickType_t xLastWakeTime = xTaskGetTickCount();

    while (1) {
        // อ่าน I2C ข้างนอก lock (blocking I/O ไม่ควรอยู่ใน critical section)
        float t = bmp.readTemperature();
        float p = bmp.readPressure() / 100.0F;

        if (xSemaphoreTake(dataMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
            sharedBmpData.temp     = t;
            sharedBmpData.pressure = p;
            newBmpDataFlag         = true;
            xSemaphoreGive(dataMutex);
        }

        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(1000));
    }
}

// ==========================================
// TASK 3: ส่งข้อมูลผ่าน Serial (Core 0, 200 Hz)
// ==========================================
void SerialTxTask(void *pvParameters) {
    TickType_t       xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency    = pdMS_TO_TICKS(5); // 200 Hz

    ImuPayload localImu;
    BmpPayload localBmp;
    bool       sendBmp  = false;

    // Debug counter — พิมพ์ทุก 1 วินาทีเพื่อยืนยันว่า Task ยังทำงานอยู่
    uint32_t txCount = 0;

    while (1) {
        txCount++;

        // ก๊อปปี้ข้อมูลออกมาอย่างรวดเร็ว
        if (xSemaphoreTake(dataMutex, pdMS_TO_TICKS(2)) == pdTRUE) {
            localImu = sharedImuData;
            if (newBmpDataFlag) {
                localBmp      = sharedBmpData;
                sendBmp       = true;
                newBmpDataFlag = false;
            }
            xSemaphoreGive(dataMutex);
        }

        // คำนวณ checksum และส่ง IMU
        localImu.checksum = calculateChecksum((uint8_t*)&localImu, sizeof(ImuPayload));
        Serial.write((uint8_t*)&localImu, sizeof(ImuPayload));

        // ส่ง BMP ถ้าครบรอบ 1 วินาที
        if (sendBmp) {
            localBmp.checksum = calculateChecksum((uint8_t*)&localBmp, sizeof(BmpPayload));
            Serial.write((uint8_t*)&localBmp, sizeof(BmpPayload));
            sendBmp = false;
        }

        // Debug heartbeat (ปิดได้ถ้า production)
        if (txCount % 200 == 0) {
            Serial.printf("DEBUG,TX_ALIVE,%lu\n", txCount);
        }

        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}

// ==========================================
// SETUP
// ==========================================
void setup() {
    Serial.setTxBufferSize(1024);
    Serial.begin(921600);
    delay(500);

    Serial.println("DEBUG,BOOTING");

    // ✅ แก้ไข: สร้าง Mutex ก่อน Task ทั้งหมด
    dataMutex = xSemaphoreCreateMutex();
    if (dataMutex == NULL) {
        Serial.println("DEBUG,MUTEX_FAIL");
        ESP.restart();
    }

    // Init I2C และ BMP280
    Wire.begin(21, 22);
    if (!bmp.begin(0x76)) {
        Serial.println("DEBUG,BMP_FAILED");
        // ไม่ restart — ยังส่ง IMU ได้แม้ BMP เสีย
    }

    // Init Serial2 สำหรับ BNO086
    Serial2.setRxBufferSize(2048);
    Serial2.begin(3000000, SERIAL_8N1, BNO08X_RX_PIN, BNO08X_TX_PIN);

    Serial.println("DEBUG,INIT_BNO08X...");

    int retry = 0;
    while (!reinitBNO()) {
        Serial.println("DEBUG,BNO08X_UART_RETRY");
        delay(500);
        if (++retry > 10) {
            Serial.println("DEBUG,BNO08X_FAIL_RESTART");
            ESP.restart();
        }
    }
    Serial.println("DEBUG,BNO08X_OK");

    // สร้าง Tasks
    // Core 1: เซ็นเซอร์
    xTaskCreatePinnedToCore(ImuReadTask, "ImuTask", 4096, NULL, 3, NULL, 1);
    xTaskCreatePinnedToCore(BmpReadTask, "BmpTask", 2048, NULL, 1, NULL, 1);

    // Core 0: ส่งข้อมูล
    xTaskCreatePinnedToCore(SerialTxTask, "TxTask", 4096, NULL, 4, NULL, 0);

    // ลบ Task loop() หลัก
    vTaskDelete(NULL);
}

void loop() {
    // RTOS จัดการทั้งหมด — ไม่ใช้ loop()
}