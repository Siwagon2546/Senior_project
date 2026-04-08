#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_BNO08x.h>
#include <Adafruit_BMP280.h>


#define BNO08X_RX_PIN    16
#define BNO08X_TX_PIN    17
#define BNO08X_RESET_PIN 4

Adafruit_BMP280 bmp;
Adafruit_BNO08x bno08x(BNO08X_RESET_PIN);
const int WDT_TIMEOUT_SECONDS = 3;

// --- 🌟 โครงสร้าง Binary Payload ---
#pragma pack(push, 1)
struct ImuPayload {
    uint8_t header1 = 0xAA;
    uint8_t header2 = 0xBB;
    uint8_t type = 0x01;
    float ax, ay, az;
    float gx, gy, gz;
    float qx, qy, qz, qw;
    uint8_t checksum;
};

struct BmpPayload {
    uint8_t header1 = 0xAA;
    uint8_t header2 = 0xBB;
    uint8_t type = 0x02;
    float temp;
    float pressure;
    uint8_t checksum;
};
#pragma pack(pop)

// ตัวแปร Global สำหรับแชร์ระหว่าง Task
ImuPayload sharedImuData;
BmpPayload sharedBmpData;
bool newBmpDataFlag = false;

// 🔒 Spinlock Mutex สำหรับป้องกัน Core 0 และ Core 1 แย่งกันอ่าน/เขียนข้อมูล
portMUX_TYPE dataMux = portMUX_INITIALIZER_UNLOCKED;

uint8_t calculateChecksum(uint8_t* ptr, size_t length) {
    uint8_t sum = 0;
    for (size_t i = 2; i < length - 1; i++) sum ^= ptr[i];
    return sum;
}

// ========================================================
// 🛠️ TASK 1: อ่าน IMU (รันบน Core 1, Priority สูง)
// ========================================================
void ImuReadTask(void *pvParameters) {
    sh2_SensorValue_t sensorValue;
    
    // เอา Task นี้เข้าสู่ระบบ Watchdog

    while (1) {

        // กวาดข้อมูลจาก UART Buffer ให้เกลี้ยง
        while (bno08x.getSensorEvent(&sensorValue)) {
            // ล็อก Mutex ก่อนอัปเดตค่า ป้องกัน Core 0 ดึงไปตอนเขียนยังไม่เสร็จ
            portENTER_CRITICAL(&dataMux);
            switch (sensorValue.sensorId) {
                case SH2_LINEAR_ACCELERATION:
                    sharedImuData.ax = sensorValue.un.linearAcceleration.x;
                    sharedImuData.ay = sensorValue.un.linearAcceleration.y;
                    sharedImuData.az = sensorValue.un.linearAcceleration.z;
                    break;
                case SH2_GYROSCOPE_CALIBRATED:
                    sharedImuData.gx = sensorValue.un.gyroscope.x;
                    sharedImuData.gy = sensorValue.un.gyroscope.y;
                    sharedImuData.gz = sensorValue.un.gyroscope.z;
                    break;
                case SH2_ROTATION_VECTOR:
                    sharedImuData.qx = sensorValue.un.rotationVector.i;
                    sharedImuData.qy = sensorValue.un.rotationVector.j;
                    sharedImuData.qz = sensorValue.un.rotationVector.k;
                    sharedImuData.qw = sensorValue.un.rotationVector.real;
                    break;
            }
            portEXIT_CRITICAL(&dataMux); // ปลดล็อก
        }
        // พัก Task 1ms ให้ CPU หายใจ
        vTaskDelay(pdMS_TO_TICKS(1)); 
    }
}

// ========================================================
// 🛠️ TASK 2: อ่าน BMP280 (รันบน Core 1, Priority ต่ำ)
// ========================================================
void BmpReadTask(void *pvParameters) {
    TickType_t xLastWakeTime = xTaskGetTickCount();
    
    while (1) {
        float t = bmp.readTemperature();
        float p = bmp.readPressure() / 100.0F;

        portENTER_CRITICAL(&dataMux);
        sharedBmpData.temp = t;
        sharedBmpData.pressure = p;
        newBmpDataFlag = true; // แจ้ง Task ส่งข้อมูลว่ามีของใหม่
        portEXIT_CRITICAL(&dataMux);

        // สั่งให้ Task นี้ตื่นมาทำใหม่ทุกๆ 1 วินาทีเป๊ะๆ (1000ms)
        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(1000));
    }
}

// ========================================================
// 🛠️ TASK 3: ส่งข้อมูลผ่าน Serial (รันบน Core 0, 100Hz เป๊ะ)
// ========================================================
void SerialTxTask(void *pvParameters) {
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(5); // 10ms = 100Hz
    
    ImuPayload localImu;
    BmpPayload localBmp;
    bool sendBmp = false;

    while (1) {
        // 1. ก๊อปปี้ข้อมูลล่าสุดออกมาอย่างรวดเร็ว
        portENTER_CRITICAL(&dataMux);
        localImu = sharedImuData; 
        if (newBmpDataFlag) {
            localBmp = sharedBmpData;
            sendBmp = true;
            newBmpDataFlag = false;
        }
        portEXIT_CRITICAL(&dataMux);

        // 2. คำนวณ Checksum และส่ง IMU
        localImu.checksum = calculateChecksum((uint8_t*)&localImu, sizeof(ImuPayload));
        Serial.write((uint8_t*)&localImu, sizeof(ImuPayload));

        // 3. ถ้าครบรอบ 1 วิ ก็ส่ง BMP ต่อท้ายไปเลย
        if (sendBmp) {
            localBmp.checksum = calculateChecksum((uint8_t*)&localBmp, sizeof(BmpPayload));
            Serial.write((uint8_t*)&localBmp, sizeof(BmpPayload));
            sendBmp = false;
        }

        // 4. บล็อก Task นี้ รอจนกว่าจะครบ 10ms (แม่นยำกว่า millis แบบเก่ามหาศาล)
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}

// ========================================================
// ⚙️ SETUP
// ========================================================
void setup() {
    Serial.setTxBufferSize(1024);
    Serial.begin(921600); 
    delay(1000);

    Serial.println("DEBUG,BOOTING_RTOS");

    Wire.begin(21, 22);
    if (!bmp.begin(0x76)) {
        Serial.println("DEBUG,BMP_FAILED");
    }
    Serial2.setRxBufferSize(2048);
    
    Serial2.begin(3000000, SERIAL_8N1, BNO08X_RX_PIN, BNO08X_TX_PIN);
    Serial.println("DEBUG,INIT_BNO08X...");
    
    int retry = 0;
    while (!bno08x.begin_UART(&Serial2)) {
        Serial.println("DEBUG,BNO08X_UART_RETRY");
        delay(500);
        if(++retry > 10) ESP.restart();
    }
    Serial.println("DEBUG,BNO08X_OK");

    bno08x.enableReport(SH2_LINEAR_ACCELERATION, 5000);  
    bno08x.enableReport(SH2_GYROSCOPE_CALIBRATED, 5000); 
    bno08x.enableReport(SH2_ROTATION_VECTOR, 5000);      


    // --- สร้าง Tasks โยนลงแต่ละ Core ---
    // Core 1: จัดการเซ็นเซอร์ทั้งหมด
    xTaskCreatePinnedToCore(ImuReadTask, "ImuTask", 4096, NULL, 3, NULL, 1);
    xTaskCreatePinnedToCore(BmpReadTask, "BmpTask", 2048, NULL, 1, NULL, 1);
    
    // Core 0: จัดการการสื่อสาร (ไม่กวนเซ็นเซอร์)
    xTaskCreatePinnedToCore(SerialTxTask, "TxTask", 4096, NULL, 4, NULL, 0);

    // ลบ Task ของ Loop หลักทิ้งไปเลย เพราะเราใช้ RTOS แล้ว
    vTaskDelete(NULL);
}

void loop() {
    // ปล่อยว่างไว้เลย RTOS จัดการหมดแล้ว
}