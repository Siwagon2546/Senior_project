#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_BNO08x.h>
#include <Adafruit_BMP280.h>

#include <micro_ros_platformio.h>
#include <stdio.h>

#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <geometry_msgs/msg/vector3.h>
#include <geometry_msgs/msg/quaternion.h>

#include "config.h"
#include "driver/twai.h"

///////////////////////////////////////////////////////////////////////////////
#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){ return false; }}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}
#define EXECUTE_EVERY_N_MS(MS, X) do { static volatile int64_t init = -1; if (init == -1) { init = uxr_millis(); } if (uxr_millis() - init > MS) { X; init = uxr_millis(); } } while (0)
///////////////////////////////////////////////////////////////////////////////

unsigned long long time_offset = 0;

// --- Publishers ---
rcl_publisher_t accel_pub;
rcl_publisher_t gyro_pub;
rcl_publisher_t quaternion_pub;
rcl_publisher_t airpress_pub;

// --- Messages ---
geometry_msgs__msg__Vector3    accel_msg;
geometry_msgs__msg__Vector3    gyro_msg;
geometry_msgs__msg__Quaternion quaternion_msg;
geometry_msgs__msg__Vector3    airpress_msg;

rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_timer_t control_timer;
rcl_timer_t bmp_timer;
rcl_init_options_t init_options;

enum states { WAITING_AGENT, AGENT_AVAILABLE, AGENT_CONNECTED, AGENT_DISCONNECTED } state;

// ==========================================
// SHARED MEMORY & MUTEX (Core 0 <-> Core 1)
// ==========================================
portMUX_TYPE sensorMux = portMUX_INITIALIZER_UNLOCKED;

// ตัวแปรเก็บค่าล่าสุดจากเซ็นเซอร์ (Global Variables)
volatile float g_accel[3] = {0, 0, 0};
volatile float g_gyro[3]  = {0, 0, 0};
volatile float g_quat[4]  = {0, 0, 0, 1.0};
volatile float g_temp     = 0.0;
volatile float g_press    = 0.0;

// ==========================================
// FUNCTION DECLARATIONS
// ==========================================
bool syncTime();
bool createEntities();
bool destroyEntities();

#define BNO08X_RX_PIN    16
#define BNO08X_TX_PIN    17
#define UROS_SERIAL      Serial
#define BNO08X_RESET_PIN 4

Adafruit_BMP280 bmp;
Adafruit_BNO08x bno08x(BNO08X_RESET_PIN);
sh2_SensorValue_t sensorValue;

// ==========================================
// CORE 1: SENSOR TASK (FreeRTOS)
// ==========================================
void SensorTask(void *pvParameters) {
    unsigned long last_bmp_time = 0;

    while (1) {
        // 1. อ่าน BNO08x ให้ไวที่สุด (Polling)
        if (bno08x.getSensorEvent(&sensorValue)) {
            // ล็อก Mutex เพื่อเขียนข้อมูล ป้องกัน Core 0 มาแอบอ่านตอนข้อมูลยังเขียนไม่เสร็จ
            portENTER_CRITICAL(&sensorMux);
            switch (sensorValue.sensorId) {
                case SH2_LINEAR_ACCELERATION:
                    g_accel[0] = sensorValue.un.linearAcceleration.x;
                    g_accel[1] = sensorValue.un.linearAcceleration.y;
                    g_accel[2] = sensorValue.un.linearAcceleration.z;
                    break;
                case SH2_GYROSCOPE_CALIBRATED:
                    g_gyro[0] = sensorValue.un.gyroscope.x;
                    g_gyro[1] = sensorValue.un.gyroscope.y;
                    g_gyro[2] = sensorValue.un.gyroscope.z;
                    break;
                case SH2_ROTATION_VECTOR:
                    g_quat[0] = sensorValue.un.rotationVector.i;
                    g_quat[1] = sensorValue.un.rotationVector.j;
                    g_quat[2] = sensorValue.un.rotationVector.k;
                    g_quat[3] = sensorValue.un.rotationVector.real;
                    break;
            }
            portEXIT_CRITICAL(&sensorMux);
        }

        // 2. อ่าน BMP280 ที่ 1Hz (1000ms) โดยไม่ใช้ delay()
        if (millis() - last_bmp_time >= 1000) {
            float temp = bmp.readTemperature();
            float press = bmp.readPressure() / 100.0F;

            portENTER_CRITICAL(&sensorMux);
            g_temp = temp;
            g_press = press;
            portEXIT_CRITICAL(&sensorMux);

            last_bmp_time = millis();
        }

        // ปล่อยให้ FreeRTOS สลับไปทำงานอื่นบ้าง (กัน Watchdog Timer เตะ)
        // หน่วง 1 Tick (~1ms) ทำให้ลูปนี้รันที่ประมาณ 1000Hz สบายๆ
        vTaskDelay(pdMS_TO_TICKS(1)); 
    }
}

// ==========================================
// CORE 0: MICRO-ROS CALLBACKS
// ==========================================
// ส่งข้อมูล IMU ทุกๆ 10ms (100Hz)
void controlCallback(rcl_timer_t *timer, int64_t last_call_time) {
    if (timer != NULL) {
        // ดึงข้อมูลจาก Shared Memory อย่างรวดเร็ว
        portENTER_CRITICAL(&sensorMux);
        accel_msg.x = g_accel[0]; accel_msg.y = g_accel[1]; accel_msg.z = g_accel[2];
        gyro_msg.x  = g_gyro[0];  gyro_msg.y  = g_gyro[1];  gyro_msg.z  = g_gyro[2];
        quaternion_msg.x = g_quat[0]; quaternion_msg.y = g_quat[1]; 
        quaternion_msg.z = g_quat[2]; quaternion_msg.w = g_quat[3];
        portEXIT_CRITICAL(&sensorMux);

        rcl_publish(&accel_pub, &accel_msg, NULL);
        rcl_publish(&gyro_pub, &gyro_msg, NULL);
        rcl_publish(&quaternion_pub, &quaternion_msg, NULL);
    }
}

// ส่งข้อมูลบรรยากาศ ทุกๆ 1 วินาที (1Hz)
void bmpCallback(rcl_timer_t *timer, int64_t last_call_time) {
    if (timer != NULL) {
        portENTER_CRITICAL(&sensorMux);
        airpress_msg.x = g_temp;
        airpress_msg.y = g_press;
        airpress_msg.z = 0.0;
        portEXIT_CRITICAL(&sensorMux);

        rcl_publish(&airpress_pub, &airpress_msg, NULL);
    }
}

// ==========================================
// MAIN SETUP & LOOP
// ==========================================
void setup() {
    Serial.begin(921600);
    delay(2000); 

    // Setup BNO08x (UART)
    Serial2.begin(3000000, SERIAL_8N1, BNO08X_RX_PIN, BNO08X_TX_PIN);
    if (!bno08x.begin_UART(&Serial2)) {
        while (1) delay(10);
    }
    bno08x.enableReport(SH2_LINEAR_ACCELERATION, 5000);  // 200Hz
    bno08x.enableReport(SH2_GYROSCOPE_CALIBRATED, 5000); // 200Hz
    bno08x.enableReport(SH2_ROTATION_VECTOR, 5000);      // 200Hz

    // Setup BMP280 (I2C)
    Wire.begin(21, 22);
    if (!bmp.begin(0x76)) {
        // BMP280 ไม่ตอบสนอง
    }

    // 🚀 สร้าง FreeRTOS Task ให้รันบน Core 1 (Core 0 เอาไว้ให้ micro-ROS)
    xTaskCreatePinnedToCore(
        SensorTask,   /* Function to implement the task */
        "SensorTask", /* Name of the task */
        8192,         /* Stack size in words */
        NULL,         /* Task input parameter */
        5,            /* Priority of the task (สูงพอสมควร) */
        NULL,         /* Task handle. */
        1             /* Core where the task should run */
    );

    set_microros_serial_transports(UROS_SERIAL);
    state = WAITING_AGENT;
}

void loop() {
    switch (state) {
    case WAITING_AGENT:
        // while (Serial.available()) {
        //     Serial.read();
        // }
        EXECUTE_EVERY_N_MS(500, state = (RMW_RET_OK == rmw_uros_ping_agent(100, 1)) ? AGENT_AVAILABLE : WAITING_AGENT;);
        break;
    case AGENT_AVAILABLE:
        state = (true == createEntities()) ? AGENT_CONNECTED : WAITING_AGENT;
        if (state == WAITING_AGENT) destroyEntities();
        break;
    case AGENT_CONNECTED:
        EXECUTE_EVERY_N_MS(200, state = (RMW_RET_OK == rmw_uros_ping_agent(100, 1)) ? AGENT_CONNECTED : AGENT_DISCONNECTED;);
        if (state == AGENT_CONNECTED) {
            rclc_executor_spin_some(&executor, RCL_MS_TO_NS(5));
        }
        break;
    case AGENT_DISCONNECTED:
        destroyEntities();
        state = WAITING_AGENT;
        break;
    default:
        break;
    }
}

// ==========================================
// ROS ENTITIES MANAGEMENT
// ==========================================
bool createEntities() {
    allocator = rcl_get_default_allocator();
    init_options = rcl_get_zero_initialized_init_options();
    rcl_init_options_init(&init_options, allocator);
    rcl_init_options_set_domain_id(&init_options, 22);

    rclc_support_init_with_options(&support, 0, NULL, &init_options, &allocator);
    RCCHECK(rclc_node_init_default(&node, "bno08x_node", "", &support));

    RCCHECK(rclc_publisher_init_best_effort(&accel_pub, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Vector3), "/bno/accel"));
    RCCHECK(rclc_publisher_init_best_effort(&gyro_pub, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Vector3), "/bno/gyro"));
    RCCHECK(rclc_publisher_init_best_effort(&quaternion_pub, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Quaternion), "/bno/quaternion"));
    RCCHECK(rclc_publisher_init_best_effort(&airpress_pub, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Vector3), "/air/airpress"));

    RCCHECK(rclc_timer_init_default(&control_timer, &support, RCL_MS_TO_NS(10), controlCallback));
    RCCHECK(rclc_timer_init_default(&bmp_timer, &support, RCL_MS_TO_NS(1000), bmpCallback));

    executor = rclc_executor_get_zero_initialized_executor();
    RCCHECK(rclc_executor_init(&executor, &support.context, 2, &allocator));
    RCCHECK(rclc_executor_add_timer(&executor, &control_timer));
    RCCHECK(rclc_executor_add_timer(&executor, &bmp_timer));

    if (!syncTime()) return false;
    return true;
}

bool destroyEntities() {
    rmw_context_t *rmw_context = rcl_context_get_rmw_context(&support.context);
    (void)rmw_uros_set_context_entity_destroy_session_timeout(rmw_context, 0);

    rcl_publisher_fini(&accel_pub,      &node);
    rcl_publisher_fini(&gyro_pub,       &node);
    rcl_publisher_fini(&quaternion_pub, &node);
    rcl_publisher_fini(&airpress_pub,   &node);
    rcl_node_fini(&node);
    rcl_timer_fini(&control_timer);
    rcl_timer_fini(&bmp_timer);
    rclc_executor_fini(&executor);
    rclc_support_fini(&support);
                            
    rcl_init_options_fini(&init_options);
    Serial.end();             // ปิดการทำงาน UART
    delay(200);               // รอให้ไฟนิ่ง
    Serial.begin(921600);     // เปิด UART ใหม่
    set_microros_serial_transports(Serial); // ผูก micro-ROS เข้ากับพอร์ตใหม่
    
    return true;
}

bool syncTime() {
    unsigned long now = millis();
    RCCHECK(rmw_uros_sync_session(10));
    unsigned long long ros_time_ms = rmw_uros_epoch_millis();
    time_offset = ros_time_ms - now;
    return true;
}