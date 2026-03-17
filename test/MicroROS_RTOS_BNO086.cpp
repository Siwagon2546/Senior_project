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
#define EXECUTE_EVERY_N_MS(MS, X) do { \
    static volatile int64_t init = -1; \
    if (init == -1) { init = uxr_millis(); } \
    if (uxr_millis() - init > MS) { X; init = uxr_millis(); } \
} while (0)
///////////////////////////////////////////////////////////////////////////////

// ---------- Pin / Serial ----------
#define BNO08X_RX_PIN    16
#define BNO08X_TX_PIN    17
#define BNO08X_RESET_PIN 4
#define UROS_SERIAL      Serial

// ---------- Sensors ----------
Adafruit_BMP280 bmp;
Adafruit_BNO08x bno08x(BNO08X_RESET_PIN);
sh2_SensorValue_t sensorValue;

// ---------- micro-ROS ----------
rcl_publisher_t accel_pub;
rcl_publisher_t gyro_pub;
rcl_publisher_t quaternion_pub;
rcl_publisher_t airpress_pub;

geometry_msgs__msg__Vector3    accel_msg;
geometry_msgs__msg__Vector3    gyro_msg;
geometry_msgs__msg__Quaternion quaternion_msg;
geometry_msgs__msg__Vector3    airpress_msg;

rclc_executor_t executor;
rclc_support_t  support;
rcl_allocator_t allocator;
rcl_node_t      node;
rcl_timer_t     control_timer;
rcl_timer_t     bmp_timer;
rcl_init_options_t init_options;

unsigned long long time_offset = 0;

enum states {
    WAITING_AGENT,
    AGENT_AVAILABLE,
    AGENT_CONNECTED,
    AGENT_DISCONNECTED
} state;

// ---------- RTOS Handles ----------
TaskHandle_t urosTaskHandle   = NULL;
TaskHandle_t imuTaskHandle    = NULL;
TaskHandle_t bmpTaskHandle    = NULL;

SemaphoreHandle_t imuMutex    = NULL;  // guard shared sensorValue / msg
SemaphoreHandle_t publishMutex = NULL; // guard rcl_publish (not re-entrant)

// ---------- Forward declarations ----------
bool createEntities();
bool destroyEntities();
bool syncTime();
void imu_sensor();
void bmp_sensor();

///////////////////////////////////////////////////////////////////////////////
// micro-ROS Callbacks
///////////////////////////////////////////////////////////////////////////////
void controlCallback(rcl_timer_t *timer, int64_t) {
    if (timer != NULL) imu_sensor();
}

void bmpCallback(rcl_timer_t *timer, int64_t) {
    if (timer != NULL) bmp_sensor();
}

///////////////////////////////////////////////////////////////////////////////
// Sensor functions
///////////////////////////////////////////////////////////////////////////////
void imu_sensor() {
    if (xSemaphoreTake(imuMutex, pdMS_TO_TICKS(5)) != pdTRUE) return;

    while (bno08x.getSensorEvent(&sensorValue)) {
        switch (sensorValue.sensorId) {
            case SH2_LINEAR_ACCELERATION:
                accel_msg.x = sensorValue.un.linearAcceleration.x;
                accel_msg.y = sensorValue.un.linearAcceleration.y;
                accel_msg.z = sensorValue.un.linearAcceleration.z;
                if (xSemaphoreTake(publishMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
                    rcl_publish(&accel_pub, &accel_msg, NULL);
                    xSemaphoreGive(publishMutex);
                }
                break;

            case SH2_GYROSCOPE_CALIBRATED:
                gyro_msg.x = sensorValue.un.gyroscope.x;
                gyro_msg.y = sensorValue.un.gyroscope.y;
                gyro_msg.z = sensorValue.un.gyroscope.z;
                if (xSemaphoreTake(publishMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
                    rcl_publish(&gyro_pub, &gyro_msg, NULL);
                    xSemaphoreGive(publishMutex);
                }
                break;

            case SH2_ROTATION_VECTOR:
                quaternion_msg.x = sensorValue.un.rotationVector.i;
                quaternion_msg.y = sensorValue.un.rotationVector.j;
                quaternion_msg.z = sensorValue.un.rotationVector.k;
                quaternion_msg.w = sensorValue.un.rotationVector.real;
                if (xSemaphoreTake(publishMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
                    rcl_publish(&quaternion_pub, &quaternion_msg, NULL);
                    xSemaphoreGive(publishMutex);
                }
                break;
        }
    }

    xSemaphoreGive(imuMutex);
}

void bmp_sensor() {
    airpress_msg.x = bmp.readTemperature();
    airpress_msg.y = bmp.readPressure() / 100.0F;
    airpress_msg.z = 0.0;
    if (xSemaphoreTake(publishMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
        rcl_publish(&airpress_pub, &airpress_msg, NULL);
        xSemaphoreGive(publishMutex);
    }
}

///////////////////////////////////////////////////////////////////////////////
// micro-ROS entity management
///////////////////////////////////////////////////////////////////////////////
bool createEntities() {
    allocator   = rcl_get_default_allocator();
    init_options = rcl_get_zero_initialized_init_options();
    rcl_init_options_init(&init_options, allocator);
    rcl_init_options_set_domain_id(&init_options, 22);

    RCCHECK(rclc_support_init_with_options(&support, 0, NULL, &init_options, &allocator));
    RCCHECK(rclc_node_init_default(&node, "bno08x_node", "", &support));

    RCCHECK(rclc_publisher_init_best_effort(&accel_pub, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Vector3), "/bno/accel"));
    RCCHECK(rclc_publisher_init_best_effort(&gyro_pub, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Vector3), "/bno/gyro"));
    RCCHECK(rclc_publisher_init_best_effort(&quaternion_pub, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Quaternion), "/bno/quaternion"));
    RCCHECK(rclc_publisher_init_best_effort(&airpress_pub, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Vector3), "/air/airpress"));

    RCCHECK(rclc_timer_init_default(&control_timer, &support, RCL_MS_TO_NS(5),  controlCallback));
    RCCHECK(rclc_timer_init_default(&bmp_timer,     &support, RCL_MS_TO_NS(1000), bmpCallback));

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
    rcl_timer_fini(&control_timer);
    rcl_timer_fini(&bmp_timer);
    rclc_executor_fini(&executor);
    rcl_node_fini(&node);
    rclc_support_fini(&support);
    rcl_init_options_fini(&init_options);

    return true;
}

bool syncTime() {
    unsigned long now = millis();
    RCCHECK(rmw_uros_sync_session(10));
    unsigned long long ros_time_ms = rmw_uros_epoch_millis();
    time_offset = ros_time_ms - now;
    return true;
}

///////////////////////////////////////////////////////////////////////////////
// RTOS Tasks
///////////////////////////////////////////////////////////////////////////////

// Task: micro-ROS state machine — รันบน Core 0
void urosTask(void *pvParameters) {
    // รอ Serial stable ก่อน ping
    vTaskDelay(pdMS_TO_TICKS(2000));
    set_microros_serial_transports(UROS_SERIAL);
    state = WAITING_AGENT;

    for (;;) {
        switch (state) {
            case WAITING_AGENT:
                EXECUTE_EVERY_N_MS(500,
                    state = (RMW_RET_OK == rmw_uros_ping_agent(100, 1))
                            ? AGENT_AVAILABLE : WAITING_AGENT;
                );
                break;

            case AGENT_AVAILABLE:
                state = createEntities() ? AGENT_CONNECTED : WAITING_AGENT;
                if (state == WAITING_AGENT) destroyEntities();
                break;

            case AGENT_CONNECTED:
                EXECUTE_EVERY_N_MS(200,
                    state = (RMW_RET_OK == rmw_uros_ping_agent(100, 1))
                            ? AGENT_CONNECTED : AGENT_DISCONNECTED;
                );
                if (state == AGENT_CONNECTED) {
                    rclc_executor_spin_some(&executor, RCL_MS_TO_NS(5));
                }
                break;

            case AGENT_DISCONNECTED:
                destroyEntities();
                // reset transport แล้ว reconnect ใหม่ ไม่ต้อง restart
                set_microros_serial_transports(UROS_SERIAL);
                vTaskDelay(pdMS_TO_TICKS(500));
                state = WAITING_AGENT;
                break;
        }
        vTaskDelay(pdMS_TO_TICKS(1)); // yield ให้ task อื่น
    }
}

// Task: อ่าน BNO08x — รันบน Core 1
void imuTask(void *pvParameters) {
    for (;;) {
        // อ่านข้อมูลดิบจาก BNO08x เข้า buffer ภายใน library
        // imu_sensor() จะถูกเรียกผ่าน controlCallback ของ executor
        // task นี้ทำหน้าที่ keepalive / watchdog เท่านั้น
        vTaskDelay(pdMS_TO_TICKS(5));
    }
}

///////////////////////////////////////////////////////////////////////////////
// Setup & Loop
///////////////////////////////////////////////////////////////////////////////
void setup() {
    Serial.begin(921600);

    // Init BNO08x ด้วย retry
    Serial2.begin(3000000, SERIAL_8N1, BNO08X_RX_PIN, BNO08X_TX_PIN);
    int retry = 0;
    while (!bno08x.begin_UART(&Serial2)) {
        vTaskDelay(pdMS_TO_TICKS(500));
        if (++retry > 10) ESP.restart();
    }
    bno08x.enableReport(SH2_LINEAR_ACCELERATION,  5000);
    bno08x.enableReport(SH2_GYROSCOPE_CALIBRATED, 5000);
    bno08x.enableReport(SH2_ROTATION_VECTOR,       5000);

    // Init BMP280
    Wire.begin(21, 22);
    bmp.begin(0x76);

    // สร้าง Mutex
    imuMutex     = xSemaphoreCreateMutex();
    publishMutex = xSemaphoreCreateMutex();

    // สร้าง Tasks
    // urosTask → Core 0 (เดียวกับ WiFi/BT stack ของ ESP-IDF)
    // imuTask  → Core 1 (app core — performance สูงกว่า)
    xTaskCreatePinnedToCore(urosTask, "uros_task", 8192, NULL, 5, &urosTaskHandle, 0);
    xTaskCreatePinnedToCore(imuTask,  "imu_task",  4096, NULL, 4, &imuTaskHandle,  1);
}

void loop() {
    // ไม่ใช้ loop() แล้ว — ทุกอย่างอยู่ใน RTOS tasks
    vTaskDelay(pdMS_TO_TICKS(1000));
}