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
rcl_publisher_t accel_pub;       // /bno/accel      (Vector3)
rcl_publisher_t gyro_pub;        // /bno/gyro       (Vector3)
rcl_publisher_t quaternion_pub;  // /bno/quaternion (Quaternion)
rcl_publisher_t airpress_pub;    // /air/airpress   (Vector3)

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

enum states {
    WAITING_AGENT,
    AGENT_AVAILABLE,
    AGENT_CONNECTED,
    AGENT_DISCONNECTED
} state;

void rclErrorLoop();
bool syncTime();
bool createEntities();
bool destroyEntities();
struct timespec getTime();
void imu_sensor();
void bmp_sensor();

#define BNO08X_RX_PIN    16
#define BNO08X_TX_PIN    17
#define UROS_SERIAL      Serial
#define BNO08X_RESET_PIN 4

Adafruit_BMP280 bmp;
Adafruit_BNO08x bno08x(BNO08X_RESET_PIN);
sh2_SensorValue_t sensorValue;

void setup() {
    // delay(2000); // ✅ เพิ่ม delay ให้ Serial stable ก่อน
    Serial.begin(921600);
    delay(2000); // ✅ เพิ่ม delay ให้ Serial stable ก่อน
    // delay(100);
    int retry = 0;
    Serial2.begin(3000000, SERIAL_8N1, BNO08X_RX_PIN, BNO08X_TX_PIN);
    if (!bno08x.begin_UART(&Serial2)) {
        // while (1) delay(10);
        // delay(500);
        if (++retry > 10) ESP.restart(); 
    }

    // เปิด 3 report: Accel, Gyro, Rotation Vector
    bno08x.enableReport(SH2_LINEAR_ACCELERATION,   5000);  // 200Hz
    bno08x.enableReport(SH2_GYROSCOPE_CALIBRATED,  5000);  // 200Hz
    bno08x.enableReport(SH2_ROTATION_VECTOR,        5000);  // 200Hz

    Wire.begin(21, 22);
    if (!bmp.begin(0x76)) {
        // BMP280 ไม่ตอบสนอง
    }

    set_microros_serial_transports(UROS_SERIAL);
    state = WAITING_AGENT;
}

void loop() {
    switch (state) {
    case WAITING_AGENT:
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
        // delay(500); // ✅ เพิ่มตรงนี้
        // ESP.restart();
        break;
    default:
        break;
    }
}

// IMU callback — drain buffer ทุก event แล้ว publish แยก topic
void controlCallback(rcl_timer_t *timer, int64_t last_call_time) {
    if (timer != NULL) {
        imu_sensor();
    }
}

// BMP callback — 1Hz ไม่กวน IMU
void bmpCallback(rcl_timer_t *timer, int64_t last_call_time) {
    if (timer != NULL) {
        bmp_sensor();
    }
}

void imu_sensor() {
    while (bno08x.getSensorEvent(&sensorValue)) {
        switch (sensorValue.sensorId) {

            case SH2_LINEAR_ACCELERATION:
                accel_msg.x = sensorValue.un.linearAcceleration.x;
                accel_msg.y = sensorValue.un.linearAcceleration.y;
                accel_msg.z = sensorValue.un.linearAcceleration.z;
                rcl_publish(&accel_pub, &accel_msg, NULL);
                break;

            case SH2_GYROSCOPE_CALIBRATED:
                gyro_msg.x = sensorValue.un.gyroscope.x;
                gyro_msg.y = sensorValue.un.gyroscope.y;
                gyro_msg.z = sensorValue.un.gyroscope.z;
                rcl_publish(&gyro_pub, &gyro_msg, NULL);
                break;

            case SH2_ROTATION_VECTOR:
                quaternion_msg.x = sensorValue.un.rotationVector.i;
                quaternion_msg.y = sensorValue.un.rotationVector.j;
                quaternion_msg.z = sensorValue.un.rotationVector.k;
                quaternion_msg.w = sensorValue.un.rotationVector.real;
                rcl_publish(&quaternion_pub, &quaternion_msg, NULL);
                break;
        }
    }
}

void bmp_sensor() {
    airpress_msg.x = bmp.readTemperature();
    airpress_msg.y = bmp.readPressure() / 100.0F;
    airpress_msg.z = 0.0;
    rcl_publish(&airpress_pub, &airpress_msg, NULL);
}

bool createEntities() {
    allocator = rcl_get_default_allocator();
    init_options = rcl_get_zero_initialized_init_options();
    rcl_init_options_init(&init_options, allocator);
    rcl_init_options_set_domain_id(&init_options, 22);

    rclc_support_init_with_options(&support, 0, NULL, &init_options, &allocator);
    RCCHECK(rclc_node_init_default(&node, "bno08x_node", "", &support));

    // /bno/accel — Vector3 (m/s^2)
    RCCHECK(rclc_publisher_init_best_effort(&accel_pub, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Vector3), "/bno/accel"));

    // /bno/gyro — Vector3 (rad/s)
    RCCHECK(rclc_publisher_init_best_effort(&gyro_pub, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Vector3), "/bno/gyro"));

    // /bno/quaternion — Quaternion (x y z w)
    RCCHECK(rclc_publisher_init_best_effort(&quaternion_pub, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Quaternion), "/bno/quaternion"));

    // /air/airpress — Vector3 (x=temp, y=pressure, z=0)
    RCCHECK(rclc_publisher_init_best_effort(&airpress_pub, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Vector3), "/air/airpress"));

    // IMU timer 10ms = 100Hz
    RCCHECK(rclc_timer_init_default(&control_timer, &support,
        RCL_MS_TO_NS(9), controlCallback));

    // BMP timer 1000ms = 1Hz
    RCCHECK(rclc_timer_init_default(&bmp_timer, &support,
        RCL_MS_TO_NS(1000), bmpCallback));

    executor = rclc_executor_get_zero_initialized_executor();
    RCCHECK(rclc_executor_init(&executor, &support.context, 2, &allocator));
    RCCHECK(rclc_executor_add_timer(&executor, &control_timer));
    RCCHECK(rclc_executor_add_timer(&executor, &bmp_timer));

    // syncTime();
    // return true;
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
    // set_microros_serial_transports(UROS_SERIAL);
    // delay(500);

    return true;
}

bool syncTime() {
    unsigned long now = millis();
    RCCHECK(rmw_uros_sync_session(10));
    unsigned long long ros_time_ms = rmw_uros_epoch_millis();
    time_offset = ros_time_ms - now;
    return true;
}

struct timespec getTime() {
    struct timespec tp = {0};
    unsigned long long now = millis() + time_offset;
    tp.tv_sec = now / 1000;
    tp.tv_nsec = (now % 1000) * 1000000;
    return tp;
}

void rclErrorLoop() {
    delay(500);
    ESP.restart(); 
}