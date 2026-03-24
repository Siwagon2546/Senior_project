import serial
import struct
import time

# 1. เชื่อมต่อบอร์ด Drive (115200)
try:
    ser_drive = serial.Serial('/dev/esp32_drive', 115200, timeout=1)
    ser_drive.setDTR(False) # ป้องกัน ESP32 Reset
    ser_drive.setRTS(False)
    print("Drive Port Opened")
except Exception as e:
    print(f"Error opening Drive: {e}")
    ser_drive = None

# 2. เชื่อมต่อบอร์ด Sensor (921600)
try:
    ser_sensor = serial.Serial('/dev/esp32_sensor', 921600, timeout=1)
    ser_sensor.setDTR(False) # ป้องกัน ESP32 Reset
    ser_sensor.setRTS(False)
    print("Sensor Port Opened")
except Exception as e:
    print(f"Error opening Sensor: {e}")
    ser_sensor = None

while True:
    # --- อ่าน Drive (v6.cpp) ---
    if ser_drive and ser_drive.in_waiting >= 19: # 19 bytes คือขนาด FeedbackPacket
        # มองหา Header 0xAA 0x55
        if ser_drive.read(1) == b'\xAA':
            if ser_drive.read(1) == b'\x55':
                data = ser_drive.read(17) # อ่านส่วนที่เหลือ (v, w, left, right, checksum)
                if len(data) == 17:
                    # <ffiiB แปลว่า Little Endian: float(4), float(4), int(4), int(4), unsigned char(1)
                    v, w, left, right, chk = struct.unpack('<ffiiB', data)
                    print(f"[DRIVE] v:{v:.2f}, w:{w:.2f}, Ticks: L={left} R={right}")

    # --- อ่าน Sensor (Uart_BNO086.cpp) ---
    if ser_sensor and ser_sensor.in_waiting >= 44:
        if ser_sensor.read(1) == b'\xAA':
            if ser_sensor.read(1) == b'\xBB':
                msg_type = ser_sensor.read(1)
                
                if msg_type == b'\x01': # IMU Payload (41 bytes left)
                    data = ser_sensor.read(41)
                    if len(data) == 41:
                        # <10fB แปลว่า float 10 ตัวติดกัน ตามด้วย unsigned char 1 ตัว
                        ax, ay, az, gx, gy, gz, qx, qy, qz, qw, chk = struct.unpack('<10fB', data)
                        print(f"[IMU] Accel Z:{az:.2f}, Gyro Z:{gz:.2f}")
                
                elif msg_type == b'\x02': # BMP Payload (9 bytes left)
                    data = ser_sensor.read(9)
                    if len(data) == 9:
                        temp, press, chk = struct.unpack('<ffB', data)
                        print(f"[BMP] Temp:{temp:.2f}C, Pressure:{press:.2f}hPa")
    
    time.sleep(0.001)