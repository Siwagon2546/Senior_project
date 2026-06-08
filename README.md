# Autonomous Robot Localization and Navigation System (Localize-and-navigate)

โปรเจกต์นี้เป็นส่วนหนึ่งของระบบนำทางและระบุตำแหน่งอัตโนมัติสำหรับหุ่นยนต์ (Autonomous Robot) โดยประยุกต์ใช้งานเทคโนโลยี **NVIDIA Isaac ROS** ร่วมกับกล้อง **Intel RealSense** เพื่อประมวลผลการคำนวณประสิทธิภาพสูงบนฮาร์ดแวร์ตระกูล NVIDIA Jetson หรือพีซีที่มี GPU สถาปัตยกรรมระดับสูง ระบบนี้เน้นไปที่การทำ Visual SLAM, การคำนวณความลึกด้วย AI (ESS Stereo Disparity) และการทำแผนที่ 3 มิติแบบ Real-time (nvblox)

---

## 🚀 ภาพรวมของระบบ (System Architecture)

ระบบทำงานบนเฟรมเวิร์ก **ROS 2** โดยรวมทุกโหนดประมวลผลไว้ในตู้คอนเทนเนอร์เดียวกันแบบ Multi-Threaded (`component_container_mt`) เพื่อเปิดใช้งานระบบสื่อสารภายในกระบวนการเดียวกัน (**Intra-process Communications**) ช่วยลด Overhead ในการส่งข้อมูลภาพที่มีความละเอียดสูงได้อย่างมีประสิทธิภาพ

### ส่วนประกอบหลักของระบบ (Core Components)
1. **Sensor Input (`realsense2_camera`)**: ดึงข้อมูลภาพจากกล้องอินฟราเรดซ้าย-ขวา (Infra Stereo), ภาพสี (RGB) และข้อมูลการเคลื่อนไหว (IMU) จากกล้อง Intel RealSense
2. **Image Pre-processing (`isaac_ros_image_proc`)**:
   * แปลงฟอร์แมตภาพขาวดำ (Mono Infra) ให้เป็น `rgb8` เพื่อให้รองรับกับโมเดลโครงข่ายประสาทเทียม
   * ย่อขนาดภาพให้เหลือ $960 \times 576$ พิกเซล ซึ่งเป็นมิติที่เหมาะสมที่สุดสำหรับโมเดล ESS 
3. **AI Depth Estimation (`isaac_ros_ess`)**: ใช้โมเดล **ESS (Stereo Disparity DNN)** ของ NVIDIA เร่งความเร็วผ่าน TensorRT (`ess.engine`) เพื่อคำนวณความลึก (Disparity) จากภาพสเตอริโอคู่
4. **Depth Filtering (`robot_bringup`)**: กรองสัญญาณรบกวนและเติมเต็มพิกเซลความลึกที่ขาดหายไปผ่านโหนดคัสตอม `RealtimeDepthFilterNode`
5. **Visual SLAM (`isaac_ros_visual_slam`)**: ทำการฟิวชันข้อมูลภาพอินฟราเรดสเตอริโอกับ IMU (Visual-Inertial Odometry) เพื่อระบุตำแหน่งของหุ่นยนต์ ($x, y, z$ และ Orientation) แบบ Real-time พร้อมฟีเจอร์โหลดแผนที่เพื่อทำ Localization ณ ตอนเริ่มต้นทำงาน
6. **3D Mapping (`nvblox_ros`)**: นำภาพความลึก (Depth) และภาพสี (Color) มาสร้างเป็นแผนที่ความละเอียดสูงแบบ 3 มิติ (TSDF Base) สำหรับใช้ในการวางแผนเส้นทางและหลบหลีกสิ่งกีดขวาง

---

## 📂 โครงสร้างการส่งข้อมูล (Data Flow & Remappings)

```mermaid
graph TD
    %% Define Nodes and Styles
    subgraph Input [เซ็นเซอร์และข้อมูลดิบ]
        RS[Intel RealSense Camera]
        Color["/camera0/color/image_raw"]
    end

    subgraph PreProcess [กระบวนการแปลงและปรับขนาดภาพ]
        FormatL[ImageFormatConverter Left]
        FormatR[ImageFormatConverter Right]
        ResizeL[Resize Node Left]
        ResizeR[Resize Node Right]
    end

    subgraph Computing [ประมวลผลตำแหน่งและมิติความลึก]
        VSLAM[Visual SLAM Node]
        ESS[ESS Disparity DNN]
        D2D[Disparity To Depth]
        Filter[Realtime Depth Filter]
    end

    subgraph Output [ผลลัพธ์ระบบนำทาง]
        TF["/odom & /map TFs"]
        NVBLOX[nvblox Node]
        Map3D[3D Costmap / Mesh]
    end

    %% Define Connections
    RS -->|/camera0/imu| VSLAM
    RS -->|/camera0/infra1/image_rect_raw| FormatL -->|rgb8| ResizeL
    RS -->|/camera0/infra2/image_rect_raw| FormatR -->|rgb8| ResizeR

    ResizeL -->|/ess/left/image_rect| ESS
    ResizeL -->|/camera0/infra1/camera_info| VSLAM
    ResizeR -->|/ess/right/image_rect| ESS
    ResizeR -->|/camera0/infra2/camera_info| VSLAM

    ESS -->|/ess/disparity| D2D
    D2D -->|/ess/depth/image| Filter
    Filter -->|/ess/depth/image_filtered| NVBLOX
    Color --> NVBLOX

    VSLAM --> TF
    NVBLOX --> Map3D

    %% Style Adjustments
    style RS fill:#e1f5fe,stroke:#01579b,stroke-width:2px
    style VSLAM fill:#e8f5e9,stroke:#2e7d32,stroke-width:2px
    style ESS fill:#ffe0b2,stroke:#ef6c00,stroke-width:2px
    style NVBLOX fill:#f3e5f5,stroke:#6a1b9a,stroke-width:2px
```
## 🚀 วิธีการสั่งงาน (Usage & Deployment)

การรันระบบระบุตำแหน่งและนำทางสามารถเลือกใช้ได้ 2 รูปแบบตามลักษณะการประมวลผลของภาพความลึก:

### 1. โหมดประมวลผล Depth แบบดั้งเดิม (Standard Mode)
โหมดนี้จะส่งภาพ Depth ที่แปลงมาจากข้อความกลุ่มของ ESS เข้าสู่โหนดสร้างแผนที่ `nvblox` โดยตรงโดยไม่มีการกรองพิกเซล:
```bash
ros2 launch robot_bringup White_poselocalize.launch.py
