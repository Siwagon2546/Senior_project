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
## 🛠️ ความต้องการของระบบ (Prerequisites & Dependencies)

* **OS:** Ubuntu 22.04 LTS (แนะนำ)
* **Middleware:** ROS 2 (Humble / Iron / Jazzy)
* **Environment:** NVIDIA Isaac ROS Dev Container (แนะนำ เพื่อความสะดวกในการติดตั้งไลบรารีของ NVIDIA)
* **Hardware:** * กล้อง Intel RealSense (เช่น D435, D435i, D455)
  * NVIDIA GPU (Jetson Orin Series หรือ RTX Desktop GPU)
* **ROS 2 Packages:**
  * `realsense2_camera`
  * `isaac_ros_image_proc`
  * `isaac_ros_ess`
  * `isaac_ros_stereo_image_proc`
  * `isaac_ros_visual_slam`
  * `nvblox_ros`

---

## ⚙️ พารามิเตอร์ที่สำคัญ (Key Configurations)

พารามิเตอร์เหล่านี้ถูกกำหนดผ่าน Launch Arguments และสามารถปรับแต่งได้ในไฟล์ Launch:

| พารามิเตอร์ | ค่าเริ่มต้น | คำอธิบาย |
| :--- | :--- | :--- |
| `camera_name` | `camera0` | ชื่อ Namespace ของกล้องที่จะใช้ในระบบ |
| `config_file` | `realsense.yaml` | ไฟล์ตั้งค่าคุณสมบัติภายในของกล้อง RealSense |
| `engine_file_path` | `ess.engine` | พาธไฟล์ TensorRT Engine ของโมเดลโครงข่ายประสาทเทียม ESS |
| `threshold` | `0.0` | ค่าเกณฑ์ความเชื่อมั่นในการคำนวณ Disparity ของ ESS |
| `load_map_folder_path` | `/workspaces/isaac_ros-dev/maps/...` | โฟลเดอร์แผนที่ VSLAM ที่บันทึกไว้ล่วงหน้าเพื่อทำ Localization ตอนเริ่มต้น |

---

## 🚀 วิธีการสั่งงาน (Usage & Deployment)

การรันระบบระบุตำแหน่งและนำทางสามารถเลือกใช้ได้ 2 รูปแบบตามลักษณะการประมวลผลของภาพความลึก:

### 1. โหมดประมวลผล Depth แบบดั้งเดิม (Standard Mode)
โหมดนี้จะส่งภาพ Depth ที่แปลงมาจากข้อความกลุ่มของ ESS เข้าสู่โหนดสร้างแผนที่ `nvblox` โดยตรงโดยไม่มีการกรองพิกเซล:
```bash
ros2 launch robot_bringup White_poselocalize.launch.py
