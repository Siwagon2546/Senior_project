[Intel RealSense]
│
├──► /camera0/imu ──────────────────────────────────────────────────────────┐
├──► /camera0/infra1/image_rect_raw ──► [ImageFormat] ──► [Left Resize] ──┐ │
└──► /camera0/infra2/image_rect_raw ──► [ImageFormat] ──► [Right Resize] ─┼─┼─► [Visual SLAM] ──► /odom & /map TFs
│ │ │
▼ ▼ │
[ESS Disparity]◄┘
│
▼
[DisparityToDepth]
│
▼
[RealtimeDepthFilter]
│
▼ (/ess/depth/image_filtered)
[camera0/color/image_raw] ──────────────────────────────────────► [nvblox_node] ──► 3D Costmap / Mesh


---

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
