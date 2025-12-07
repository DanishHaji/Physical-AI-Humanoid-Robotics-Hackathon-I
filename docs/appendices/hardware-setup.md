# Hardware Setup Guide

**Complete guide to procuring and setting up hardware for Physical AI & Humanoid Robotics development**

---

## Overview

This guide covers three hardware tiers:
1. **Minimal Setup** ($500-1000): Simulation-only development on consumer hardware
2. **Development Setup** ($2000-5000): Basic sensors + simulation on workstation
3. **Production Setup** ($10,000-50,000): Full humanoid robot with compute platform

Choose based on your goals:
- **Learning/Research**: Minimal or Development setup sufficient
- **Prototyping**: Development setup with real sensors
- **Deployment**: Production setup for real-world applications

---

## 1. Minimal Setup (Simulation-Only)

**Purpose**: Learn ROS 2, Isaac Sim, VLA concepts without physical hardware

### Hardware Requirements

#### Compute Platform (Choose One)

**Option A: Desktop Workstation** (Recommended for performance)
- **CPU**: AMD Ryzen 7 5800X or Intel i7-12700K (8+ cores)
- **GPU**: NVIDIA RTX 3060 Ti (8GB VRAM minimum) or better
  - RTX 3070/3080: Better for Isaac Sim (more parallel envs)
  - RTX 4070/4090: Best performance, supports latest features
- **RAM**: 32GB DDR4 (16GB minimum, 64GB recommended for large scenes)
- **Storage**: 1TB NVMe SSD (500GB minimum)
- **OS**: Ubuntu 22.04 LTS (dual-boot or native install recommended)

**Estimated Cost**: $1200-2000

**Retailers**:
- USA: Newegg, Micro Center, Amazon
- Europe: Scan.co.uk, Alternate.de
- Asia: Lazada, Shopee

**Option B: Gaming Laptop with NVIDIA GPU**
- **Specs**: RTX 3060 Mobile (6GB VRAM) or better, 16GB RAM, i7/Ryzen 7
- **Pros**: Portable, all-in-one solution
- **Cons**: Thermal throttling, harder to upgrade
- **Estimated Cost**: $1000-1500

**Models**:
- ASUS ROG Zephyrus G14/G15
- Lenovo Legion 5 Pro
- MSI Katana/Sword series

**Option C: Cloud Instance** (For those without local GPU)
- **Providers**: AWS EC2 (g4dn.xlarge), Google Cloud (n1-standard-4 + T4), Paperspace
- **Specs**: 4 vCPUs, 16GB RAM, NVIDIA T4 or better
- **Estimated Cost**: $0.50-1.00/hour ($40-80/month for part-time use)
- **Pros**: No upfront cost, scalable
- **Cons**: Recurring cost, network latency for GUI

### Software Installation

#### 1. Ubuntu 22.04 LTS Setup
```bash
# Download from: https://ubuntu.com/download/desktop
# Create bootable USB: https://ubuntu.com/tutorials/create-a-usb-stick-on-windows

# After installation, update system
sudo apt update && sudo apt upgrade -y

# Install essential build tools
sudo apt install -y build-essential cmake git curl wget
```

#### 2. NVIDIA Drivers + CUDA
```bash
# Install NVIDIA drivers (use recommended version)
ubuntu-drivers devices
sudo ubuntu-drivers autoinstall

# Reboot
sudo reboot

# Verify GPU detected
nvidia-smi
# Should show your GPU model and driver version

# Install CUDA 12.1+ (for Isaac Sim)
wget https://developer.download.nvidia.com/compute/cuda/repos/ubuntu2204/x86_64/cuda-keyring_1.1-1_all.deb
sudo dpkg -i cuda-keyring_1.1-1_all.deb
sudo apt update
sudo apt install -y cuda-toolkit-12-1

# Add to PATH (~/.bashrc)
echo 'export PATH=/usr/local/cuda/bin:$PATH' >> ~/.bashrc
echo 'export LD_LIBRARY_PATH=/usr/local/cuda/lib64:$LD_LIBRARY_PATH' >> ~/.bashrc
source ~/.bashrc

# Verify CUDA
nvcc --version
```

#### 3. ROS 2 Humble Installation
```bash
# Add ROS 2 repository
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Install ROS 2 Humble Desktop
sudo apt update
sudo apt install -y ros-humble-desktop

# Install development tools
sudo apt install -y python3-colcon-common-extensions python3-rosdep

# Initialize rosdep
sudo rosdep init
rosdep update

# Add to ~/.bashrc
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc

# Verify installation
ros2 run demo_nodes_cpp talker
# Should see "Hello World" messages
```

#### 4. Isaac Sim Installation (Docker Method)
```bash
# Install Docker
curl -fsSL https://get.docker.com -o get-docker.sh
sudo sh get-docker.sh
sudo usermod -aG docker $USER
# Log out and back in for group changes

# Install NVIDIA Container Toolkit
distribution=$(. /etc/os-release;echo $ID$VERSION_ID)
curl -s -L https://nvidia.github.io/nvidia-docker/gpgkey | sudo apt-key add -
curl -s -L https://nvidia.github.io/nvidia-docker/$distribution/nvidia-docker.list | sudo tee /etc/apt/sources.list.d/nvidia-docker.list
sudo apt update && sudo apt install -y nvidia-docker2
sudo systemctl restart docker

# Pull Isaac Sim image (22GB download, be patient)
docker pull nvcr.io/nvidia/isaac-sim:4.0.0

# Create workspace
mkdir -p ~/isaac_workspace

# Run Isaac Sim
xhost +local:docker
docker run --rm -it \
  --gpus all \
  -e DISPLAY=$DISPLAY \
  -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
  -v ~/isaac_workspace:/workspace \
  --network host \
  nvcr.io/nvidia/isaac-sim:4.0.0

# Inside container, launch Isaac Sim
./isaac-sim.sh
```

### Verification Tests

**Test 1: ROS 2 Communication**
```bash
# Terminal 1
ros2 run demo_nodes_cpp talker

# Terminal 2
ros2 run demo_nodes_cpp listener
# Should see messages received
```

**Test 2: Isaac Sim GPU Acceleration**
```bash
# In Isaac Sim GUI:
# Create > Humanoid Robot > Unitree H1
# Press Play button (bottom-left)
# Robot should simulate in real-time (60 FPS)

# Check GPU usage
nvidia-smi
# GPU Utilization should be 80-100% during simulation
```

---

## 2. Development Setup (Sensors + Workstation)

**Purpose**: Develop and test perception/manipulation algorithms with real sensors before deploying to robot

### Additional Hardware

#### Intel RealSense D435i Depth Camera

**Specs**:
- RGB: 1920x1080 @ 30 FPS
- Depth: 1280x720 @ 30 FPS (stereo IR + depth processor)
- Range: 0.3m - 10m
- IMU: Accelerometer + Gyroscope
- Interface: USB 3.0

**Purchase**:
- **Price**: $329 USD (official), $250-280 (third-party sellers)
- **Retailers**: Intel Store, Amazon, Mouser Electronics
- **Link**: https://www.intelrealsense.com/depth-camera-d435i/

**Alternative**: RealSense D405 (shorter range, 0.07-1.5m, better for manipulation): $129

**Setup**:
```bash
# Install librealsense SDK
sudo apt install -y ros-humble-realsense2-camera ros-humble-realsense2-description

# Connect camera via USB 3.0 (blue port)

# Test camera
rs-enumerate-devices
# Should show D435i details

# Launch ROS 2 node
ros2 launch realsense2_camera rs_launch.py enable_depth:=true enable_color:=true

# View RGB stream
ros2 run rqt_image_view rqt_image_view /camera/color/image_raw

# View depth stream
ros2 run rqt_image_view rqt_image_view /camera/depth/image_rect_raw
```

**Calibration** (See Appendix C: Troubleshooting, Camera Calibration section)

#### NVIDIA Jetson Orin Nano/NX (Edge Compute)

**Use Case**: Deploy trained models on robot for on-device inference (no cloud dependency)

**Models**:
- **Jetson Orin Nano** (4GB/8GB): $199/$399, 512-1024 CUDA cores, 5-10 TOPS AI
- **Jetson Orin NX** (8GB/16GB): $399/$599, 1024-2048 CUDA cores, 40-100 TOPS AI
- **Jetson AGX Orin** (32GB/64GB): $999/$1999, 2048 CUDA cores, 200-275 TOPS AI

**Recommendation**: Orin NX 16GB for development (balance of performance + cost)

**Purchase**: https://www.nvidia.com/en-us/autonomous-machines/embedded-systems/jetson-orin/

**Setup**:
```bash
# Flash JetPack 5.1+ using NVIDIA SDK Manager
# Download: https://developer.nvidia.com/sdk-manager

# On Jetson (after flashing):
# Install ROS 2 Humble (ARM64 binaries)
sudo apt install -y ros-humble-desktop

# Install Isaac Sim for Jetson (lighter image)
docker pull nvcr.io/nvidia/isaac-sim:4.0.0-jetson

# Test GPU
sudo tegrastats
# Should show GPU/CPU utilization
```

#### Development Accessories

**Mounting Hardware**:
- **Camera Mount**: Tripod or robotic arm clamp ($20-50)
- **Jetson Case**: Waveshare metal case with fan ($30)
- **Power Supply**: USB-C PD 65W for Jetson ($25)

**Networking**:
- **WiFi 6 Router**: For low-latency ROS 2 communication ($80-150)
  - Recommended: TP-Link AX3000, ASUS RT-AX55
- **Ethernet Switch**: If using wired (Gigabit recommended) ($20-40)

**Total Development Setup Cost**: $2500-4000

---

## 3. Production Setup (Full Humanoid Robot)

**Purpose**: Real-world deployment, research, commercial applications

### Humanoid Robot Platforms

#### Option A: Unitree H1 Humanoid

**Specs**:
- Height: 180 cm, Weight: 47 kg
- DoF: 19 (legs: 12, arms: 6, head: 1)
- Actuators: Unitree motors (peak 360 Nm)
- Battery: 15 Ah (2 hours operation)
- Compute: Jetson AGX Orin (32GB)
- Sensors: RealSense D435i, IMU, force sensors in feet
- Connectivity: Ethernet, WiFi, 4G optional

**Price**: ~$90,000 USD (2024 estimate, contact Unitree for quote)

**Purchase**: https://www.unitree.com/ (B2B sales, requires application)

**Pros**:
- Full ROS 2 support out-of-box
- Open SDK (Python/C++ APIs)
- Strong community (research labs worldwide)
- Spare parts available

**Cons**:
- Expensive
- Long lead time (3-6 months)
- Export restrictions (check your country)

#### Option B: Agility Robotics Digit

**Specs**:
- Height: 160 cm, Weight: 46 kg
- DoF: 20 (legs: 12, arms: 4, torso: 4)
- Battery: 2-3 hours
- Payload: 16 kg
- Sensors: LiDAR, cameras, IMU

**Price**: ~$250,000 USD (lease options available)

**Purchase**: https://agilityrobotics.com/ (commercial sales)

**Pros**:
- Production-grade reliability
- Warehouse-tested (Amazon partnership)
- Excellent support

**Cons**:
- Very expensive
- Closed software stack (limited customization)
- Long-term contracts

#### Option C: DIY Open-Source Humanoid

**Projects**:
- **Open Dynamic Robot Initiative** (Max Planck Institute): https://open-dynamic-robot-initiative.github.io/
  - Cost: $15,000-30,000 (parts + machining)
  - Difficulty: Expert (requires mechanical engineering)
- **THOR (UCLA)**: Open-source humanoid design
  - Cost: $20,000-40,000
  - Difficulty: Advanced

**Pros**:
- Full control over design
- Educational value
- Community support

**Cons**:
- Requires machining/fabrication access
- Time-intensive (6-12 months build)
- No warranty or support

### Production Compute Setup

#### Onboard Compute (Robot)
- **Jetson AGX Orin 64GB**: $1999 (AI inference, perception)
- **Intel NUC 12 Pro** (i7, 64GB RAM): $1200 (high-level planning, ROS 2 coordination)

#### Offboard Compute (Workstation/Server)
- **Training Server**:
  - CPU: AMD Threadripper 3970X (32 cores): $2000
  - GPU: 2x RTX 4090 (24GB VRAM each): $4000
  - RAM: 128GB DDR4: $400
  - Storage: 4TB NVMe SSD: $300
  - **Total**: ~$7000
  - **Use**: Isaac Lab RL training, large-scale data processing

#### Networking
- **5G/WiFi 6E Router**: Low-latency teleoperation ($200-400)
- **Edge Server**: For local LLM inference (avoid cloud latency)
  - Mini PC with RTX 4060 Ti: $1200
  - Run Llama 3 70B or GPT-4o via API

**Total Production Setup Cost**: $100,000-300,000 (robot + compute + accessories)

---

## 4. Recommended Purchase Order (Development Setup)

### Phase 1: Foundations ($1500)
1. Desktop workstation OR gaming laptop with RTX GPU ($1200)
2. External SSD for datasets (1TB): $80
3. USB 3.0 hub (powered): $30
4. Monitor (if desktop): $150-300

### Phase 2: Sensors ($500-800)
1. Intel RealSense D435i: $329
2. Camera tripod/mount: $40
3. Jetson Orin Nano 8GB Dev Kit: $399 (optional, if want edge testing)

### Phase 3: Scaling ($1000+)
1. Upgrade to Jetson Orin NX 16GB: $599
2. Additional RealSense for stereo vision: $329
3. LiDAR (RPLiDAR A1): $99 (for navigation testing)
4. Force-torque sensor (6-axis, Robotiq FT 300): $4000 (advanced manipulation)

---

## 5. Maintenance and Upgrades

### Regular Maintenance

**Software**:
- Update Ubuntu packages: `sudo apt update && sudo apt upgrade` (monthly)
- Update ROS 2 packages: `sudo apt install ros-humble-desktop` (quarterly)
- Update Isaac Sim: Pull latest Docker image (when new features needed)

**Hardware**:
- Clean GPU fans: Every 6 months (compressed air)
- Check cable connections: Monthly (USB 3.0 can become loose)
- Calibrate cameras: After any physical movement

### Upgrade Path

**Year 1-2**: Development Setup
- Workstation + RealSense + Jetson Orin Nano
- Focus: Learning, algorithm development in simulation

**Year 2-3**: Prototype Hardware
- Add: Robotic arm (UR5e: $20,000 or Franka Emika Panda: $25,000)
- Purpose: Test manipulation policies on real hardware

**Year 3+**: Full Humanoid
- Unitree H1 or custom build
- Purpose: End-to-end system deployment

---

## 6. Troubleshooting Hardware Issues

**GPU not detected in Docker**:
```bash
# Verify nvidia-docker2 installed
docker run --rm --gpus all nvidia/cuda:12.1.0-base-ubuntu22.04 nvidia-smi

# If fails, reinstall NVIDIA Container Toolkit
sudo apt purge nvidia-docker2
sudo apt install -y nvidia-docker2
sudo systemctl restart docker
```

**RealSense camera not found**:
```bash
# Check USB 3.0 connection (blue port)
lsusb | grep Intel
# Should show "Intel Corp. RealSense"

# Check permissions
sudo usermod -aG video $USER
# Log out and back in

# Reset camera
sudo systemctl restart udev
```

**Jetson boot issues**:
- Use official NVIDIA power supply (65W USB-C PD)
- Flash with latest JetPack via SDK Manager
- Check SD card/NVMe SSD seating

---

## 7. Safety Equipment (Production Setups)

**For Humanoid Robots**:
- **Emergency Stop Button** (wireless): $200-500
  - Mount on robot torso (easily accessible)
  - Wired backup on workstation
- **Safety Enclosure**: $5000-20,000 (if testing dangerous behaviors)
  - Plexiglass walls, padded floor
- **Personal Protective Equipment**:
  - Safety glasses (flying debris from dropped objects)
  - Steel-toe boots (robot feet can cause injury)
  - Hearing protection (some motors are loud)

**Electrical Safety**:
- Surge protector: $50-100
- Uninterruptible Power Supply (UPS): $200-500 (prevents data loss during power outages)

---

## Budget Summary Table

| Setup Tier | Hardware Cost | Annual Software/Cloud | Total Year 1 |
|------------|---------------|----------------------|--------------|
| **Minimal** (Simulation Only) | $1,200-2,000 | $0-500 (cloud GPU optional) | $1,200-2,500 |
| **Development** (Sensors + Workstation) | $2,500-4,000 | $200 (OpenAI API, cloud storage) | $2,700-4,200 |
| **Production** (Full Humanoid) | $100,000-300,000 | $2,000 (cloud, API, maintenance) | $102,000-302,000 |

---

## Recommended Retailers by Region

**North America**:
- Newegg, Micro Center, B&H Photo (electronics)
- RobotShop, TrossenRobotics (robotics-specific)

**Europe**:
- Scan.co.uk, Alternate.de (electronics)
- RobotShop EU, Generation Robots (robotics)

**Asia**:
- Taobao, JD.com (China)
- Akizuki Denshi, Switch Science (Japan)
- Lazada, Shopee (Southeast Asia)

**Official Vendors** (Worldwide):
- Intel (RealSense cameras)
- NVIDIA (Jetson dev kits)
- Unitree Robotics (humanoid robots)

---

**Last Updated**: 2025-12-05
**Note**: Prices are estimates as of 2024-2025 and may vary by region and availability. Always check current pricing and compatibility before purchasing.
