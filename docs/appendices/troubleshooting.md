# Troubleshooting Guide

**Consolidated troubleshooting tips and solutions for common issues across all modules**

---

## Table of Contents

1. [ROS 2 Issues](#1-ros-2-issues)
2. [Gazebo & Unity Simulation](#2-gazebo--unity-simulation)
3. [Isaac Sim & Isaac Lab](#3-isaac-sim--isaac-lab)
4. [Camera & Depth Sensing](#4-camera--depth-sensing)
5. [Vision Models (CLIP, SAM, Depth Anything)](#5-vision-models-clip-sam-depth-anything)
6. [LLM Integration (GPT-4o, OpenAI API)](#6-llm-integration-gpt-4o-openai-api)
7. [Docker & GPU Issues](#7-docker--gpu-issues)
8. [Network & Communication](#8-network--communication)
9. [Performance & Optimization](#9-performance--optimization)
10. [Hardware-Specific Issues](#10-hardware-specific-issues)

---

## 1. ROS 2 Issues

### Problem: `ros2` command not found

**Symptoms**:
```bash
$ ros2 topic list
bash: ros2: command not found
```

**Solutions**:
1. **Source ROS 2 setup**:
   ```bash
   source /opt/ros/humble/setup.bash
   ```
   Add to `~/.bashrc` for persistence:
   ```bash
   echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
   source ~/.bashrc
   ```

2. **Verify installation**:
   ```bash
   dpkg -l | grep ros-humble
   # Should show installed ROS 2 packages
   ```

3. **Reinstall if missing**:
   ```bash
   sudo apt update
   sudo apt install -y ros-humble-desktop
   ```

---

### Problem: No nodes discovered / DDS communication failure

**Symptoms**:
- `ros2 node list` shows no nodes (but nodes are running)
- `ros2 topic echo /topic_name` receives no data

**Causes**:
- Firewall blocking DDS ports (UDP 7400, 7401)
- Different ROS_DOMAIN_ID between nodes
- Network interface issues (multiple NICs, VPN active)

**Solutions**:

1. **Check ROS_DOMAIN_ID**:
   ```bash
   echo $ROS_DOMAIN_ID
   # Should be same across all terminals/machines (default: 0)

   # Set explicitly
   export ROS_DOMAIN_ID=0
   ```

2. **Disable firewall temporarily** (Ubuntu):
   ```bash
   sudo ufw disable
   # Test communication
   # If works, add rules instead of disabling:
   sudo ufw allow 7400/udp
   sudo ufw allow 7401/udp
   sudo ufw enable
   ```

3. **Force localhost-only communication** (single machine):
   ```bash
   export ROS_LOCALHOST_ONLY=1
   # Add to ~/.bashrc if always working on single machine
   ```

4. **Check network interfaces**:
   ```bash
   ip addr
   # If multiple interfaces (e.g., eth0, wlan0, docker0), force one:
   export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
   export CYCLONEDDS_URI='<CycloneDDS><Domain><General><NetworkInterfaceAddress>127.0.0.1</NetworkInterfaceAddress></General></Domain></CycloneDDS>'
   ```

---

### Problem: `colcon build` fails with C++ compilation errors

**Symptoms**:
```
CMake Error: ... missing dependency ...
```

**Solutions**:

1. **Install missing dependencies**:
   ```bash
   cd ~/ros2_ws  # Your workspace
   rosdep install --from-paths src --ignore-src -r -y
   ```

2. **Clean build artifacts**:
   ```bash
   rm -rf build/ install/ log/
   colcon build
   ```

3. **Check package.xml dependencies** match CMakeLists.txt

4. **Update colcon**:
   ```bash
   sudo apt install -y python3-colcon-common-extensions
   ```

---

## 2. Gazebo & Unity Simulation

### Problem: Gazebo runs very slow (less than 10 FPS)

**Symptoms**:
- Real-time factor less than 0.5 (shown in Gazebo GUI bottom-left)
- High CPU usage (100% on all cores)

**Solutions**:

1. **Reduce physics iteration rate**:
   - Edit world file: Change `<max_step_size>0.001</max_step_size>` to `0.01` (10ms)
   - Trade-off: Less accurate physics

2. **Disable GUI** (run headless):
   ```bash
   gazebo --verbose -s libgazebo_ros_factory.so world.sdf
   # No rendering, faster simulation
   ```

3. **Simplify collision geometry**:
   - Use primitive shapes (box, sphere) instead of meshes
   - Reduce polygon count in visual meshes

4. **Limit sensor update rates**:
   ```xml
   <update_rate>10</update_rate>  <!-- Instead of 30 Hz -->
   ```

---

### Problem: Robot falls through ground plane in Gazebo

**Symptoms**:
- Robot spawns, immediately drops below Z=0
- No collision with ground

**Causes**:
- Ground plane not loaded
- Collision geometry missing in robot model

**Solutions**:

1. **Add ground plane to world**:
   ```xml
   <include>
     <uri>model://ground_plane</uri>
   </include>
   ```

2. **Verify robot URDF has collision tags**:
   ```xml
   <collision>
     <geometry>
       <box size="0.5 0.3 0.2"/>
     </geometry>
   </collision>
   ```

3. **Check inertia values** (very small inertia can cause instability):
   ```xml
   <inertial>
     <mass>1.0</mass>  <!-- Not 0.0001 -->
     <inertia>
       <ixx>0.1</ixx>  <!-- Reasonable values -->
       ...
     </inertia>
   </inertial>
   ```

---

### Problem: Unity ML-Agents: "No module named mlagents"

**Symptoms**:
```bash
$ mlagents-learn config.yaml --run-id=test
bash: mlagents-learn: command not found
```

**Solutions**:

1. **Install ML-Agents Python package**:
   ```bash
   pip3 install mlagents==0.30.0
   # Verify
   mlagents-learn --help
   ```

2. **If Unity package not found**:
   - Open Unity Package Manager
   - Add package from git URL: `com.unity.ml-agents`
   - Version 2.3.0+ recommended

3. **Check Python version**:
   ```bash
   python3 --version
   # ML-Agents requires 3.8-3.10 (not 3.11+)
   ```

---

## 3. Isaac Sim & Isaac Lab

### Problem: Isaac Sim fails to start with GPU error

**Symptoms**:
```
[Error] Failed to create Vulkan instance
[Error] GPU not detected
```

**Solutions**:

1. **Verify NVIDIA driver installed**:
   ```bash
   nvidia-smi
   # Should show driver version 535+
   ```

2. **Check CUDA compatibility**:
   ```bash
   nvcc --version
   # Isaac Sim 4.0 requires CUDA 12.1+
   ```

3. **Update driver**:
   ```bash
   sudo ubuntu-drivers autoinstall
   sudo reboot
   ```

4. **Docker-specific**: Ensure `--gpus all` flag used:
   ```bash
   docker run --rm --gpus all nvidia/cuda:12.1.0-base nvidia-smi
   # Should work before trying Isaac Sim
   ```

---

### Problem: Isaac Lab RL training: reward not increasing

**Symptoms**:
- Training runs for 500+ epochs
- Average return stays negative or constant
- Policy doesn't improve

**Causes**:
- Reward function poorly scaled
- Observation not normalized
- Learning rate too high/low
- Episode terminates too quickly

**Solutions**:

1. **Check reward scale**:
   ```python
   # Print rewards during first 10 episodes
   print(f"Reward components: upright={upright_reward:.3f}, energy={energy_penalty:.3f}")
   # If one component dominates (e.g., energy = -10000, upright = 0.5), rescale
   ```

2. **Enable observation normalization** (CRITICAL):
   ```python
   config = {
       "normalize_input": True,  # Must be True
       "normalize_value": True,
   }
   ```

3. **Adjust learning rate**:
   ```python
   # Start high, decrease if unstable
   "lr": 3e-4,  # Default for PPO
   # If diverging (NaN loss), try 1e-4
   # If too slow, try 5e-4
   ```

4. **Increase episode length**:
   ```python
   episode_length_s = 20.0  # Give policy more time to learn
   ```

5. **Visualize in TensorBoard**:
   ```bash
   tensorboard --logdir=runs/
   # Check if return, policy loss, value loss change over time
   ```

---

### Problem: Isaac Sim crashes when loading URDF

**Symptoms**:
```
[Error] Failed to import URDF
Segmentation fault (core dumped)
```

**Solutions**:

1. **Validate URDF syntax**:
   ```bash
   check_urdf robot.urdf
   # Install if missing: sudo apt install liburdfdom-tools
   ```

2. **Check mesh file paths**:
   ```xml
   <mesh filename="package://my_robot/meshes/base.stl"/>
   <!-- Ensure package path resolves correctly -->
   ```

3. **Convert to USD first** (more stable):
   ```bash
   # In Isaac Sim Python:
   from omni.isaac.urdf import _urdf
   _urdf.acquire_urdf_interface().import_urdf(
       "robot.urdf", "/World/Robot", urdf_to_usd=True
   )
   # Saves as robot.usd, load that instead
   ```

4. **Simplify URDF**:
   - Remove complex meshes, use primitives (box, cylinder) for testing
   - Reduce joint count to minimal subset

---

## 4. Camera & Depth Sensing

### Problem: RealSense camera not detected (`rs-enumerate-devices` shows nothing)

**Symptoms**:
```bash
$ rs-enumerate-devices
No device detected. Is it plugged in?
```

**Solutions**:

1. **Check USB 3.0 connection**:
   - Use **blue** USB port (USB 3.0), not black (USB 2.0)
   - Try different USB port
   - Avoid USB hubs (direct connection to motherboard preferred)

2. **Verify device in lsusb**:
   ```bash
   lsusb | grep Intel
   # Should show: Intel Corp. RealSense
   ```

3. **Update udev rules**:
   ```bash
   cd ~/librealsense
   sudo cp config/99-realsense-libusb.rules /etc/udev/rules.d/
   sudo udevadm control --reload-rules && sudo udevadm trigger
   ```

4. **Check permissions**:
   ```bash
   sudo usermod -aG video $USER
   # Log out and back in
   ```

5. **Reinstall librealsense**:
   ```bash
   sudo apt remove librealsense2-*
   sudo apt install -y ros-humble-realsense2-camera
   ```

---

### Problem: Depth image is noisy or has holes

**Symptoms**:
- Depth map shows black regions (invalid depth)
- Objects at close range not detected

**Causes**:
- Insufficient lighting
- Reflective/transparent surfaces
- Out of sensor range (D435i: 0.3m-10m)

**Solutions**:

1. **Improve lighting**:
   - Add diffuse lighting (avoid direct sunlight)
   - For dark objects, add IR illumination (RealSense has built-in IR emitter)

2. **Enable post-processing filters** (ROS 2 launch args):
   ```bash
   ros2 launch realsense2_camera rs_launch.py \
     enable_depth:=true \
     spatial_filter.enable:=true \
     temporal_filter.enable:=true \
     hole_filling_filter.enable:=true
   ```

3. **Adjust depth accuracy vs range trade-off**:
   ```bash
   # High accuracy (0.3-3m)
   ros2 launch realsense2_camera rs_launch.py depth_module.profile:=640x480x30 depth_module.visual_preset:=3

   # High range (3-10m)
   ros2 launch realsense2_camera rs_launch.py depth_module.visual_preset:=1
   ```

4. **For transparent objects**: Use monocular depth (Depth Anything) instead of stereo

---

### Problem: Camera calibration fails (checkerboard not detected)

**Symptoms**:
- Calibration GUI shows "No checkerboard found"
- Coverage bars don't increase

**Solutions**:

1. **Print larger checkerboard**:
   - Recommended: 9x6 squares, 25mm square size (A3 paper)
   - Ensure high-contrast printing (black squares fully black)

2. **Improve lighting**:
   - Diffuse, even lighting (no shadows on checkerboard)
   - Avoid glare (matte paper, not glossy)

3. **Hold checkerboard flat**:
   - Mount on rigid board (not floppy paper)
   - Ensure all corners visible

4. **Verify checkerboard size in calibration command**:
   ```bash
   ros2 run camera_calibration cameracalibrator --size 9x6 --square 0.025
   # Size must match your printed pattern
   ```

5. **Move slowly**:
   - GUI needs time to detect pattern (2-3 seconds per position)
   - Wait for green rectangle around detected checkerboard

---

## 5. Vision Models (CLIP, SAM, Depth Anything)

### Problem: CLIP scores too low for obvious object matches

**Symptoms**:
```
Query: "red mug"
Detected: mug at (0.5, 0.3, 0.8) with confidence 0.15
# 0.15 is very low, should be >0.3 for clear matches
```

**Solutions**:

1. **Improve image quality**:
   - Better lighting (avoid shadows, glare)
   - Higher resolution (1280x720 instead of 640x480)
   - Sharp focus (calibrate camera autofocus)

2. **Refine text query**:
   ```python
   # Instead of:
   query = "mug"

   # Try:
   query = "a red ceramic coffee mug on a table"
   # More context helps CLIP
   ```

3. **Use CLIP ViT-L instead of ViT-B** (larger model, better accuracy):
   ```python
   model = CLIPModel.from_pretrained("openai/clip-vit-large-patch14")
   # Trade-off: 2x slower
   ```

4. **Ensemble multiple prompts**:
   ```python
   queries = ["red mug", "coffee cup red", "ceramic mug"]
   scores = [clip_score(img, q) for q in queries]
   final_score = max(scores)
   ```

---

### Problem: SAM segments background instead of object

**Symptoms**:
- Clicking on object produces mask of table/wall
- Multiple unintended masks generated

**Solutions**:

1. **Use point prompt more precisely**:
   ```python
   # Click center of object, not edge
   point_coords = [[object_center_x, object_center_y]]
   point_labels = [1]  # 1 = foreground
   ```

2. **Add negative points** (background):
   ```python
   point_coords = [[obj_x, obj_y], [bg_x, bg_y]]
   point_labels = [1, 0]  # 1=foreground, 0=background
   ```

3. **Use bounding box prompt** (more stable):
   ```python
   # If you have rough object location
   box = [x_min, y_min, x_max, y_max]
   masks = predictor.predict(box=box)
   ```

4. **Adjust SAM parameters**:
   ```python
   mask_generator = SamAutomaticMaskGenerator(
       model=sam,
       min_mask_region_area=100,  # Ignore tiny regions
       stability_score_thresh=0.9,  # Higher = fewer false positives
   )
   ```

---

### Problem: Depth Anything outputs unrealistic depths

**Symptoms**:
- Depth map shows objects 50m away (clearly wrong)
- Relative depth correct, but absolute scale wrong

**Causes**:
- Depth Anything outputs **relative depth** (not metric)
- Scaling factor depends on scene

**Solutions**:

1. **Calibrate with known distance**:
   ```python
   # Place object at measured distance (e.g., 1.0m)
   depth_map = depth_estimator.estimate_depth(image)
   center_depth = depth_map[center_y, center_x]

   # Compute scale factor
   scale = 1.0 / center_depth
   depth_map_scaled = depth_map * scale
   # Now depths are in meters
   ```

2. **Fuse with hardware depth** (RealSense):
   ```python
   # Where RealSense is valid (0.3-10m), use it
   # Elsewhere, use scaled Depth Anything
   depth_fused = np.where(depth_hw > 0, depth_hw, depth_monocular)
   ```

3. **Use depth ranges per scene**:
   ```python
   # For indoor scenes, clip to reasonable range
   depth_map = np.clip(depth_map, 0.1, 5.0)  # 10cm - 5m
   ```

---

## 6. LLM Integration (GPT-4o, OpenAI API)

### Problem: OpenAI API rate limit errors

**Symptoms**:
```
RateLimitError: You exceeded your current quota
```

**Solutions**:

1. **Check API usage**:
   - https://platform.openai.com/usage
   - Ensure you have credits (pay-as-you-go) or within free tier limits

2. **Add retry logic with exponential backoff**:
   ```python
   from openai import OpenAI
   import time

   client = OpenAI()
   max_retries = 3

   for i in range(max_retries):
       try:
           response = client.chat.completions.create(...)
           break
       except RateLimitError:
           if i < max_retries - 1:
               time.sleep(2 ** i)  # 1s, 2s, 4s
           else:
               raise
   ```

3. **Reduce frequency**:
   - Cache LLM responses (if same query repeated)
   - Batch multiple queries into one API call

4. **Use cheaper model for non-critical tasks**:
   ```python
   # For simple planning:
   model = "gpt-4o-mini"  # 15x cheaper than gpt-4o
   ```

---

### Problem: LLM generates invalid Python code

**Symptoms**:
- Code contains syntax errors
- Uses undefined functions
- Logic doesn't match command

**Solutions**:

1. **Improve system prompt**:
   ```python
   system_prompt = """
   You are a robot control planner. Output ONLY valid Python code.

   Available functions:
   - move_arm(x, y, z) - floats in meters
   - grasp() - no arguments

   Rules:
   1. Do NOT import any libraries
   2. Do NOT use undefined functions
   3. Always check if detect_object() returns None before using result

   Example:
   obj = detect_object("mug")
   if obj:
       move_arm(obj["x"], obj["y"], obj["z"])
       grasp()
   """
   ```

2. **Validate code before execution**:
   ```python
   import ast

   try:
       ast.parse(code)  # Check syntax
   except SyntaxError as e:
       print(f"Invalid code generated: {e}")
       # Retry or ask user
   ```

3. **Use lower temperature**:
   ```python
   response = client.chat.completions.create(
       model="gpt-4o",
       temperature=0.1,  # More deterministic (less creative)
       ...
   )
   ```

4. **Add few-shot examples**:
   ```python
   messages = [
       {"role": "system", "content": system_prompt},
       {"role": "user", "content": "Pick up the red ball"},
       {"role": "assistant", "content": "obj = detect_object('red ball')\nif obj:\n    move_arm(obj['x'], obj['y'], obj['z'])\n    grasp()"},
       {"role": "user", "content": user_command},  # Actual command
   ]
   ```

---

## 7. Docker & GPU Issues

### Problem: Docker can't access GPU (`nvidia-smi` fails in container)

**Symptoms**:
```bash
$ docker run --rm --gpus all nvidia/cuda:12.1.0-base nvidia-smi
docker: Error response from daemon: could not select device driver
```

**Solutions**:

1. **Install NVIDIA Container Toolkit**:
   ```bash
   distribution=$(. /etc/os-release;echo $ID$VERSION_ID)
   curl -s -L https://nvidia.github.io/nvidia-docker/gpgkey | sudo apt-key add -
   curl -s -L https://nvidia.github.io/nvidia-docker/$distribution/nvidia-docker.list | sudo tee /etc/apt/sources.list.d/nvidia-docker.list

   sudo apt update
   sudo apt install -y nvidia-docker2
   sudo systemctl restart docker
   ```

2. **Verify Docker daemon config**:
   ```bash
   cat /etc/docker/daemon.json
   # Should contain:
   {
     "runtimes": {
       "nvidia": {
         "path": "nvidia-container-runtime",
         "runtimeArgs": []
       }
     }
   }
   ```

3. **Test with simple container**:
   ```bash
   docker run --rm --gpus all nvidia/cuda:12.1.0-base-ubuntu22.04 nvidia-smi
   # Should show GPU info
   ```

---

### Problem: Docker X11 display not working (GUI apps)

**Symptoms**:
```bash
$ docker run -e DISPLAY=$DISPLAY ...
Error: cannot open display
```

**Solutions**:

1. **Allow X11 access**:
   ```bash
   xhost +local:docker
   ```

2. **Mount X11 socket**:
   ```bash
   docker run --rm -it \
     -e DISPLAY=$DISPLAY \
     -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
     your-image
   ```

3. **For WSL2 (Windows)**:
   ```bash
   # Install VcXsrv or X410 on Windows
   # In WSL2:
   export DISPLAY=$(cat /etc/resolv.conf | grep nameserver | awk '{print $2}'):0
   ```

---

## 8. Network & Communication

### Problem: ROS 2 nodes on different machines can't communicate

**Symptoms**:
- Robot on Machine A, workstation on Machine B
- `ros2 node list` on A doesn't show nodes from B

**Solutions**:

1. **Ensure same ROS_DOMAIN_ID**:
   ```bash
   # On both machines:
   export ROS_DOMAIN_ID=42  # Any number 0-101
   ```

2. **Disable ROS_LOCALHOST_ONLY**:
   ```bash
   # On both machines:
   export ROS_LOCALHOST_ONLY=0
   ```

3. **Check firewall**:
   ```bash
   # On both machines:
   sudo ufw allow from <other_machine_IP>
   # Or disable for testing:
   sudo ufw disable
   ```

4. **Use multicast-friendly network**:
   - DDS (ROS 2 default) uses multicast
   - Some WiFi routers block multicast → Use wired Ethernet
   - Or switch to unicast DDS:
     ```bash
     export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
     export CYCLONEDDS_URI='<CycloneDDS><Domain><Discovery><Peers><Peer Address="<other_machine_IP>"/></Peers></Discovery></Domain></CycloneDDS>'
     ```

---

## 9. Performance & Optimization

### Problem: High CPU usage, low FPS during vision inference

**Symptoms**:
- CPU at 100%, GPU at 20%
- Object detection takes 5+ seconds

**Causes**:
- Models running on CPU instead of GPU
- Large image sizes

**Solutions**:

1. **Force GPU usage**:
   ```python
   import torch
   device = "cuda" if torch.cuda.is_available() else "cpu"
   model = model.to(device)
   inputs = {k: v.to(device) for k, v in inputs.items()}
   ```

2. **Verify GPU usage**:
   ```bash
   nvidia-smi
   # Check "GPU-Util" column during inference (should be >80%)
   ```

3. **Reduce image resolution**:
   ```python
   # Resize before inference
   image = cv2.resize(image, (640, 480))  # Instead of 1920x1080
   ```

4. **Use quantized models**:
   ```python
   # INT8 quantization (3x faster, less than 1% accuracy drop)
   model = torch.quantization.quantize_dynamic(
       model, {torch.nn.Linear}, dtype=torch.qint8
   )
   ```

---

## 10. Hardware-Specific Issues

### Jetson Orin: Out of memory during RL training

**Symptoms**:
```
RuntimeError: CUDA out of memory. Tried to allocate 2.00 GiB
```

**Solutions**:

1. **Reduce parallel environments**:
   ```python
   num_envs = 256  # Instead of 4096 (desktop GPU)
   ```

2. **Enable memory swap** (Jetson):
   ```bash
   sudo systemctl disable nvzramconfig
   sudo fallocate -l 8G /swapfile
   sudo chmod 600 /swapfile
   sudo mkswap /swapfile
   sudo swapon /swapfile
   # Make permanent:
   echo '/swapfile none swap sw 0 0' | sudo tee -a /etc/fstab
   ```

3. **Use smaller models**:
   ```python
   # Reduce policy network size
   mlp_units = [128, 128]  # Instead of [256, 256, 128]
   ```

---

### Windows WSL2: USB devices not accessible

**Symptoms**:
- RealSense camera works on Windows, not in WSL2
- `lsusb` doesn't show device

**Solutions**:

1. **Use USBIPD** (Windows 11):
   ```powershell
   # In PowerShell (Admin):
   winget install usbipd
   usbipd list  # Find camera
   usbipd bind --busid X-Y  # Replace X-Y with camera bus ID
   usbipd attach --wsl --busid X-Y
   ```

2. **In WSL2**:
   ```bash
   lsusb  # Should now show camera
   ```

3. **Alternative**: Run RealSense node on Windows, communicate with WSL2 ROS 2 nodes via network (same machine, different ROS_DOMAIN_ID not needed)

---

## Quick Reference: Error Codes

| Error Code | Component | Common Cause | Quick Fix |
|------------|-----------|--------------|-----------|
| `ros2: command not found` | ROS 2 | Not sourced | `source /opt/ros/humble/setup.bash` |
| `CUDA out of memory` | Isaac Lab | Too many envs | Reduce `num_envs` |
| `RateLimitError` | OpenAI API | Exceeded quota | Add credits or use gpt-4o-mini |
| `No device detected` | RealSense | USB connection | Use USB 3.0 port (blue) |
| `Segmentation fault` | Isaac Sim | Invalid URDF | Validate with `check_urdf` |
| `could not select device driver` | Docker | NVIDIA runtime missing | Install nvidia-docker2 |
| `No nodes discovered` | ROS 2 DDS | Firewall | Allow UDP 7400-7401 |

---

## Getting Help

If issue persists:

1. **Check official documentation**:
   - ROS 2: https://docs.ros.org/en/humble/
   - Isaac Sim: https://docs.omniverse.nvidia.com/isaacsim/
   - RealSense: https://dev.intelrealsense.com/

2. **Search community forums**:
   - ROS Answers: https://answers.ros.org/
   - NVIDIA Forums: https://forums.developer.nvidia.com/c/omniverse/simulation/69
   - Stack Overflow: Tag `ros2`, `isaac-sim`, `realsense`

3. **Provide debug info when asking**:
   - OS version: `lsb_release -a`
   - ROS 2 version: `ros2 --version`
   - GPU info: `nvidia-smi`
   - Full error traceback (not just last line)
   - Minimal reproduction steps

4. **GitHub Issues** (for bugs):
   - ROS 2: https://github.com/ros2/ros2/issues
   - Isaac Lab: https://github.com/isaac-sim/IsaacLab/issues
   - RealSense ROS: https://github.com/IntelRealSense/realsense-ros/issues

---

**Last Updated**: 2025-12-05
**Maintained By**: Physical AI Textbook Contributors
**Contributions Welcome**: Submit issues or PRs to improve this guide!
