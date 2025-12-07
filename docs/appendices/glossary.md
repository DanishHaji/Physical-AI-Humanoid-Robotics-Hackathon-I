# Glossary

**Comprehensive glossary of terms used throughout the Physical AI & Humanoid Robotics textbook, organized alphabetically.**

---

## A

**Action Model**: Component of a VLA system that translates high-level plans into low-level robot control commands (joint positions, gripper commands, wheel velocities).

**Action Space**: In reinforcement learning, the set of all possible actions an agent can take (e.g., 12-dimensional continuous space for humanoid joint positions).

**Actuator**: A mechanical device that converts energy (electrical, hydraulic, pneumatic) into motion, such as motors that drive robot joints.

**A2C (Advantage Actor-Critic)**: A reinforcement learning algorithm that uses both a policy network (actor) and a value network (critic) to improve learning efficiency.

**Articulation**: A robotic system with joints connecting rigid bodies (links), represented in Isaac Sim as a connected hierarchy with constraints.

**Autonomous Navigation**: The capability of a robot to plan and execute paths from one location to another without human intervention, avoiding obstacles.

---

## B

**Baseline (ROS 2)**: In robotics, often refers to the distance between stereo camera lenses, affecting depth accuracy; also can mean a reference implementation for comparison.

**Bounding Box**: A rectangular region in an image that encloses a detected object, typically defined by (x, y, width, height).

---

## C

**Calibration (Camera)**: The process of determining intrinsic parameters (focal length, principal point, distortion) and extrinsic parameters (position, orientation) of a camera for accurate 3D reconstruction.

**CLIP (Contrastive Language-Image Pre-training)**: OpenAI's vision-language model that learns joint embeddings of images and text, enabling zero-shot object detection by matching text queries to image regions.

**Closed-Loop Control**: A control system that uses feedback from sensors to adjust actions in real-time, correcting for errors and disturbances (contrasts with open-loop control).

**Colcon**: The build system for ROS 2 packages, replacing catkin from ROS 1, used to compile C++/Python code and manage dependencies.

**Contact Sensor**: A sensor that detects physical contact between robot parts and the environment, measuring forces or binary contact state (touching/not touching).

**Control Loop**: The cycle of sensing, computing, and actuating that runs continuously in a robot system, typically at high frequency (100-1000 Hz).

**CUDA**: NVIDIA's parallel computing platform that enables GPU acceleration for general-purpose computing, essential for deep learning and physics simulation.

---

## D

**DDS (Data Distribution Service)**: The middleware standard used by ROS 2 for real-time publish-subscribe communication between distributed nodes.

**Depth Anything**: State-of-the-art monocular depth estimation model using vision transformers, trained on millions of RGB-depth pairs to predict dense depth maps from single images.

**Depth Map**: An image where each pixel value represents the distance from the camera to the corresponding point in the scene, typically in meters.

**Digital Twin**: A virtual replica of a physical robot or environment used for simulation, testing, and training before deploying to real hardware.

**Domain Randomization**: A sim-to-real transfer technique that randomizes simulation parameters (physics, visuals, sensor noise) during training to prevent overfitting and improve real-world robustness.

**DoF (Degrees of Freedom)**: The number of independent parameters defining a robot's configuration; a humanoid arm typically has 7 DoF (3 shoulder, 1 elbow, 3 wrist).

---

## E

**Embodied AI**: Artificial intelligence systems that interact with the physical world through sensors and actuators (robots), requiring grounding of abstract knowledge in physical experience. Also called Physical AI.

**End-Effector**: The device at the end of a robotic arm that interacts with the environment, such as a gripper, welding tool, or camera.

**Episode**: In reinforcement learning, a sequence of states, actions, and rewards from an initial state to a terminal state (e.g., robot attempts task from start to success/failure).

---

## F

**Focal Length (Camera)**: The distance from the camera lens to the image sensor, determining the field of view and magnification; key parameter for pinhole camera model.

**Force Sensor**: A sensor that measures applied forces (and sometimes torques), used in robotic grippers to detect object contact and prevent damage.

**Forward Kinematics**: Computing the end-effector position and orientation from given joint angles using robot geometry and link transformations.

---

## G

**GAE (Generalized Advantage Estimation)**: An RL algorithm component (parameterized by λ) that balances bias vs variance in advantage estimation for policy gradient updates, used in PPO.

**Gazebo**: An open-source 3D robot simulator with physics engines (ODE, Bullet, DART), widely used in ROS ecosystem for testing navigation and manipulation.

**GPU-Accelerated Physics**: Using graphics processing units (GPUs) to parallelize physics simulation (collision detection, constraint solving), achieving 100-1000x speedup over CPU-based simulators.

**Grasp Planning**: The process of determining where and how a robot gripper should approach and grip an object to achieve a stable grasp.

**Grounding Problem**: Mapping natural language references (e.g., "the red mug") to specific physical entities in the environment, resolving ambiguities through spatial reasoning and context.

---

## H

**Headless Mode**: Running a simulator (Isaac Sim, Gazebo) without a graphical user interface (GUI), useful for faster execution on servers or edge devices.

**URDF (Unified Robot Description Format)**: An XML-based file format for describing robot kinematics, dynamics, collision geometry, and visual appearance in ROS.

---

## I

**IMU (Inertial Measurement Unit)**: A sensor that measures linear acceleration and angular velocity, used for estimating robot orientation and motion.

**Inverse Kinematics**: Computing joint angles required to achieve a desired end-effector position and orientation, typically solved via optimization or analytical methods.

**Isaac Lab**: NVIDIA's reinforcement learning framework for robotics (formerly Isaac Gym), providing vectorized GPU environments, pre-built tasks, and RL algorithm integration.

**Isaac Sim**: NVIDIA's robotics simulator built on Omniverse, featuring GPU-accelerated PhysX physics, ray-traced rendering (RTX), USD scene format, and ROS 2 integration.

---

## J

**Joint**: A connection between two robot links that allows relative motion, categorized as revolute (rotational) or prismatic (linear).

**Joint State**: The current position, velocity, and effort (torque/force) of all robot joints, published on `/joint_states` topic in ROS 2.

---

## K

**Kinematics**: The study of motion without considering forces, including forward kinematics (joint angles → end-effector pose) and inverse kinematics (end-effector pose → joint angles).

---

## L

**Latency**: The time delay between an event (e.g., sensor reading) and the corresponding response (e.g., actuator command), critical for real-time robot control.

**Launch File**: A configuration file (Python or XML) in ROS 2 that starts multiple nodes with specified parameters and remappings, simplifying system startup.

**LiDAR (Light Detection and Ranging)**: A sensor that measures distances by emitting laser pulses and measuring time-of-flight, used for 2D/3D mapping and obstacle detection.

**Link (URDF)**: A rigid body component of a robot (e.g., forearm, thigh) defined in URDF with mass, inertia, collision geometry, and visual mesh.

**LLM (Large Language Model)**: Transformer-based neural networks trained on massive text corpora (GPT-4o, Claude, LLaMA) capable of natural language understanding, reasoning, and code generation.

**Localization**: The process of determining a robot's position and orientation within a known map, often using SLAM (Simultaneous Localization and Mapping).

---

## M

**Mermaid**: A JavaScript-based diagramming tool that renders diagrams from text descriptions, natively supported in Docusaurus for flowcharts, sequence diagrams, etc.

**MoveIt2**: The motion planning framework for ROS 2, providing inverse kinematics, collision checking, trajectory generation, and execution for robotic arms.

**Multimodal Model**: A neural network that processes multiple input types (text, images, audio), such as vision-language models like CLIP or GPT-4o with vision.

---

## N

**Nav2 (Navigation 2)**: The ROS 2 navigation stack providing autonomous navigation capabilities including path planning, obstacle avoidance, and localization.

**Node (ROS 2)**: An independent process in ROS 2 that performs a specific function (e.g., camera driver, motion planner), communicating via topics, services, or actions.

---

## O

**Observation Space**: In reinforcement learning, the set of all possible observations an agent receives from the environment (e.g., 48-dimensional vector of joint states + IMU data).

**Odometry**: Estimation of robot position over time based on sensor data (wheel encoders, IMU, visual features), subject to drift without correction.

**Omniverse**: NVIDIA's platform for 3D collaboration and simulation, providing the foundation for Isaac Sim (rendering, USD support, multi-user workflows).

**Open-Loop Control**: A control system that executes commands without feedback, assuming perfect execution (contrasts with closed-loop control).

**Open-Vocabulary Detection**: Object detection that works for ANY text description without retraining, using vision-language models like CLIP (contrasts with closed-vocabulary detectors like YOLO).

---

## P

**PhysX 5**: NVIDIA's GPU-accelerated rigid body physics engine, capable of simulating thousands of robots in parallel with accurate contact dynamics and articulation solvers.

**Physical AI**: See Embodied AI. Artificial intelligence systems that perceive and interact with the physical world through sensors and actuators.

**Pinhole Camera Model**: A mathematical model relating 3D world points to 2D image pixels using focal length and principal point, fundamental to computer vision.

**Point Cloud**: A set of 3D points representing object surfaces or scenes, typically generated by depth cameras or LiDAR sensors.

**Policy (RL)**: A function (often a neural network) that maps observations to actions, representing an agent's behavior strategy in reinforcement learning.

**PPO (Proximal Policy Optimization)**: A popular on-policy RL algorithm that updates policies via clipped objective function, balancing sample efficiency and stability. Default for Isaac Lab.

**Publish-Subscribe**: A messaging pattern where publishers send messages to topics and subscribers receive them, decoupling senders from receivers (core to ROS 2).

---

## Q

**Qdrant**: A vector database optimized for similarity search, used for storing and retrieving embeddings in RAG (Retrieval-Augmented Generation) systems.

**Quaternion**: A 4-dimensional representation of 3D rotation (w, x, y, z) that avoids gimbal lock, commonly used in robotics and graphics.

---

## R

**RealSense D435i**: Intel's stereo depth camera with RGB, dual infrared cameras, and IMU, providing aligned RGB-D data for 3D perception in robotics.

**Reinforcement Learning (RL)**: A machine learning paradigm where an agent learns to maximize cumulative reward through trial-and-error interactions with an environment.

**Reward Function**: In RL, a function that assigns a scalar reward to each state-action pair, guiding the agent toward desired behaviors.

**RGB-D**: Image data combining RGB color information with depth (distance) for each pixel, enabling 3D scene understanding.

**Robot Operating System (ROS)**: An open-source middleware framework for robot software development, providing communication, hardware abstraction, and libraries. ROS 2 is the latest version with improved real-time performance and security.

**RViz**: The standard 3D visualization tool for ROS, displaying robot models, sensor data (point clouds, images), and planning results.

---

## S

**SAM (Segment Anything Model)**: Meta's foundation model for image segmentation, trained on 1 billion masks, capable of zero-shot segmentation of any object with optional prompts.

**SDF (Simulation Description Format)**: An XML format used by Gazebo to describe robot models, environments, and simulation parameters (alternative to URDF).

**Semantic Segmentation**: Classifying each pixel in an image into object categories (e.g., road, car, person), providing dense scene understanding.

**Sensor Fusion**: Combining data from multiple sensors (camera, LiDAR, IMU) to produce more accurate and robust estimates than any single sensor.

**Service (ROS 2)**: A request-response communication pattern in ROS 2, where a client sends a request and waits for a server's reply (e.g., motion planning service).

**Sim-to-Real Transfer**: The process of deploying policies trained in simulation to real robots, addressing the reality gap through domain randomization and accurate modeling.

**SLAM (Simultaneous Localization and Mapping)**: Building a map of an unknown environment while simultaneously tracking the robot's location within it.

---

## T

**Task Decomposition**: Breaking down high-level goals ("clean the table") into sequences of low-level sub-tasks ("detect objects", "grasp", "place in bin"), typically done by LLMs.

**Tensor API**: Isaac Sim's direct GPU memory access interface for observations/actions, avoiding CPU-GPU transfers. All data stays on GPU as PyTorch tensors.

**TF2 (Transform Library 2)**: ROS 2's library for managing coordinate frame transformations, maintaining a tree of spatial relationships (e.g., base_link → camera_link → gripper_link).

**Topic (ROS 2)**: A named communication channel in ROS 2's publish-subscribe pattern, carrying messages of a specific type (e.g., `/camera/image_raw` carries sensor_msgs/Image).

**Trajectory**: A time-parameterized path specifying robot joint positions/velocities at each timestep, used for smooth motion execution.

---

## U

**Unity ML-Agents**: Unity's machine learning framework enabling RL training in Unity game engine environments, popular for game AI and some robotics applications.

**Universal Scene Description (USD)**: An open-source 3D scene file format (developed by Pixar) supporting hierarchical composition, time-sampled data, materials, and lighting. Used by Isaac Sim.

**URDF (Unified Robot Description Format)**: See "H" section. XML format for robot descriptions in ROS.

---

## V

**Vectorized Environment**: Running N identical RL environments in parallel on the GPU, with batch operations (obs, actions, rewards as tensors). Enables massive throughput in Isaac Lab.

**Vision Transformer (ViT)**: Transformer architecture applied to images by dividing images into patches and processing them as sequences, enabling state-of-the-art performance (used in Depth Anything, CLIP, SAM).

**Visuomotor Policy**: A neural network that maps visual observations (camera images) directly to motor commands (joint positions), learned via imitation or reinforcement learning.

**VLA (Vision-Language-Action)**: Robotics paradigm integrating computer vision (3D perception), language models (task planning), and action models (control) to enable natural language-commanded robot behaviors.

---

## W

**Waypoint**: A discrete position along a planned path that a robot must reach, used in trajectory planning and navigation.

**Whisper**: OpenAI's speech recognition model trained on 680,000 hours of multilingual audio, achieving human-level transcription accuracy for voice-to-text conversion.

**Workspace**: In robotics, the reachable space for a robot's end-effector; also refers to a ROS 2 directory containing packages built with colcon.

---

## X

**X11**: A windowing system for Unix-like operating systems, used to display GUI applications from Docker containers on the host display.

---

## Y

**YAML (YAML Ain't Markup Language)**: A human-readable data serialization format used extensively in ROS for configuration files, launch files, and parameter definitions.

**YOLO (You Only Look Once)**: A family of real-time object detection models that process images in a single forward pass, popular for robotics vision but limited to pre-defined object classes.

---

## Z

**Zero-Shot Learning**: A machine learning paradigm where a model performs tasks it wasn't explicitly trained on, by leveraging knowledge transfer (e.g., CLIP detecting novel objects via text descriptions).

---

**Total Terms**: 100+

**Last Updated**: 2025-12-05
**Note**: Terms are drawn from all chapters of the Physical AI & Humanoid Robotics textbook. For detailed explanations and context, refer to the specific chapters where each term is introduced.
