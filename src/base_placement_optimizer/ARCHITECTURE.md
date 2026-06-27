# Base Placement Optimizer: Architectural Deep Dive & Usage

The `base_placement_optimizer` package solves a critical problem in mobile manipulation: **Where should the robot park its mobile base so the robotic arm can safely and optimally reach a target object?**

This document details how the system achieves this using a distributed ROS 2 Action Server/Client architecture, and how to run the provided examples.

## 1. System Architecture

The optimizer operates as a standalone C++ Action Server (`optimizer_node`). It remains idle until a client (like `run_task.py` or a Behavior Tree node) requests an optimal placement over the ROS 2 DDS network. 

```mermaid
sequenceDiagram
    participant Client as run_task.py (Client)
    participant Server as optimizer_node (Server)
    participant Nav as Nav2 (Costmap)
    participant MoveIt as MoveIt 2 (Planning Scene)

    Note over Client,Server: Communication via ROS 2 Action (/optimize_placement)
    
    Client->>Server: Send OptimizePlacement.Goal (Target Pose)
    Server->>Nav: Fetch latest /global_costmap/costmap
    Nav-->>Server: Return OccupancyGrid
    
    loop For N Angular Samples
        Server->>Server: Calculate candidate Base XY
        Server->>Server: Check map safety (Cost < 253)
        alt Is Safe
            Server->>MoveIt: Validate IK & Check Collisions
            MoveIt-->>Server: Return Kinematic State
            alt IK Valid
                Server->>Server: Compute Manipulability Score
            end
        end
    end
    
    Server->>Server: Select candidate with highest score
    Server-->>Client: Return OptimizePlacement.Result (Best Base Pose)
```

## 2. Optimization Algorithm

When a goal is received, the C++ node executes a fast, multi-stage filtering algorithm to find the best parking spot.

```mermaid
graph TD
    A[Receive Target Object Pose] --> B[Generate N Angular Samples<br/>at fixed Reach Radius]
    
    B --> C[Evaluate Candidate]
    
    C --> D{1. Obstacle Check<br/>Is Costmap cell < 253?}
    D -- Hit Obstacle --> E[Discard Candidate]
    
    D -- Safe --> F{2. IK & Collision<br/>Can Arm Reach Target?}
    F -- Invalid/Collides --> E
    
    F -- Valid --> G[3. Calculate Manipulability<br/>Yoshikawa Index]
    G --> H[4. Calculate Distance<br/>Base to Target]
    H --> I[5. Compute Composite Score]
    
    I --> J{More Candidates?}
    E --> J
    
    J -- Yes --> C
    J -- No --> K[Select Highest Score]
    K --> L[Return Optimal Base Pose]
```

### The Scoring Metric
If a candidate survives the Obstacle and IK checks, it is scored using a weighted combination:

1. **Manipulability (Yoshikawa Index)**: Ensures the arm is in a dexterous, comfortable configuration, avoiding singularities (where the arm locks up) and extreme stretching.
   $$ w = \sqrt{\det(J \cdot J^T)} $$
2. **Distance**: Penalizes positions that are unnecessarily far away.

The final composite score is calculated as:
`Score = (alpha * Manipulability) + ((1 - alpha) * Distance Score)`

## 3. The ROS 2 Action Interface

The contract between the Client and Server is defined in `action/OptimizePlacement.action`. Because it is an action (not a service), the optimizer runs asynchronously and can provide feedback or be canceled mid-computation.

```yaml
# Goal: Where is the object we want to reach?
geometry_msgs/PoseStamped target_pose
---
# Result: Where should the robot park, and how good is it?
bool success
string error_reason
geometry_msgs/PoseStamped base_pose
float32 manipulability_score
float32 path_distance
float32 composite_score
---
# Feedback: What is the optimizer currently doing?
string current_phase
```

## 4. Running the Example (`run_task.py`)

To see the optimizer in action, you can run the full demo launch file and the provided Python example script.

### Step 1: Launch the Demo Environment
This launch file brings up Gazebo, Nav2, MoveIt2, and the Base Placement Optimizer Action Server.

```bash
# Build the workspace if you haven't already
colcon build --symlink-install --packages-select base_placement_optimizer

# Source the workspace
source install/setup.bash

# Launch the demo
ros2 launch base_placement_optimizer optimizer_demo.launch.py
```

### Step 2: Run the Example Script
In a new terminal, run the Python client script. This script executes a full end-to-end task:
1. **Stows the arm** for safe navigation.
2. **Calls the Optimizer** to find the best parking spot for a hardcoded target coordinate.
3. **Navigates** the mobile base to the calculated XY coordinate.
4. **Aligns the heading** using a custom P-controller for smooth rotation.
5. **Reaches for the target** using MoveIt2.

```bash
# Source the workspace
source install/setup.bash

# Run the task client
ros2 run base_placement_optimizer run_task.py
```

You will see the robot intelligently navigate to an optimal position and reach for the target coordinate.
