# Ackermann Robot Simulation with Coverage Path Planning & Pure Pursuit
[![Watch the video](https://img.youtube.com/vi/5JkVA1SqY-8/maxresdefault.jpg)](https://youtu.be/5JkVA1SqY-8)

### [2D Simulation](https://youtu.be/5JkVA1SqY-8)
## Overview
[![Watch the video](https://img.youtube.com/vi/VTgKCuTHZdw/maxresdefault.jpg)](https://youtu.be/VTgKCuTHZdw)

### [3D Simulation](https://youtu.be/VTgKCuTHZdw)
This project simulates a **four-wheeled Ackermann robot** (like a car) following a **Coverage Path Planning (CPP)** path using **Pure Pursuit steering**.  

The robot navigates a rectangular area systematically using a **lawnmower (boustrophedon) pattern**, similar to an autonomous lawn mower or floor cleaning robot. The simulation also shows a real-time **terminal-like overlay** displaying the robot’s state, including linear and angular velocities.  

---

## Features

- **Lawnmower CPP Path**: automatically generates waypoints to cover the area efficiently.  
- **Pure Pursuit Controller**: calculates steering angles based on a lookahead point.  
- **Ackermann Kinematics**: realistic car-like motion using rear-axle model.  
- **Visual Simulation**: live animation showing robot body, wheels, heading, trajectory, and sensor radius.  
- **Terminal Overlay**: real-time display of robot state (position, yaw, steer, segment, linear & angular speed).  

---

## Key Concepts

| Variable | Description |
|----------|-------------|
| `rear_x, rear_y` | Position of the **rear axle** in global coordinates. |
| `yaw` | Robot orientation relative to the global X-axis (radians). |
| `steer` | Steering angle of front wheels (radians). |
| `v` | Linear velocity of the robot (m/s). |
| `omega` | Angular velocity (rad/s) around vertical axis. |
| `current_seg` | Index of the current path segment being followed. |
| `LOOKAHEAD` | Lookahead distance for Pure Pursuit (m). |
| `WAYPOINT_TOL` | Distance tolerance to consider a waypoint reached. |
| `GOAL_TOL` | Distance tolerance for reaching the final goal. |
| `WHEELBASE` | Distance between front and rear axle (m). |
| `BODY_LEN, BODY_WID` | Robot body dimensions (m). |

---

## Coverage Path Planning (CPP)

The **lawnmower pattern** covers a rectangular area using sequential horizontal passes:

`def generate_lawnmower(x_min, x_max, y_min, y_max, spacing):
    path = []
    y = y_min
    direction = 1
    while y <= y_max:
        if direction == 1:
            path.append((x_min, y))
            path.append((x_max, y))
        else:
            path.append((x_max, y))
            path.append((x_min, y))
        y += spacing
        direction *= -1
    return np.array(path, dtype=float)`
    <img width="1713" height="1248" alt="image" src="https://github.com/user-attachments/assets/689eeda3-e113-42fc-b373-1450763a64b0" />

## Pure Pursuit Controller

The **Pure Pursuit algorithm** guides the robot toward a target point located at a **lookahead distance** along the path.

### Steps:

1. **Find Lookahead Point:**  
   The function `find_lookahead_point_from()` searches along the current path segment for a point that is `LOOKAHEAD` meters ahead.

2. **Compute Lateral Offset:**  
   Transform the lookahead point into the robot's coordinate frame to compute lateral error \(y_l\).

3. **Compute Curvature:**  
<img width="194" height="127" alt="image" src="https://github.com/user-attachments/assets/80f65107-922a-4655-a19b-002e1171eedf" />

4. **Compute Steering Angle (Ackermann):**  
<img width="322" height="69" alt="image" src="https://github.com/user-attachments/assets/655fdfb3-0c59-41ab-a449-26bd4c7f3682" />

Steering angle is then limited to:  
<img width="284" height="80" alt="image" src="https://github.com/user-attachments/assets/fc02ea57-680a-4d59-ade9-34bbe0e8258a" />


---

## Ackermann Kinematics

The robot uses a **rear-axle bicycle model** to simulate realistic car-like motion:

<img width="626" height="197" alt="image" src="https://github.com/user-attachments/assets/cc6bf927-cca1-4311-9bb9-75b9cfbc1051" />


Where:

<img width="530" height="151" alt="image" src="https://github.com/user-attachments/assets/5fab487a-7c98-4669-a0a2-afa4da699bda" />


**Angular velocity:**

<img width="364" height="120" alt="image" src="https://github.com/user-attachments/assets/5ed4e332-a690-4102-b1b7-e5f267b01c4e" />


---

## Simulation Loop (`update(frame)`)

For each animation frame:

1. Compute the **lookahead point**.  
2. Compute **steering angle** using Pure Pursuit.  
3. Limit **steering rate** for smooth motion.  
4. Reduce **speed on sharp turns**.  
5. Integrate **Ackermann kinematics** to update `(rear_x, rear_y, yaw)`.  
6. Advance `current_seg` if the robot is close enough to the next waypoint.  
7. Update **visual elements**: robot body, wheels, heading, trajectory, lookahead circle.  
8. Update **terminal overlay** displaying:

   `Sim:
rear=(x.xx, y.yy)
yaw=Z deg
steer=S deg
seg=N`


