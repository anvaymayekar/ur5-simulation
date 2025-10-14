# 🤖 UR5 Robotic Arm Pick-and-Place Simulation

<br>

A comprehensive **UR5 robotic arm simulation** implementing multi-object **pick-and-place tasks** with real-time control capabilities. The project leverages MATLAB's **Robotics System Toolbox** to simulate the robot, its kinematics, and object manipulation. Users can interactively configure objects, dynamically adjust animation speed during execution, and run the simulation with a user-friendly GUI.

> 💡 Developed as part of a **robotics simulation micro-project** for the course **Fundamentals of Robotics** under **Prof. Salaba Jacob**, this demonstrates the principles of **inverse kinematics, trajectory planning, smooth path transitions, and GUI-based control** for industrial robot tasks.

---

## 🎬 Demo

![Simulation Demo](sample/demo.gif)

> The GIF demonstrates the pick-and-place simulation for multiple objects with configurable positions, real-time speed control, and smooth path transitions between sequential objects.

---

## 🧠 Code Architecture & Concepts

The simulation is implemented in a **modular and object-oriented style** with clear separation of concerns:

### 1. **Robot Initialization**
- Loads the **UR5 rigid body tree** model with gravity and row-based configuration.
- Home position is defined, and workspace boundaries are set for proper visualization with cubic aspect ratio.
- End-effector tracking allows real-time visualization of positions during the simulation.
- Enables full 3D interaction (rotate, zoom, pan) with preserved aspect ratios.

### 2. **Graphical User Interface (GUI)**
- Built using MATLAB **uicontrols** and **uipanels** with a modern dark theme.
- Interactive controls include:
  - **Number of objects** to manipulate (dynamically adjustable).
  - **Real-time animation speed slider** with live update during simulation.
  - **Object configuration table**: Editable pick and place coordinates (X, Y, Z).
  - **Quick presets**: Line, Circle, Grid, Random arrangements.
  - **Add/Remove object buttons**: Dynamically modify object count during configuration.
  - **Start/Reset buttons**: Run and stop simulation with state management.
- Status messages provide real-time feedback on current operation.
- Live end-effector position display updated every few frames.

### 3. **Pick-and-Place Logic**
- The robot moves sequentially to **pick each object** and place it at the target location.
- Smooth motion is generated using **inverse kinematics** and **trap velocity trajectory (`trapveltraj`)** for natural movement.
- **Intelligent path planning**: After placing an object, the robot automatically follows a smooth path to the next object's pick position.
- Hover heights are used to avoid collisions when moving between positions.
- Objects are represented as **colored cubes**, with their movement visually attached to the end-effector during transport.
- **Attach/detach logic**: Objects are picked up at the pick position and released at the place position with precise timing.

### 4. **Trajectory & Inverse Kinematics**
- Uses MATLAB's `inverseKinematics` object to calculate joint configurations for desired end-effector poses.
- Trajectories are **generated segment-wise**, connecting waypoints for pick and place with smooth interpolation.
- **Waypoint system**:
  - First object: Home → Above Pick → Pick → Above Pick → Above Place → Place → Above Place
  - Subsequent objects: Previous Place → Above Pick → Pick → Above Pick → Above Place → Place → Above Place
- Real-time **speed adjustment** allows dynamic control of animation velocity during execution without restarting.
- Smooth transitions between all waypoints using trapezoidal velocity profiles.

### 5. **Visualization**
- Simulation window shows:
  - 3D robot model with accurate UR5 kinematics.
  - Pick-and-place object positions (red/green markers).
  - Moving objects (colored cubes) with dynamic trails showing path history.
  - Real-time end-effector position display (X, Y, Z coordinates).
- Full **3D interaction** is enabled: rotate, zoom, and pan with preserved aspect ratio.
- Path trails visualized with unique colors for each object.
- Cubic workspace with equal aspect ratios prevents distortion during interaction.

### 6. **Dynamic Control Features**
- **Real-time speed control**: Adjust animation speed using the slider during simulation execution.
- **Multi-object support**: Add or remove objects before starting simulation.
- **Pause and reset**: Stop simulation at any time and reset to initial state.
- **Preset patterns**: Quickly load predefined object arrangements (line, circle, grid, random).
- **Editable coordinates**: Manually adjust pick and place positions in the table.

### 7. **Utilities**
- Functions for:
  - **Cube creation** (`createCube`) with vertices and faces for 3D object representation.
  - **Preset loading** (line, circle, grid, random) for easy object configuration.
  - **Dynamic add/remove object functionality** with automatic table updates.
  - **Trajectory segment calculation** for precise attach/detach timing.
  - **Real-time position updates** with frame-based throttling for performance.

---

## ⚙️ How to Run

1. **Open the function**:
   ```matlab
   UR5_PickPlace_GUI
   ```

2. **Configure the simulation**:
   - Set the number of objects (or use Add/Remove Object buttons).
   - Adjust pick-and-place positions in the table or use quick presets.
   - Set initial animation speed via slider (can be changed during simulation).

3. **Start the simulation**:
   - Click **START SIMULATION**.
   - The UR5 arm will sequentially pick and place objects with visual feedback.
   - **Adjust speed in real-time** using the slider while simulation is running.
   - Watch the robot follow smooth paths between sequential objects.

4. **Monitor progress**:
   - View real-time end-effector position in the overlay.
   - Track object trails as they move through the workspace.
   - Read status messages showing current operation.

5. **Reset simulation**:
   - Click **RESET SIMULATION** to clear and restart.

---

## 🏗️ Key Features

- ✅ **Multi-object pick-and-place** with configurable coordinates for unlimited objects.
- ✅ **Smooth and realistic trajectory planning** with inverse kinematics and trapezoidal velocity profiles.
- ✅ **Real-time speed control** - dynamically adjust animation speed during execution without stopping.
- ✅ **Intelligent path transitions** - robot automatically follows smooth paths from previous place position to next pick position.
- ✅ **Interactive GUI** with live updates on speed, position, and status.
- ✅ **Visual 3D workspace** with object cubes, path trails, and position annotations.
- ✅ **Quick preset configurations** for easy testing (line, circle, grid, random patterns).
- ✅ **Dynamic object management** - add or remove objects on the fly.
- ✅ **Pause and reset controls** for experimentation and debugging.
- ✅ **Real-time end-effector tracking** with coordinate display.
- ✅ **Colored path trails** showing movement history for each object.
- ✅ **Full 3D interaction** with preserved aspect ratios (rotate, zoom, pan).

---

## 🧩 Concepts Demonstrated

- **Inverse Kinematics (IK)**: Computing joint angles for desired end-effector positions using iterative solvers.
- **Trajectory Generation**: Smooth motion between waypoints using trapezoidal velocity profiles (`trapveltraj`).
- **Path Planning**: Intelligent waypoint sequencing for efficient multi-object manipulation with smooth transitions.
- **Robot Simulation**: Rigid body tree model of UR5 with full 3D visualization and accurate kinematics.
- **GUI Programming**: User interface with uicontrols, tables, sliders, and panels with event handling.
- **Dynamic Object Handling**: Real-time attachment/detachment of objects to end-effector with precise timing.
- **Visualization Enhancements**: Cube representation, path trails, position annotations, and real-time updates.
- **Real-time Control**: Dynamic parameter adjustment during execution (speed control).
- **State Management**: Proper handling of simulation states (running, stopped, reset).
- **Frame-based Updates**: Throttled display updates for optimal performance.

---

## 🛠️ Requirements

- MATLAB R2022a or later (with Robotics System Toolbox)
- Works entirely within MATLAB, including **MATLAB Online**
- No external libraries or dependencies required

---

## 📂 Project Structure

```
ur5-simulation/
├── project.m    # Main simulation function
├── sample/
│   └── demo.gif           # Demonstration GIF
└── README.md              # This file
```

---

## 🎯 Use Cases

- **Educational Tool**: Learning inverse kinematics, trajectory planning, and robot control.
- **Algorithm Testing**: Experimenting with different path planning strategies.
- **Demonstration**: Showcasing robotic manipulation concepts.
- **Prototyping**: Testing pick-and-place sequences before real-world implementation.
- **Research**: Exploring multi-object manipulation strategies.

---

## 🔧 Customization

### Adding Custom Presets
Modify the `loadPreset` function to add new patterns:
```matlab
case 'custom'
    % Your custom positioning logic
    pickPos = [...];
    placePos = [...];
```

### Adjusting Robot Parameters
- **Hover height**: Modify `hoverHeight` variable (default: 0.15m)
- **Trajectory steps**: Adjust `numSteps` calculation for smoother/faster motion
- **Workspace size**: Change `cubeSize` in simulation window setup

### Modifying Object Appearance
- **Cube size**: Change `cubeSize` in object creation (default: 0.05m)
- **Colors**: Modify `pathColors = lines(numObjects)` for different color schemes
- **Trail width**: Adjust `'LineWidth'` parameter in trail plotting

---

## 🐛 Troubleshooting

**Issue**: Robot moves jerkily
- **Solution**: Increase `numSteps` or decrease animation speed

**Issue**: Objects don't attach properly
- **Solution**: Check `pickIdx` and `placeIdx` calculations match trajectory segments

**Issue**: Simulation window distorted
- **Solution**: Code includes fixed aspect ratio - ensure MATLAB graphics drivers are updated

**Issue**: Slow performance
- **Solution**: Reduce number of objects or increase frame throttling interval

---

## 📝 Technical Notes

- **Coordinate System**: Right-handed coordinate system with Z-axis pointing upward
- **Units**: All distances in meters, angles in radians
- **IK Solver**: Weights prioritize position over orientation [0.25 0.25 0.25 1 1 1]
- **Trajectory**: Trapezoidal velocity profile ensures smooth acceleration/deceleration
- **Frame Rate**: Display updates throttled every 5 frames for optimal performance

---

## 🎓 Educational Value

This project demonstrates key concepts in:
- Robotics kinematics and dynamics
- Inverse kinematics solvers
- Trajectory planning algorithms
- GUI development in MATLAB
- 3D visualization techniques
- Real-time control systems
- State machine implementation

---

## 🙏 Acknowledgments

This micro-project was developed as part of the **MDM course: Fundamentals of Robotics**, instructed by **Prof. Salaba Jacob** at Shah & Anchor Kutchhi Engineering College (SAKEC).

---

## 👨‍💻 Author

> **Anvay Mayekar**
> 🎓 B.Tech in Electronics & Computer Science — SAKEC
>
>[![GitHub](https://img.shields.io/badge/GitHub-181717.svg?style=for-the-badge\&logo=GitHub\&logoColor=white)](https://github.com/anvaymayekar)
[![LinkedIn](https://img.shields.io/badge/LinkedIn-0A66C2.svg?style=for-the-badge\&logo=LinkedIn\&logoColor=white)](https://linkedin.com/in/anvaymayekar)
[![Instagram](https://img.shields.io/badge/Instagram-%23E4405F.svg?style=for-the-badge\&logo=Instagram\&logoColor=white)](https://instagram.com/anvaymayekar)



