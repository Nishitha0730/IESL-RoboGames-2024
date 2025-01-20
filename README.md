# IESL-RoboGames-2024

This repository contains the submission for the **IESL RoboGames 2024 - University Category First Round**. The task focuses on designing and programming a Webots simulation for an E-puck robot to navigate a maze and interact with colored walls in a predefined order.

---

## Project Description

The objective of the first round was to:

1. Build a 2.5x2.5 meter arena in Webots according to the competition's specifications.
2. Program the E-puck robot to navigate the arena and visit colored walls in the order:
   **Red → Yellow → Pink → Brown → Green**.
3. Stop the robot upon completion of the task.

The arena and robot behavior were designed to adhere strictly to the competition guidelines.

---

## Arena Details

- **Dimensions**: 2.5m x 2.5m
- **Wall Gap**: 0.25m
- **Wall Dimensions**:
  - Height: 0.1m
  - Breadth: 0.01m
  - Length: Multiples of 0.25m
- **Colored Walls**:
  - **Red (#FF0000)**
  - **Yellow (#FFFF00)**
  - **Pink (#FF00FF)**
  - **Brown (#A5691E)**
  - **Green (#00FF00)**

---

## How It Works

1. The robot starts at any position in the maze.
2. It detects and navigates to the colored walls in the specified order.
3. The robot stops upon completing the sequence.

---

## Demonstration

### Image of the Simulation

![Webots Simulation](https://github.com/Nishitha0730/IESL-RoboGames-2024/blob/main/First%20Round/ElectroBots.png)

### Video of Full Simulation

[Click here to watch the simulation](https://1drv.ms/v/c/66efd464b4ddcd14/EQJkU4Rn0iZAtjbNSy9vMhcBhon2ZhFBdb3_L9B-IZXYNQ?e=gV7gZM)

---

## Code Overview

### File Structure

```
IESL-RoboGames-2024/
├── controllers/
│   └── epuck_controller/
│       └── epuck_controller.py
├── worlds/
│   └── RoboGames_2024_University_Category.wbt
└── README.md
```

- **controllers/epuck_controller/epuck_controller.py**: Contains the Python code for the robot's behavior.
- **worlds/RoboGames_2024_University_Category.wbt**: The Webots world file for the simulation.

### Example Controller Code

```python
from controller import Robot

# Constants
TIME_STEP = 64
COLORS = ["Red", "Yellow", "Pink", "Brown", "Green"]

# Initialize robot
def main():
    robot = Robot()
    wheels = [robot.getDevice("wheel1"), robot.getDevice("wheel2")]

    for wheel in wheels:
        wheel.setPosition(float('inf'))
        wheel.setVelocity(0.0)

    # Add robot control logic here
    print("Starting robot navigation...")

    # Simulate navigation (placeholder for actual logic)
    for color in COLORS:
        print(f"Navigating to {color} wall...")
        robot.step(TIME_STEP * 10)  # Placeholder for delay

    print("Task completed!")

if __name__ == "__main__":
    main()
```

---

## Requirements

1. [Webots](https://cyberbotics.com/) installed on your system.
2. Clone this repository:
   ```bash
   git clone https://github.com/Nishitha0730/IESL-RoboGames-2024.git
   ```
3. Open the `.wbt` file in Webots and run the simulation.

---

## License

This project is licensed under the MIT License. See the [LICENSE](LICENSE) file for details.

---

## Acknowledgments

- **IESL RoboGames 2024** organizers for providing the platform.
- University mentors and peers for their support.

For any queries, feel free to contact me via [GitHub Issues](https://github.com/Nishitha0730/IESL-RoboGames-2024/issues).
