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

## Maze Design

The maze used in this project was designed by **[Friend's GitHub Name](https://github.com/FriendsGitHubProfile)**.

### Maze Image

![Maze Design](https://your-image-link.com)

### Maze World File

Download the Webots world file of the maze here: [Maze World File](https://your-maze-world-link.com)

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

### Highlights of Controller Code

1. **Color Detection**: The robot uses the camera to detect the average color and identify the wall's color.
2. **Flood Fill Algorithm**: A flood fill approach is implemented to calculate the shortest path from the robot's current position to the goal.
3. **Proximity Sensors**: Sensors are utilized to map walls and detect obstacles.
4. **Goal Check**: The robot ensures it has reached the correct wall based on color before proceeding to the next target.

---

## Requirements

1. Install [Webots](https://cyberbotics.com/).
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
- **[Friend's GitHub Name](https://github.com/FriendsGitHubProfile)** for designing the maze.
- University mentors and peers for their support.

For any queries, feel free to contact me via [GitHub Issues](https://github.com/Nishitha0730/IESL-RoboGames-2024/issues).
