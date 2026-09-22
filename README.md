# Robotics_Project1
# Underwater Cable Fault Detection Robot

An underwater robotic simulation designed for subsea cable inspection and fault detection using Gazebo-based physics simulation and robotics control.

The project creates an underwater environment with environmental effects such as buoyancy and water currents, while simulating a robotic system capable of inspecting underwater cables and recording inspection data.

## Project Overview

Underwater communication and power cables are critical infrastructure, but inspecting them manually can be expensive, time-consuming, and difficult in challenging underwater conditions.

This project explores an autonomous robotic approach for underwater cable inspection using simulation.

The system provides:

- Underwater environment simulation
- Underwater cable inspection robot
- Buoyancy simulation
- Water-current simulation
- Simulated underwater ecosystem
- AI-based fish and environment plugin
- Cable inspection and fault-detection workflow
- Inspection data logging
- Runtime inspection logs

## System Architecture

                    UNDERWATER ENVIRONMENT
                            |
             +--------------+--------------+
             |              |              |
          Buoyancy       Currents      Ecosystem
             |              |              |
             +--------------+--------------+
                            |
                            v
                  +-------------------+
                  | Underwater Robot  |
                  +---------+---------+
                            |
                    Robot Controller
                            |
                            v
                  +-------------------+
                  | Cable Inspection  |
                  +---------+---------+
                            |
                            v
                     Fault Detection
                            |
                            v
                  +-------------------+
                  | Inspection Logs   |
                  +-------------------+

## Technologies Used

- Gazebo
- ROS
- C++
- Python
- SDF
- CMake
- CSV

## Project Structure

    Robotics_Project1/
    |
    +-- cable_robot_controller/
    |   +-- Robot control components
    |
    +-- fish_ai_plugin/
    |   +-- AI and environment plugin
    |
    +-- CurrentSystem.cc
    |   +-- Water-current simulation system
    |
    +-- buoyancy_test.sdf
    |   +-- Buoyancy testing environment
    |
    +-- ocean_ecosystem_full.sdf
    |   +-- Complete underwater ecosystem
    |
    +-- test.sdf
    |   +-- Simulation testing environment
    |
    +-- inspection_log.csv
    |   +-- Recorded inspection data
    |
    +-- inspection_runtime_log.csv
    |   +-- Runtime inspection records
    |
    +-- CMakeLists.txt
    |   +-- Build configuration
    |
    +-- README.md

## Underwater Environment

The simulation models an underwater environment where the robot operates under environmental forces.

### Buoyancy

The simulation includes buoyancy effects to represent the upward force acting on underwater objects.

This allows the robot's underwater movement and stability to be studied under realistic physical conditions.

### Water Currents

A custom current-system component is included to simulate underwater water flow.

The current can affect the robot's:

- Position
- Velocity
- Stability
- Navigation
- Cable inspection trajectory

## Robot Controller

The cable_robot_controller component is responsible for controlling the underwater inspection robot.

The controller provides the foundation for:

- Robot movement
- Inspection motion
- Interaction with the simulated underwater environment
- Cable-following and inspection behavior

## Cable Inspection

The primary objective of the project is to simulate underwater cable inspection.

The robot operates near the subsea cable and records information during the inspection process.

Inspection information can be stored in CSV files for later analysis.

## Underwater Ecosystem

The project contains an underwater ecosystem simulation through:

    ocean_ecosystem_full.sdf

and the:

    fish_ai_plugin

component.

This allows the simulation environment to contain additional underwater entities and provides a more realistic environment for testing autonomous underwater robots.

## Data Logging

The project includes inspection logs:

    inspection_log.csv
    inspection_runtime_log.csv

These files can be used to analyze robot inspection behavior and runtime performance.

Potential information that can be analyzed includes:

- Robot position
- Inspection status
- Runtime information
- Cable inspection events
- Environmental conditions

## Build

Clone the repository:

    git clone https://github.com/saikousikganduri01/Robotics_Project1.git

    cd Robotics_Project1

Create a build directory:

    mkdir build
    cd build

Configure the project:

    cmake ..

Build the project:

    make

## Running the Simulation

After building the project, launch the required Gazebo environment using the appropriate SDF world file.

For example:

    gazebo ../ocean_ecosystem_full.sdf

For buoyancy testing:

    gazebo ../buoyancy_test.sdf

The exact launch procedure may depend on the installed Gazebo and ROS version and the local environment configuration.

## Objectives

The major objectives of the project are:

1. Simulate an underwater robotic environment.
2. Model realistic underwater physical effects.
3. Simulate buoyancy and water currents.
4. Develop a robotic controller for underwater cable inspection.
5. Explore autonomous subsea inspection.
6. Record inspection data for analysis.
7. Provide a simulation environment for future fault-detection algorithms.

## Future Improvements

The project can be extended with:

- Underwater camera-based cable inspection
- Deep-learning-based cable fault detection
- Autonomous cable tracking
- Underwater localization and mapping
- Real-time fault classification
- Sonar-based inspection
- Autonomous navigation and obstacle avoidance
- Real-time inspection dashboard
- Multi-robot underwater inspection
- Battery-aware mission planning

## Applications

Potential applications include:

- Subsea power-cable inspection
- Underwater communication-cable monitoring
- Offshore infrastructure inspection
- Marine robotics research
- Autonomous underwater vehicle development
- Underwater fault detection
- Simulation-based robotics testing

## Author

Sai Kousik Ganduri

B.Tech Computer Science and Engineering
Artificial Intelligence and Machine Learning
Amrita Vishwa Vidyapeetham

## License

This project is intended for educational and research purposes.
