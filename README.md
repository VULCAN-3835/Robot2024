# Robot2024
Welcome to Team 3835's Robot2024 project repository, showcasing our robot designed specifically for the 2024 FRC competition, Crescendo.

## Overview
Our robot Sisyphus for the 2024 FRC season, Crescendo, was built in our workshop in Tichon Hadash Tel Aviv in order to compete. This repository contains all the code and configurations required for operating our robot.

## Subsystems
* ChassisSubsystem
* ClimberSubsystem
* IntakeSubsystem
* LEDSubsystem
* ShooterSubsystem

## Autonomous Commands with PathPlanner
### ChassisSubsystem Configuration
In the `chassisSubsystem.java` file, our robot's holonomic drive system is fine-tuned for optimal path following using the `AutoBuilder.configureHolonomic()` method. This configuration ensures smooth traversal of specified paths during autonomous operations.

### Autonomous Command Instantiation
Within the `RobotContainer.java` file, autonomous commands are instantiated based on generated Autonomous paths from the PathPlanner app. These commands are organized into a chooser in the Shuffleboard, facilitating easy selection and modification during testing and competitions.

We use `NamedCommands.registerCommand()` to assign names to basic commands such as `ShootCmd.java` to be later used in the PathPlanner app, The PathPlanner app has 2 main sections. one is for making paths for the robot to follow, there are many options in the path generation tool such as waypoitns to represent when to perform a certain operation during path runtime or rotation waypoints along the way. The second section is for designing full Auto courses which are basicly SequentialCommandGroups that easily integrate commands that follow certain path trajectories that we made in the previous menu and usage of named commands.
https://www.microsoft.com/en-us/p/frc-pathplanner/9nqbkb5dw909?cid=storebadge&ocid=badge&rtc=1&activetab=pivot:overviewtab