## 2025RobotCode
 FRC Team 5414's code for the 2025 competition robot for Reefscape 🪸, Pearracuda 🐠. The code is written in Java and uses WPILib's Java command-based structure.

 [Simulation Instructions](ascope_assets/README.md)

 ![Pearracuda](/pearracuda.png)
 
## Code Highlights
- Field-Centric Swerve Drive
  
  The robot's drivetrain is a [standard swerve drivetrain with field-centric control](https://github.com/Pearadox/2025RobotCode/blob/main/src/main/java/frc/robot/RobotContainer.java) using SDS MK4i modules with double Kraken X60 motors. The drivetrain uses encoders, a Pigeon 2 gyro, and odometry to control movement during the autonomous and teleoperated phases. The rotation of the drivetrain can be controlled either through [speed](https://github.com/Pearadox/2025RobotCode/blob/22e8898f33335ec9af541d74a5766eed62fa08fb/src/main/java/frc/robot/RobotContainer.java#L417) or [heading](https://github.com/Pearadox/2024BetaRobot/blob/main/src/main/java/frc/robot/subsystems/Drivetrain.java#L215).

- Elevator + Arm, End Effector/Intake Rollers, Climber Winch

  The robot uses WPILib subsystems and enums to effectively create a state machine that controls each mechanism. The robot features an arm mounted on an elevator, enabling two degree of freedom movement to reach all levels of the reef. The arm and elevator use [Motion Magic](https://v6.docs.ctr-electronics.com/en/latest/docs/api-reference/device-specific/talonfx/motion-magic.html) to ensure precise, repeatable control, with both fixed setpoints and [dynamic positions](https://github.com/Pearadox/2025RobotCode/blob/22e8898f33335ec9af541d74a5766eed62fa08fb/src/main/java/frc/robot/commands/AutoAlign.java#L325C1-L371C1) determined through [inverse kinematics](https://www.desmos.com/calculator/jvobhlmrbn). The end effector uses voltage control to intake coral from the coral stations and algae from the reef or lollipops, and [position control](https://github.com/Pearadox/2025RobotCode/blob/4b8c491f6c4f645ceead8bfcd7b8b43b8ccf90e0/src/main/java/frc/robot/subsystems/EndEffector.java#L231) to hold onto a game piece. The climber implements a [state machine](https://github.com/Pearadox/2025RobotCode/blob/22e8898f33335ec9af541d74a5766eed62fa08fb/src/main/java/frc/robot/subsystems/Climber.java#L104) with positional PID control to pull the deep cage and lift the robot.

- Autonomous Path Following

  The robot uses [Team 3015's PathPlanner](https://github.com/mjansen4857/pathplanner), a motion profile generator for FRC robots, to generate and follow autonomous trajectories. Autonomous routines are created using PathPlanner's built-in AutoBuilder and declaring [NamedCommands](https://github.com/Pearadox/2025RobotCode/blob/22e8898f33335ec9af541d74a5766eed62fa08fb/src/main/java/frc/robot/RobotContainer.java#L273) with the PathPlanner application, and selected through [sendable choosers](https://github.com/Pearadox/2025RobotCode/blob/22e8898f33335ec9af541d74a5766eed62fa08fb/src/main/java/frc/robot/RobotContainer.java#L112) in [Elastic](https://frc-elastic.gitbook.io/docs).

- Reef and Coral Station Alignment/Limelight Vision
  
  The robot uses [Limelight's Vision Software](https://limelightvision.io/) to manage two mounted Limelight 4 cameras that provide real-time positional data (Pose) of the robot based on field elements with [Team 3636 General's](https://github.com/frc3636) [vision backend code](https://github.com/Pearadox/2024BetaRobot/blob/main/src/main/java/frc/lib/drivers/vision/LimelightBackend.java) modified to support multiple cameras and [MegaTag2](https://github.com/Pearadox/2024BetaRobot/blob/main/src/main/java/frc/lib/drivers/vision/LimelightBackend.java#L32). A [single button press](https://github.com/Pearadox/2025RobotCode/blob/22e8898f33335ec9af541d74a5766eed62fa08fb/src/main/java/frc/robot/RobotContainer.java#L169) autonomously drive the robot to a pose a given offset from an April Tag using PID controllers for translational and rotational velocity, enabling swift [alignment](https://github.com/Pearadox/2025RobotCode/blob/main/src/main/java/frc/robot/commands/AutoAlign.java) to the reef and coral stations.

- Logging, Simulation, and Replay

  The robot uses [AdvantageKit](https://docs.advantagekit.org/) made by [6328 Mechanical Advantage](https://github.com/Mechanical-Advantage) to log useful information about the robot for debugging purposes. This data can be analayzed in [AdvantageScope](https://docs.advantagescope.org/) and through [Pearascope](https://github.com/Pearadox/PearascopeV2), a custom spreadsheet generator that outputs key entries of interest to to tell a story of what happened throughout a match. [WPILib's simulation classes](https://docs.wpilib.org/en/stable/docs/software/wpilib-tools/robot-simulation/introduction.html) output data that can be used to verify code logic without a physical robot, and Team 5516's [Maple Sim](https://github.com/Shenzhen-Robotics-Alliance/maple-sim) library models interactions between robots, field elements, and game pieces using a physics engine.  
