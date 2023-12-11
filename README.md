# 2023 Swerve Integrated Advanced Project (IAP)

WARNING: SOME AUTONOMOUS LIBRARY METHODS (PathPlannerLib) ARE DEPRECATED IN 2024!!! USE AT YOUR OWN RISK. PathPlannerLIB WILL SWITCH TO NEW FRAMEWORK OF METHODS!!!
Autonomous is disabled until ready to test (Teleop hasn't been tested).

## Non-exhaustive list of features
- Supports 8-motor NEO swerve
- Modularity with Java Interface class (inspired by FRC Team 3181) - in the future, we can create an Interface for TalonFX
- Uses Java Suppliers and Consumers as well as Java Lambdas in order to supply them
- Uses Java Object References for Autonomous code
- Implements Trajectory driving with PathPlannerLib and PathPlanner's app by reading JSON file
- Asymmetric Rate limiting, inspired by FRC Team NOMAD 6995's code
- Integrated encoder support for driving motor only (due to low CPR of REV NEO turning encoder)
- Ability to visualize Swerve Drive modules + robot in WPILib Glass
- Ability to visualize Swerve Drive module states in AdvantageScope 2023 **(Deprecated in 2024)**
- Ability to switch from keyboard control to XBox with one boolean **(not during matches, have to re-deploy)**
- **Supports simulation of Swerve teleop, autonomous, and module testing code**
  - **Only supports swerve simulation for now, not mechanisms!**
- Convenient telemetry using centralized telemetry method in module class + Data Logging of SmartDashboard
- Ability to switch between testing commands using SendableChooser:
  - Regular Swerve Teleop command
  - "Crab Drive" command for testing without gyro
  - Module testing command with power/voltage only
  - Module testing command with PIDF control

### "250 lines or less" 
"250 lines or less" design philosophy - SwerveDrive subsystem is small compared to most other codebases. 
Contains a Utility class in order to support more technical/Calculus features and make the code more compact.
- Helps support students who are just learning AP Calculus AB or do not know it
- Example of Calc usage: integrating angular velocity (calculated from module states, not odometry) in order to simulate gyro
- Example of Calc usage: kinematics fix (discretize) for "curving" motion while chassis translates + rotates

### CAN ID Protocol
- Front Left: CAN #1 - Drive; CAN #2 - Turn
- Front Right: CAN #3 - Drive; CAN #4 - Turn
- Back Left: CAN #5 - Drive; CAN #6 - Turn
- Back Right: CAN #7 - Drive; CAN #8 - Turn
- CAN #9 - Front Left CANCoder
- CAN #10 - Front Right CANCoder
- CAN #11 - Back Left CANCoder
- CAN #12 - Back Right CANCoder
