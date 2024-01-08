# 2023 Swerve Integrated Advanced Project (IAP)

This code is updated to WPILib's 2024 release.

### CAN ID Protocol
| Module Name  | Drive Motor | Turn Motor | CANCoder |
| ------------- | ------------- | ------------- | ------------- |
| Front Left  | CAN #1  | CAN #2  | CAN #9  |
| Front Right  | CAN #3  | CAN #4  | CAN #10  |
| Back Left  | CAN #5  | CAN #6  | CAN #11  |
| Back Right  | CAN #7  | CAN #8  | CAN #12  |

## Testing updates
### What has been done physically (non-exhaustive):
- Set all proper CAN IDs
- Verified all module motors function
- Verified all CANCoders function
- Verified correct motor inverts
- Verified PIDF for drive motors (12/16)
### What has been done in simulation (non-exhaustive):
- Verified that all drive and turn motors function
- Determined proper characterizations for all drive and turn motors
- Verified that kinematics of swerve drive is correct with high-level controls
- High-level control methods (Crab Drive, Swerve Teleop, Test Command that switches between modules) are functional
- High-level autonomous (PathPlannerLib) is functional
### TODO List (non-exhaustive):
- Test PID for turn motors
- Verify that turn motors are recording correct module angle
- Verify that odometry is accurate
- Verify that turn motors are responsive enough with regular kP control
- Dry-run a "Crab Drive" mode to test basic driving

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
  - Module testing command with both PIDF and voltage testing, that can switch between modules

### "250 lines or less" 
"250 lines or less" design philosophy - SwerveDrive subsystem is small compared to most other codebases. 
Contains a Utility class in order to support more technical/Calculus features and make the code more compact.
- Helps support students who are just learning AP Calculus AB or do not know it
- Example of Calc usage: integrating angular velocity (calculated from module states, not odometry) in order to simulate gyro
- Example of Calc usage: kinematics fix (discretize) for "curving" motion while chassis translates + rotates
- Example of Calc usage: rate-limiting control differential (dC/dt) by certain rate for smoother control
