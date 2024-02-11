package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.*;
import frc.robot.subsystems.PhotonVision.PhotonVision;
import frc.robot.subsystems.swerve.SwerveDrive;
import frc.robot.subsystems.swerve.SwerveModuleIO;
import frc.robot.subsystems.swerve.SwerveModuleIOSim;
import frc.robot.subsystems.swerve.SwerveModuleIOSparkMax;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;

public class RobotContainer {

  /*
   * TO THE FUTURE READERS/REVIEWERS OF THIS FILE:
   * Feel free to use any parts of this project or its entirety in a Competition
   * FRC robot of any kind or team.
   * This codebase is 95% Competition-ready (minus some minor cosmetic things). It
   * is designed so that the modules are modular (meaning easy to switch).
   * This technique enables us to simulate the swerve drivebase and develop at
   * home to our heart's content.
   * In the future, we can also write a SwerveModuleIOTalonFX.java as well and
   * easily "plug" it in.
   * 
   * Minor warning: advanced Java syntax that this project uses:
   * - Java Lambdas
   * - Java Suppliers and Consumers
   * - Java Interface Classes
   * - Java For-Each Loops
   */

  // ---------------------- START OF CONFIG SECTION --------------------------

  // WARNING: TRAJECTORY DRIVING NOT TESTED IN REAL LIFE (IRL)
  // DO NOT USE UNTIL DRIVING IN SAFE SPACE
  // THIS IS A SECOND WARNING!!! THIS IS VERY DANGEROUS.
  // To do trajectory driving or not
  // TREAT THIS LIKE A RED BUTTON
  private final boolean autoOrNot = true;

  // Whether to set alliance for teleop driving or not
  private final boolean setAlliance = true;
  
  // Set to blue alliance
  // Only enabled if the setAlliance boolean is enabled
  // TODO - Set automatically via game data
  private final boolean blueAllianceOrNot = true;

  // Checks if using xBox or keyboard
  // False : keyboard
  // True : Xbox
  public static final boolean isXbox = true;

  // If we need to data log or not
  // Works in simulation
  // False : not data log
  // True : will data log
  public final boolean isDataLog = true;

  // Defines starting pose of robot
  // TODO - Please remove this in future if developing for AprilTags
  public final Pose2d startpose = new Pose2d(new Translation2d(0, 0), new Rotation2d());

  
  // ---------------------- END OF CONFIG SECTION --------------------------

  // Checks if robot is real or not
  private static boolean isSim = Robot.isSimulation();

  // Xbox + an additional one for PC use
  private final Joystick actualXbox = new Joystick(0);
  private final Joystick additionalJoy = new Joystick(1);
  // Chooser for testing teleop commands
  private final SendableChooser<Command> teleopCommandChooser = new SendableChooser<>();

  // Define axises for using joystick
  private final int translationAxis = 1;
  private final int strafeAxis = 0;
  private final int rotationAxis = 4; // For xBox

  // Creates array of swerve modules for use in SwerveDrive object - null in
  // context of code
  SwerveModuleIO[] swerveMods = new SwerveModuleIO[4];
  // Empty SwerveDrive object
  private SwerveDrive swerve;
  // Empty testing commands (not used if not needed)
  private TestFourModules allFour;
  // Empty SwerveTeleop object
  private SwerveTeleop teleop;
  // Empty CrabDrive object
  private CrabDrive crabDrive;

  // Auto Trajectories
  private final SwerveAuto driveForward;

  public RobotContainer() {

    if (isDataLog) {
      // Data logging works on both real + simulated robot with all DriverStation
      // outputs!
      DataLogManager.start();
      DriverStation.startDataLog(DataLogManager.getLog(), false);
      SmartDashboard.putString("Data Log Folder: ", DataLogManager.getLogDir());
    }

    // Initialize SwerveDrive object with modules
    if (isSim) {
      // Construct swerve modules with simulated motors
      for (int i = 0; i < swerveMods.length; i++) {
        swerveMods[i] = new SwerveModuleIOSim(i);
      }

    } else {
      // Construct swerve modules with real motors
      for (int i = 0; i < swerveMods.length; i++) {
        swerveMods[i] = new SwerveModuleIOSparkMax(i, Constants.SwerveConstants.moduleCANIDs[i][0],
            Constants.SwerveConstants.moduleCANIDs[i][1], Constants.SwerveConstants.moduleCANIDs[i][2],
            Constants.SwerveConstants.moduleAngleOffsets[i], Constants.SwerveConstants.moduleInverts[i]);
      }

    }

    this.swerve = new SwerveDrive(startpose, this.swerveMods[0], this.swerveMods[1], this.swerveMods[2], this.swerveMods[3]);

    if (isXbox) {
      // Supply teleop command with joystick methods - USES LAMBDAS
      teleop = new SwerveTeleop(this.swerve, () -> {
        return -this.actualXbox.getRawAxis(translationAxis);
      }, () -> {
        return -this.actualXbox.getRawAxis(strafeAxis);
      }, () -> {
        return -this.actualXbox.getRawAxis(rotationAxis);
      }, () -> {
        return this.actualXbox.getRawAxis(XboxController.Axis.kRightTrigger.value);
      }, () -> {
        return true;
      }, setAlliance, blueAllianceOrNot);

    } else if (!isXbox) {
      // Supply teleop command with joystick methods - USES LAMBDAS
      teleop = new SwerveTeleop(this.swerve, () -> {
        return -this.actualXbox.getX();
      }, () -> {
        return -this.actualXbox.getY();
      }, () -> {
        return -this.additionalJoy.getRawAxis(0);
      }, () -> {
        return 1.0;
      }, () -> {
        return true;
      }, setAlliance, blueAllianceOrNot);

    }

    crabDrive = new CrabDrive(this.swerve, () -> {
      return -this.actualXbox.getX();
    }, () -> {
      return -this.actualXbox.getY();
    });

    allFour = new TestFourModules(swerve, actualXbox);

    teleopCommandChooser.addOption("Regular Teleop", teleop);
    teleopCommandChooser.addOption("Crab Teleop", crabDrive);
    teleopCommandChooser.addOption("Module Test Command", allFour);
    teleopCommandChooser.setDefaultOption("Regular Teleop", teleop);

    if (autoOrNot) {
      driveForward = new SwerveAuto("DriveForward", this.swerve);
    }

    SmartDashboard.putData(teleopCommandChooser);
    this.configureBindings();
  }

  private void configureBindings() {
  }

  public Command getAutonomousCommand() {
    // if (autoOrNot) {
    //   return auto;
    // } else {
    //   return null;
    // }
    return new TargetAprilTag(new PhotonVision());
  }

  public void initCommandInTeleop() {
    swerve.setDefaultCommand(teleopCommandChooser.getSelected());
  }

  /**
   * Gets Robot.isReal() from RobotContainer (slow when calling every loop)
   * 
   * @return If simulated or not
   */
  public static boolean getSimOrNot() {
    return isSim;
  }

}
