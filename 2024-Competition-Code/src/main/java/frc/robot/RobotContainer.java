package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.*;
import frc.robot.subsystems.PhotonVision.PhotonVision;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.CrabDrive;
import frc.robot.commands.SwerveAuto;
import frc.robot.commands.SwerveTeleop;
import frc.robot.commands.TestFourModules;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.swerve.SwerveDrive;
import frc.robot.subsystems.swerve.SwerveModuleIO;
import frc.robot.subsystems.swerve.SwerveModuleIOSim;
import frc.robot.subsystems.swerve.SwerveModuleIOSparkMax;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import org.photonvision.PhotonCamera;

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

  // Defines starting pose of robot
  // TODO - Please remove this in future if developing for AprilTags
  public final Pose2d startpose = new Pose2d(new Translation2d(0, 0), new Rotation2d());

  // ---------------------- END OF CONFIG SECTION --------------------------

  // Xbox + an additional one for PC use
  private final Joystick drivingXbox = new Joystick(0);
  private final Joystick simulationJoy = new Joystick(1);
  private final static Joystick mechanismJoy = new Joystick(2);

  // Chooser for testing teleop commands
  private final SendableChooser<Command> teleopCommandChooser = new SendableChooser<>();

  // Define axises for using joystick
  private final int translationAxis = XboxController.Axis.kLeftY.value; // Axis ID: 1
  private final int strafeAxis = XboxController.Axis.kLeftX.value; // Axis ID: 0
  private final int rotationAxis = XboxController.Axis.kRightX.value; // Axis ID: 4

  // Creates array of swerve modules for use in SwerveDrive object - null in
  // context of code
  SwerveModuleIO[] swerveMods = new SwerveModuleIO[4];
  // Empty SwerveDrive object
  private SwerveDrive swerve;
  // Empty testing commands (not used if not needed)
  private TestFourModules testAllFourModules;
  // Empty SwerveTeleop object
  private SwerveTeleop teleop;
  // Empty CrabDrive object
  private CrabDrive crabDrive;

  // Empty AprilTag command object
  private TargetAprilTag targetAprilTag;

  // Empty Shooter object
  private Shooter shooter;

  // Auto Trajectories
  private SwerveAuto autoPath;

  // Field centric toggle - true for field centric, false for robot centric
  private boolean fieldCentricToggle = true;

  public RobotContainer() {

    // Construct swerve subsystem with appropriate modules - DO NOT REMOVE THIS
    this.constructSwerve();

    // Create swerve commands - DO NOT REMOVE THIS
    this.createSwerveCommands();

    PhotonCamera camera = new PhotonCamera("Microsoft_LifeCam_HD-3000");
    PhotonVision photonVision = new PhotonVision(camera);
    targetAprilTag = new TargetAprilTag(photonVision, swerve);

    teleopCommandChooser.addOption("AprilTagTargetingTEST", targetAprilTag);
    teleopCommandChooser.addOption("Regular Teleop", teleop);
    teleopCommandChooser.addOption("Crab Teleop", crabDrive);
    teleopCommandChooser.addOption("Module Test Command", testAllFourModules);
    teleopCommandChooser.setDefaultOption("Regular Teleop", teleop);

    SmartDashboard.putData(teleopCommandChooser);

    shooter = new Shooter();

    this.configureBindings();
  }

  private void constructSwerve() {
    if (Constants.currentRobot.dataLogEnabled) {
      // Data logging works on both real + simulated robot with all DriverStation
      // outputs!
      DataLogManager.start();
      DriverStation.startDataLog(DataLogManager.getLog(), false);
      SmartDashboard.putString("Data Log Folder: ", DataLogManager.getLogDir());
    }

    // Initialize SwerveDrive object with modules
    if (Constants.isSim) {
      // Construct swerve modules with simulated motors
      for (int i = 0; i < swerveMods.length; i++) {
        swerveMods[i] = new SwerveModuleIOSim(i);
      }

    } else {
      // Construct swerve modules with real motors
      for (int i = 0; i < swerveMods.length; i++) {
        swerveMods[i] = new SwerveModuleIOSparkMax(i, Constants.currentRobot.moduleCANIDs[i][0],
            Constants.currentRobot.moduleCANIDs[i][1], Constants.currentRobot.moduleCANIDs[i][2],
            Constants.currentRobot.moduleAngleOffsets[i], Constants.SwerveConstants.moduleInverts[i]);
      }

    }

    this.swerve = new SwerveDrive(startpose, this.swerveMods[0], this.swerveMods[1], this.swerveMods[2],
        this.swerveMods[3]);

  }

  private void createSwerveCommands() {
    autoPath = new SwerveAuto("DriveForward (test)", this.swerve);
    

    if (Constants.currentRobot.xboxEnabled) {
      // Supply teleop command with joystick methods - USES LAMBDAS
      teleop = new SwerveTeleop(this.swerve, () -> {
        return -this.drivingXbox.getRawAxis(translationAxis);
      }, () -> {
        return -this.drivingXbox.getRawAxis(strafeAxis);
      }, () -> {
        return -this.drivingXbox.getRawAxis(rotationAxis);
      }, () -> {
        return this.drivingXbox.getRawAxis(XboxController.Axis.kRightTrigger.value);
      }, () -> {

        // Toggles between field centric (true) and robot centric (false)
        if (this.drivingXbox.getRawButtonPressed(XboxController.Button.kX.value)) {
          fieldCentricToggle = !fieldCentricToggle;
        }

        return fieldCentricToggle;
      }, Constants.currentRobot.allianceEnabled);

    } else if (!Constants.currentRobot.xboxEnabled) {
      // Supply teleop command with joystick methods - USES LAMBDAS
      teleop = new SwerveTeleop(this.swerve, () -> {
        return -this.drivingXbox.getX();
      }, () -> {
        return -this.drivingXbox.getY();
      }, () -> {
        return -this.simulationJoy.getRawAxis(0);
      }, () -> {
        return 0.0;
      }, () -> {

        // Toggles between field centric (true) and robot centric (false)
        if (this.drivingXbox.getRawButtonPressed(1)) {
          fieldCentricToggle = !fieldCentricToggle;
        }

        return fieldCentricToggle;
      }, Constants.currentRobot.allianceEnabled);

    }

    crabDrive = new CrabDrive(this.swerve, () -> {
      return -this.drivingXbox.getX();
    }, () -> {
      return -this.drivingXbox.getY();
    });

    testAllFourModules = new TestFourModules(swerve, drivingXbox);

  }

  private void configureBindings() {

    JoystickButton triggerIntake = new JoystickButton(mechanismJoy, Constants.ButtonMap.intakeNote);
    triggerIntake.onTrue(new IntakeCommand(0.6, shooter));

    JoystickButton triggerManualIntake = new JoystickButton(mechanismJoy, 1);
    triggerManualIntake.whileTrue(new IntakeManual(0.8, shooter));

    JoystickButton triggerShooterButton = new JoystickButton(mechanismJoy, 8);
    triggerShooterButton.whileTrue(new Shoot(2500, shooter));
  }

  public Command getAutonomousCommand() {
    return autoPath;

  }

  public static Joystick getIntakeJoy() {
    return mechanismJoy;
  }

  public void initCommandInTeleop() {
    swerve.setDefaultCommand(teleopCommandChooser.getSelected());
  }

}
