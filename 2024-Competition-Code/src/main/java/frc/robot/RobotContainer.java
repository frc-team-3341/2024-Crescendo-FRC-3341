package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;

import org.photonvision.PhotonCamera;

import frc.robot.commands.*;
import frc.robot.commands.climber.BasicClimbTeleop;
import frc.robot.commands.notemechanism.*;
import frc.robot.commands.swerve.*;
import frc.robot.commands.swerve.BackingUpIntoAmp.MoveBackIntoAmp;
import frc.robot.commands.swerve.BackingUpIntoAmp.BackupSimple;
import frc.robot.commands.swerve.ClosestNinetyDegrees;
import frc.robot.subsystems.*;
import frc.robot.subsystems.swerve.*;
import frc.robot.subsystems.photonvision.*;

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
  private TestFourModules allFour;
  // Empty SwerveTeleop object
  private SwerveTeleop teleop;
  // Empty CrabDrive object
  private CrabDrive crabDrive;
  // Empty MoveBackIntoAmp object
  private MoveBackIntoAmp moveBackIntoAmp;

  // Empty AprilTag command object
  private TargetAprilTag targetAprilTag;

  // Empty BackupSimple command object
  private BackupSimple backupSimple;

  private ClosestNinetyDegrees ninetyDegreeRotation;

  // Empty Shooter object
  private Shooter shooter;

  // Field centric toggle - true for field centric, false for robot centric
  private boolean fieldCentricToggle = true;

  // Empty Climber object
  private Climber climber;

  // Empty InitializeAutoPaths object
  private InitializeAutoPaths autoPaths;

  public RobotContainer() {

    // Construct swerve subsystem with appropriate modules - DO NOT REMOVE THIS
    this.constructSwerve();

    // Create swerve commands - DO NOT REMOVE THIS
    this.createSwerveCommands();

    if (Constants.currentRobot.enableClimber) {
      this.configureClimber();
    }

    if (Constants.currentRobot.enableShooter) {
      this.configureShooter();
    }

    if (Constants.currentRobot.enablePhotonVision) {
      this.configurePhotonVision();
    }

    this.configureAuto();
    
    // Construct all other things
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

    allFour = new TestFourModules(swerve, drivingXbox);

    moveBackIntoAmp = new MoveBackIntoAmp(swerve);
    JoystickButton moveButton = new JoystickButton(drivingXbox, XboxController.Button.kY.value);

    backupSimple = new BackupSimple(swerve, swerveMods[0]);
    JoystickButton backupSimpleButton = new JoystickButton(drivingXbox, XboxController.Button.kA.value);

    ninetyDegreeRotation = new ClosestNinetyDegrees(swerve);
    JoystickButton ninetyDegreeRotationButton = new JoystickButton(drivingXbox, XboxController.Button.kB.value);

    ninetyDegreeRotationButton.onTrue(ninetyDegreeRotation);
    backupSimpleButton.toggleOnTrue(backupSimple);
    moveButton.toggleOnTrue(moveBackIntoAmp);
    
    teleopCommandChooser.addOption("Regular Teleop", teleop);
    teleopCommandChooser.addOption("Crab Teleop", crabDrive);
    teleopCommandChooser.addOption("Module Test Command", allFour);
    teleopCommandChooser.setDefaultOption("Regular Teleop", teleop);

    SmartDashboard.putData(teleopCommandChooser);
  }

  private void configureShooter() {
    shooter = new Shooter();
    // Triggers intake rollers and stops at beambreaks at the middle of the note mechanism
    JoystickButton triggerIntake = new JoystickButton(mechanismJoy, 16); 
    triggerIntake.onTrue(new IntakeBeamBreak(0.6, shooter));
    JoystickButton triggerIntakeSource = new JoystickButton(mechanismJoy, 3); 
    triggerIntakeSource.onTrue(new IntakeSource(-1500, -1500,  0.8, shooter));
    // Stops rollers
    JoystickButton stopIntake = new JoystickButton(mechanismJoy, 4);
    stopIntake.onTrue(new StopIntake(shooter));

    /*JoystickButton manualIntake = new JoystickButton(mechanismJoy, 16);
    manualIntake.whileTrue(new IntakeManual(0.6, shooter));
    /* */
    // Manually activates intake rollers when you go up on the POV 
    POVButton triggerIntakeManual = new POVButton(mechanismJoy, 0); 
    triggerIntakeManual.whileTrue(new IntakeManual(0.8, shooter));

  }

  public void configureClimber() {
    climber = new Climber(); // Climber CAN ID was inactive, causing a timeout
    JoystickButton climberControl = new JoystickButton(mechanismJoy, mechanismJoy.getYChannel());
    climberControl.whileTrue(new BasicClimbTeleop(climber, mechanismJoy));
    //Throttle switching the power hasn't been updated yet. Should test code before implementing
  }

  public void configurePhotonVision() {
    PhotonCamera camera = new PhotonCamera("Microsoft_LifeCam_HD-3000");
    photonvision photonVision = new photonvision(camera);
    targetAprilTag = new TargetAprilTag(photonVision, swerve);

//    JoystickButton alignButton = new JoystickButton(drivingXbox, XboxController.Button.kA.value);
//    alignButton.onTrue(targetAprilTag);
  }

  public void configureAuto() {
    autoPaths = new InitializeAutoPaths(swerve, shooter);
  }

  private void configureBindings() {
  }

  public Command getAutonomousCommand() {
    return autoPaths.getAutonomousCommand();
  }

  public static Joystick getIntakeJoy() {
    return mechanismJoy;
  }

  public void initCommandInTeleop() {
    swerve.setDefaultCommand(teleopCommandChooser.getSelected());
  }

}
