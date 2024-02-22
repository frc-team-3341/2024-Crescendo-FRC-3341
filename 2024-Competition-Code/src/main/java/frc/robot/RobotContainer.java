package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.swerve.CrabDrive;
import frc.robot.commands.IntakeBeamBreak;
import frc.robot.commands.IntakeManual;
import frc.robot.commands.Shoot;
import frc.robot.commands.StopIntake;
import frc.robot.commands.swerve.SwerveAuto;
import frc.robot.commands.swerve.SwerveTeleop;
import frc.robot.commands.swerve.TestFourModules;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.swerve.SwerveDrive;
import frc.robot.subsystems.swerve.SwerveModuleIO;
import frc.robot.subsystems.swerve.SwerveModuleIOSim;
import frc.robot.subsystems.swerve.SwerveModuleIOSparkMax;
import frc.robot.subsystems.swerve.SwerveModuleIOCANCoder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;

import frc.robot.commands.auto.BlueAlliance1.B1_StartToAmp;
import frc.robot.commands.auto.BlueAlliance1.B1_NoteToAmp;
import frc.robot.commands.auto.BlueAlliance1.B1_AmpToNote;
import frc.robot.commands.auto.BlueAlliance1.B1_NoteToSpeaker;
import frc.robot.commands.auto.BlueAlliance1.B1_SpeakerToNote;
import frc.robot.commands.auto.BlueAlliance2.B2_StartToAmp;
import frc.robot.commands.auto.BlueAlliance2.B2_NoteToSpeaker;
import frc.robot.commands.auto.BlueAlliance2.B2_SpeakerToNote;
import frc.robot.commands.auto.BlueAlliance2.B2_AmpToNote;
import frc.robot.commands.auto.BlueAlliance2.B2_StartToSpeaker;
import frc.robot.commands.auto.BlueAlliance3.B3_StartToAmp;
import frc.robot.commands.auto.BlueAlliance3.B3_NoteToAmp;
import frc.robot.commands.auto.BlueAlliance3.B3_NoteToSpeaker;
import frc.robot.commands.auto.BlueAlliance3.B3_SpeakerToNote;
import frc.robot.commands.auto.BlueAlliance3.B3_AmpToNote;
import frc.robot.commands.auto.BlueAlliance3.B3_StartToSpeaker;

import frc.robot.commands.auto.BlueAlliance1.Plays.B1_StartAmpNoteSpeaker;

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
  private final static Joystick intakeJoy = new Joystick(2);
  private final static Joystick intakeXbox = new Joystick(3);
  // Chooser for testing teleop commands
  private final SendableChooser<Command> teleopCommandChooser = new SendableChooser<>();

  private final SendableChooser<Command> autoCommandChooser = new SendableChooser<>();
  

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
  // Empty Auto object
  // Empty SwerveTeleop object
  private SwerveTeleop teleop;
  // Empty CrabDrive object
  private CrabDrive crabDrive;

  private Shooter shooter;

  // Auto Trajectories
  private final SwerveAuto auto;
  private final B1_StartToAmp B1_StartToAmp;
  private final B1_NoteToSpeaker B1_NoteToSpeaker;
  private final B1_SpeakerToNote B1_SpeakerToNote;
  private final B1_NoteToAmp B1_NoteToAmp;
  private final B1_AmpToNote B1_AmpToNote;
  
  private final B2_StartToAmp B2_StartToAmp;
  private final B2_NoteToSpeaker B2_NoteToSpeaker;
  private final B2_SpeakerToNote B2_SpeakerToNote;
  private final B2_AmpToNote B2_AmpToNote;
  private final B2_StartToSpeaker B2_StartToSpeaker;
  
  private final B3_StartToAmp B3_StartToAmp;
  private final B3_NoteToAmp B3_NoteToAmp;
  private final B3_NoteToSpeaker B3_NoteToSpeaker;
  private final B3_SpeakerToNote B3_SpeakerToNote;
  private final B3_AmpToNote B3_AmpToNote;
  private final B3_StartToSpeaker B3_StartToSpeaker;
  
  private final B1_StartAmpNoteSpeaker B1_StartAmpNoteSpeaker;

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
        //chaging the variable below:
        // true = field centric
        // false = robot centric
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
        return true;
      }, setAlliance, blueAllianceOrNot);

    }

    crabDrive = new CrabDrive(this.swerve, () -> {
      return -this.actualXbox.getX();
    }, () -> {
      return -this.actualXbox.getY();
    });

    allFour = new TestFourModules(swerve, actualXbox);
    shooter = new Shooter();
    teleopCommandChooser.addOption("Regular Teleop", teleop);
    teleopCommandChooser.addOption("Crab Teleop", crabDrive);
    teleopCommandChooser.addOption("Module Test Command", allFour);
    teleopCommandChooser.setDefaultOption("Regular Teleop", teleop);
   
    if (autoOrNot) {
      // driveForward = new SwerveAuto("B1 Note to Amp", this.swerve);
      auto = new SwerveAuto("DriveForward (test)", this.swerve);
      B1_StartToAmp = new B1_StartToAmp("B1 Start to Amp", this.swerve);
      B1_NoteToAmp = new B1_NoteToAmp("B1 Note to Amp", this.swerve);
      B1_NoteToSpeaker = new B1_NoteToSpeaker("B1 Note to Speaker", this.swerve);
      B1_AmpToNote = new B1_AmpToNote("B1 Amp To Note", this.swerve);
      B1_SpeakerToNote = new B1_SpeakerToNote("B1 Speaker to Note", this.swerve);
  
      B2_AmpToNote = new B2_AmpToNote("B2 Amp to Note", this.swerve);
      B2_StartToAmp = new B2_StartToAmp("B2 Start to Amp", this.swerve);
      B2_NoteToSpeaker = new B2_NoteToSpeaker("B2 Note to Speaker", this.swerve);
      B2_SpeakerToNote = new B2_SpeakerToNote("B2 Speaker to Note", this.swerve);
      B2_StartToSpeaker = new B2_StartToSpeaker("B2 Start to Speaker", this.swerve);
  
      B3_AmpToNote = new B3_AmpToNote("B3 Amp to Note", this.swerve);
      B3_NoteToAmp = new B3_NoteToAmp("B3 Note to Amp", this.swerve);
      B3_StartToAmp = new B3_StartToAmp("B3 Start to Amp", this.swerve);
      B3_NoteToSpeaker = new B3_NoteToSpeaker("B3 Note to Speaker", this.swerve);
      B3_SpeakerToNote = new B3_SpeakerToNote("B3 Speaker to Note", this.swerve);
      B3_StartToSpeaker = new B3_StartToSpeaker("B3 Start to Speaker", this.swerve);
      
      B1_StartAmpNoteSpeaker = new B1_StartAmpNoteSpeaker("B1 Start to Amp", this.swerve);
    }

    // Autonomous command selector
  autoCommandChooser.addOption("B1_StartToAmp", B1_StartToAmp);
  autoCommandChooser.addOption("B1_NoteToAmp", B1_NoteToAmp);
  autoCommandChooser.addOption("B1_NoteToSpeaker", B1_NoteToSpeaker);
  autoCommandChooser.addOption("B1_AmpToNote", B1_AmpToNote);
  autoCommandChooser.addOption("B1_SpeakerToNote", B1_SpeakerToNote);
  autoCommandChooser.addOption("B1_StartAmpNoteSpeaker", B1_StartAmpNoteSpeaker);

  autoCommandChooser.addOption("B2_AmpToNote", B2_AmpToNote);
  autoCommandChooser.addOption("B2_StartToAmp", B2_StartToAmp);
  autoCommandChooser.addOption("B2_NoteToSpeaker", B2_NoteToSpeaker);
  autoCommandChooser.addOption("B2_SpeakerToNote", B2_SpeakerToNote);
  autoCommandChooser.addOption("B2_StartToSpeaker", B2_StartToSpeaker);

  autoCommandChooser.addOption("B3_AmpToNote", B3_AmpToNote);
  autoCommandChooser.addOption("B3_NoteToAmp", B3_NoteToAmp);
  autoCommandChooser.addOption("B3_StartToAmp", B3_StartToAmp);
  autoCommandChooser.addOption("B3_NoteToSpeaker", B3_NoteToSpeaker);
  autoCommandChooser.addOption("B3_SpeakerToNote", B3_SpeakerToNote);
  autoCommandChooser.addOption("B3_StartToSpeaker", B3_StartToSpeaker);


  SmartDashboard.putData(teleopCommandChooser);
  SmartDashboard.putData(autoCommandChooser);
  this.configureBindings();
  }

  private void configureBindings() {
    JoystickButton triggerIntake = new JoystickButton(intakeJoy, 1);
    triggerIntake.onTrue(new IntakeBeamBreak(0.8, shooter));
    JoystickButton stopIntake = new JoystickButton(intakeJoy, 2);
    stopIntake.onTrue(new StopIntake(shooter));

    JoystickButton triggerManualIntake = new JoystickButton(intakeJoy, 13);
    triggerManualIntake.whileTrue(new IntakeManual(1.0, shooter));
    JoystickButton triggerShooterButton = new JoystickButton(intakeJoy, 13);
    triggerShooterButton.whileTrue(new Shoot(2500, -2500, shooter));
  }

  public Command getAutonomousCommand() {
    return autoCommandChooser.getSelected();
    // return B1_StartAmpNoteSpeaker;
    // return B1_StartToAmp;
    // return auto;
    
  }

  public void initCommandInTeleop() {
    swerve.setDefaultCommand(teleopCommandChooser.getSelected());
  }
  public static Joystick getIntakeJoy(){
    return intakeJoy;
  }
  public static Joystick getIntakeXbox(){
    return intakeXbox;
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