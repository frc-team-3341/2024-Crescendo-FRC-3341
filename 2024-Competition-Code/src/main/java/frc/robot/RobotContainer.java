package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.SwerveTeleop;
import frc.robot.commands.TargetAprilTag;
import frc.robot.subsystems.photonvision.PhotonVision;
import frc.robot.subsystems.swerve.SwerveDrive;

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
  // Empty SwerveDrive object
  private SwerveDrive swerve;
  // Empty testing commands (not used if not needed)
  // Empty SwerveTeleop object
  private SwerveTeleop teleop;
  // Empty CrabDrive object

  // Auto Trajectories

  public RobotContainer() {
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
