package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.SwerveAuto;
import frc.robot.commands.SwerveTeleop;
import frc.robot.commands.TestSwerveModulePIDF;
import frc.robot.commands.TestSwerveModulePower;
import frc.robot.subsystems.swerve.SingularModule;
import frc.robot.subsystems.swerve.SwerveDrive;
import frc.robot.subsystems.swerve.SwerveModuleIO;
import frc.robot.subsystems.swerve.SwerveModuleIOSim;
import frc.robot.subsystems.swerve.SwerveModuleIOSparkMax;

public class RobotContainer {

  /*
   * TO THE FUTURE READERS/REVIEWERS OF THIS FILE:
   * Feel free to use any parts of this project or its entirety in a Competition FRC robot of any kind or team. 
   * This codebase is 95% Competition-ready (minus some minor cosmetic things). It is designed so that the modules are modular (meaning easy to switch).
   * This technique enables us to simulate the swerve drivebase and develop at home to our heart's content.
   * In the future, we can also write a SwerveModuleIOTalonFX.java as well and easily "plug" it in.
   *  
   * Minor warning: advanced Java syntax that this project uses:
   * - Java Lambdas
   * - Java Suppliers and Consumers
   * - Java Interface Classes
   * - Java For-Each Loops
   */

  // WARNING: TRAJECTORY DRIVING NOT TESTED IN REAL LIFE (IRL)
  // DO NOT USE UNTIL DRIVING IN SAFE SPACE
  // THIS IS A SECOND WARNING!!! THIS IS VERY DANGEROUS.
  // To do trajectory driving or not
  private final boolean autoOrNot = false;

  private final boolean setAlliance = false;
  private final boolean blueAllianceOrNot = true;

  // Confirmed ready for testing 12/9
  // Switches to single module testing mode
  private final boolean testSingleModule = true;
  private final int testModuleIndex = 0;
  private final boolean testPIDF = false;

  public static final boolean isXbox = false;

  private SwerveAuto auto;

  // Xbox + an additional one for PC use
  private final Joystick actualXbox = new Joystick(0);
  private final Joystick additionalJoy = new Joystick(1);

  // Define axises for using joystick
  private final int translationAxis = 0;
  private final int strafeAxis = 1;
  private final int rotationAxis = 4; // For xBox

  // private final int rotationAxis = 0;
  private static boolean isSim = Robot.isSimulation();
  
  SingularModule module;

  // Creates array of swerve modules for use in SwerveDrive object
  SwerveModuleIO[] swerveMods = new SwerveModuleIO[4];

  // Empty SwerveDrive object
  public SwerveDrive swerve;
  
  // Empty testing commands (not used if not needed)
  public TestSwerveModulePower powerCommand;
  public TestSwerveModulePIDF pidfCommand;

  public RobotContainer() {
  
    // Initialize SwerveDrive object with modules
    if (!testSingleModule) {
      if (isSim) {
        // Construct swerve modules with simulated motors
        for (int i = 0; i < swerveMods.length; i++) {
          swerveMods[i] = new SwerveModuleIOSim(i);
        }

      } else {
        // Construct swerve modules with real motors
        for (int i = 0; i < swerveMods.length; i++) {
          swerveMods[i] = new SwerveModuleIOSparkMax(i, Constants.SwerveConstants.moduleCANIDs[i][0], Constants.SwerveConstants.moduleCANIDs[i][1], Constants.SwerveConstants.moduleCANIDs[i][2], Constants.SwerveConstants.moduleAngleOffsets[i]);
        }

      }

      this.swerve = new SwerveDrive(this.swerveMods[0], this.swerveMods[1], this.swerveMods[2], this.swerveMods[3]);
      if (isXbox) {
        // Supply teleop command with joystick methods - USES LAMBDAS
        this.swerve.setDefaultCommand(new SwerveTeleop(this.swerve, () -> {
          return -this.actualXbox.getRawAxis(translationAxis);
        }, () -> {
          return -this.actualXbox.getRawAxis(strafeAxis);
        }, () -> {
          return -this.actualXbox.getRawAxis(rotationAxis);
        }, () -> {
          return true;
        }, setAlliance, blueAllianceOrNot));

      } else if (!isXbox) {
        // Supply teleop command with joystick methods - USES LAMBDAS
        this.swerve.setDefaultCommand(new SwerveTeleop(this.swerve, () -> {
          return -this.actualXbox.getX();
        }, () -> {
          return -this.actualXbox.getY();
        }, () -> {
          return -this.additionalJoy.getRawAxis(0);
        }, () -> {
          return true;
        }, setAlliance, blueAllianceOrNot));

      }

    // Else if testing singular module
    } else {
        if (!isSim) {
          module = new SingularModule(new SwerveModuleIOSparkMax(testModuleIndex, Constants.SwerveConstants.moduleCANIDs[testModuleIndex][0], Constants.SwerveConstants.moduleCANIDs[testModuleIndex][1], Constants.SwerveConstants.moduleCANIDs[testModuleIndex][2], 0));
        } else {
          module = new SingularModule(new SwerveModuleIOSim(0));
        }

        if (testPIDF) {
          pidfCommand = new TestSwerveModulePIDF(module, actualXbox);
          module.setDefaultCommand(pidfCommand);

        } else {
            if (isXbox) {
              powerCommand = new TestSwerveModulePower(module,
                () -> {
                  return -this.actualXbox.getRawAxis(translationAxis);
                }, 
                () -> {
                  return -this.actualXbox.getRawAxis(5);
                }, actualXbox);
            } else {
              powerCommand = new TestSwerveModulePower(module,
                () -> {
                  return -this.actualXbox.getRawAxis(translationAxis);
                }, 
                () -> {
                  return -this.additionalJoy.getRawAxis(0);
                }, actualXbox);
            }
          module.setDefaultCommand(powerCommand);

        }
      
    }
    
    if (autoOrNot) {
      auto = new SwerveAuto("New Path", this.swerve);
    }
    this.configureBindings();
  }

  private void configureBindings() {
  }

  public Command getAutonomousCommand() {
    if (!testSingleModule) {
      if (autoOrNot) {
        return auto;
      } else {
        return null;
      }
    } else {
      return null;
    }
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