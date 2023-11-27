package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.SwerveAuto;
import frc.robot.commands.SwerveTeleop;
import frc.robot.commands.TestSwerveModulePower;
import frc.robot.subsystems.swerve.SingularModule;
import frc.robot.subsystems.swerve.SwerveDrive;
import frc.robot.subsystems.swerve.SwerveModuleIO;
import frc.robot.subsystems.swerve.SwerveModuleIOSim;
import frc.robot.subsystems.swerve.SwerveModuleIOSparkMax;

public class RobotContainer {

  // To do trajectory driving or not
  private final boolean autoOrNot = true; // Set true to test auto!

  private final boolean testSingleModule = false;

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

  // Creates array of swerve modules for use in SwerveDrive object
  SwerveModuleIO[] swerveMods = new SwerveModuleIOSim[] {
      new SwerveModuleIOSim(0),
      new SwerveModuleIOSim(1),
      new SwerveModuleIOSim(2),
      new SwerveModuleIOSim(3) };

  // Empty SwerveDrive object
  public SwerveDrive swerve;
  
  public TestSwerveModulePower power;

  public RobotContainer() {
    // Initialize SwerveDrive object with modules
    if (!testSingleModule) {
      this.swerve = new SwerveDrive(this.swerveMods[0], this.swerveMods[1], this.swerveMods[2], this.swerveMods[3]);
      if (isXbox) {
        // Supply teleop command with joystick methods
        this.swerve.setDefaultCommand(new SwerveTeleop(this.swerve, () -> {
          return -this.actualXbox.getRawAxis(translationAxis);
        }, () -> {
          return -this.actualXbox.getRawAxis(strafeAxis);
        }, () -> {
          return -this.actualXbox.getRawAxis(rotationAxis);
        }, () -> {
          return true;
        }));
      } else if (!isXbox) {        // Supply teleop command with joystick methods
        this.swerve.setDefaultCommand(new SwerveTeleop(this.swerve, () -> {
          return -this.actualXbox.getX();
        }, () -> {
          return -this.actualXbox.getY();
        }, () -> {
          return -this.additionalJoy.getRawAxis(0);
        }, () -> {
          return true;
        }));
      }
    } else {
        SingularModule module;
       /*  if (isSim) {
          module = new SingularModule(new SwerveModuleIOSim(0));
          power = new TestSwerveModulePower(module,
          () -> {
            return this.actualXbox.getRawAxis(translationAxis);
          }, 
          () -> {
            return this.additionalJoy.getRawAxis(0);
          });
        } */
        if (isXbox) {
          module = new SingularModule(new SwerveModuleIOSparkMax(0, 1, 2, 3, 0));
          power = new TestSwerveModulePower(module,
            () -> {
              return -this.actualXbox.getRawAxis(translationAxis);
            }, 
            () -> {
              return -this.actualXbox.getRawAxis(5);
            });
        } else {
          module = new SingularModule(new SwerveModuleIOSparkMax(0, 1, 2, 3, 0));
          power = new TestSwerveModulePower(module,
            () -> {
              return -this.actualXbox.getRawAxis(translationAxis);
            }, 
            () -> {
              return -this.additionalJoy.getRawAxis(0);
            });
        }
        module.setDefaultCommand(power);
    }
    
    auto = new SwerveAuto("New Path", this.swerve);
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