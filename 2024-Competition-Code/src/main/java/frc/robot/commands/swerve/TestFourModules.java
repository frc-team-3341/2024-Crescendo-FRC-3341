// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.swerve;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.swerve.SwerveDrive;

public class TestFourModules extends Command {

  private SwerveDrive swerve;
  private Joystick joy;

  private SendableChooser<String> pidOrPowerMode;

  private String[] modeStrings = {"Voltage control", "PIDF control"};

  private int index = 0;

  private double driveVoltage, turnVoltage = 0.0;
  private double velocity, angle = 0.0;

  /*
   * TODO for the future people:
   * This may sound impossible or crazy for you, but you can do it:
   * Create an automated testing routine that logs all the data. In industry, this is done everywhere with FPGAs and other devices. Suggested by Mr. Rongey 12/16/2023.
   */

  /** Creates a new TestFourModules. */
  public TestFourModules(SwerveDrive swerve, Joystick joy) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.swerve = swerve;
    this.joy = joy;
    pidOrPowerMode = new SendableChooser<String>();
    addRequirements(swerve);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    pidOrPowerMode.addOption(modeStrings[0], modeStrings[0]);
    pidOrPowerMode.addOption(modeStrings[1], modeStrings[1]);
    pidOrPowerMode.setDefaultOption(modeStrings[0], modeStrings[0]);
    SmartDashboard.putData(pidOrPowerMode);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Voltage mode
    if (pidOrPowerMode.getSelected().equals(modeStrings[0])) {
      if (Constants.currentRobot.xboxEnabled) {
        driveVoltage = 1.0;
        // Driving to 1.0 - Left bumper (upper)
        if (joy.getRawButton(XboxController.Button.kLeftBumper.value)) {
          driveVoltage = 8.0;
        }
  
        // Driving to 4.0 - Left trigger bumper (lower)
        else if (joy.getRawAxis(XboxController.Axis.kLeftTrigger.value) > 0.5) {
          driveVoltage = 4.0;
        }
  
        // Driving to 6.0 - Right bumper (upper)
        else if (joy.getRawButton(XboxController.Button.kRightBumper.value)) {
          driveVoltage = 6.0;
        }
  
        // Driving to 12.0 - Right trigger bumper (lower)
        else if (joy.getRawAxis(XboxController.Axis.kRightTrigger.value) > 0.5) {
          driveVoltage = 12.0;
        }
  
        // Turning to 1.0 - Button X
        else if (joy.getRawButton(XboxController.Button.kX.value)) {
          turnVoltage = 1.0;
        }
  
        // Turning to 4.0 - Button A
        else if (joy.getRawButton(XboxController.Button.kA.value)) {
          turnVoltage = 4.0;
        }
  
        // Turning to 8.0 - Button B
        else if (joy.getRawButton(XboxController.Button.kB.value)) {
          turnVoltage = 8.0;
        }
  
        // Turning to 12.0 - Button Y
        else if (joy.getRawButton(XboxController.Button.kY.value)) {
          turnVoltage = 12.0;
        }

        swerve.setModuleVoltage(driveVoltage*-joy.getRawAxis(1), turnVoltage*-joy.getRawAxis(5), index);
      }
    
    // Velocity mode
    } else if (pidOrPowerMode.getSelected().equals(modeStrings[1])) {

      if (Constants.currentRobot.xboxEnabled) {
        // Driving to 1.0 - Left bumper (upper)
        if (joy.getRawButton(XboxController.Button.kLeftBumper.value)) {
          velocity = 1.0;
        }

        // Driving to 2.0 - Left trigger bumper (lower)
        else if (joy.getRawAxis(XboxController.Axis.kLeftTrigger.value) > 0.5) {
          velocity = 2.0;
        }

        // Driving to 3.0 - Right bumper (upper)
        else if (joy.getRawButton(XboxController.Button.kRightBumper.value)) {
          velocity = 3.0;
        }

        // Driving to 3.7 - Right trigger bumper (lower)
        else if (joy.getRawAxis(XboxController.Axis.kRightTrigger.value) > 0.5) {
          velocity = Constants.ModuleConstants.maxFreeWheelSpeedMeters;
        }

        // Turning to pi/4 - Button X
        else if (joy.getRawButton(XboxController.Button.kX.value)) {
          angle = Math.PI/4;
        }

        // Turning to pi/2 - Button A
        else if (joy.getRawButton(XboxController.Button.kA.value)) {
          angle = 2*Math.PI/4;
        }

        // Driving to 3*pi/4 - Button B
        else if (joy.getRawButton(XboxController.Button.kB.value)) {
          angle = 3*Math.PI/4;
        }

        // Driving to pi - Button Y
        else if (joy.getRawButton(XboxController.Button.kY.value)) {
          angle = Math.PI;
        }
      }

      swerve.setModuleSetpoints(velocity*-joy.getRawAxis(1), angle*-joy.getRawAxis(5), index);
    }

    if (joy.getRawButtonPressed(XboxController.Button.kLeftStick.value)) {
      swerve.setModuleSetpoints(0, 0, index);
      swerve.setModuleVoltage(0, 0, index);
      if (index == 3) {
        index = 0;
      } else {
        index++;
      }
      
      SmartDashboard.putNumber("Index of button", index);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // PLEASE SET THIS FOR SAFETY!!!
    this.swerve.stopMotors();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}