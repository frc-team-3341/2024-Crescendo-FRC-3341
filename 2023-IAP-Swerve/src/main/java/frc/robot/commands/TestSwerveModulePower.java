// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.swerve.SingularModule;

public class TestSwerveModulePower extends CommandBase {

  private SingularModule moduleSubsystem;

  // Create suppliers as object references
  private DoubleSupplier drivePowerSup;
  private DoubleSupplier turnPowerSup;

  private Joystick joy;

  private double turnVoltage, driveVoltage;

  /** Creates a new TestSwerveModule. */
  public TestSwerveModulePower(SingularModule moduleSubsystem, DoubleSupplier drivePowerSup,
      DoubleSupplier turnPowerSup, Joystick joy) {

    this.drivePowerSup = drivePowerSup;
    this.turnPowerSup = turnPowerSup;

    this.joy = joy;

    this.moduleSubsystem = moduleSubsystem;

    addRequirements(moduleSubsystem);

    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    moduleSubsystem.module.setDriveVoltage(0.0);
    moduleSubsystem.module.setTurnVoltage(0.0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    driveVoltage = 0.0;
    turnVoltage = 0.0;

    // Driving to 1.0 - Left bumper (upper)
    if (joy.getRawButton(4)) {
      driveVoltage = 1.0;
    }

    // Driving to 2.0 - Left trigger bumper (lower)
    if (joy.getRawButton(6)) {
      driveVoltage = 4.0;
    }

    // Driving to 3.0 - Right bumper (upper)
    if (joy.getRawButton(5)) {
      driveVoltage = 8.0;
    }

    // Driving to 4.0 - Right trigger bumper (lower)
    if (joy.getRawButton(7)) {
      driveVoltage = 12.0;
    }

    // Driving to pi/4 - Button X
    if (joy.getRawButton(0)) {
      turnVoltage = 1.0;
    }

    // Driving to pi/2 - Button A
    if (joy.getRawButton(1)) {
      turnVoltage = 4.0;
    }

    // Driving to 3*pi/4 - Button B
    if (joy.getRawButton(2)) {
      turnVoltage = 8.0;
    }

    // Driving to 4*pi/4 - Button Y
    if (joy.getRawButton(3)) {
      turnVoltage = 12.0;
    }
    
    moduleSubsystem.module.setDriveVoltage(drivePowerSup.getAsDouble() * driveVoltage);
    moduleSubsystem.module.setTurnVoltage(turnPowerSup.getAsDouble() * turnVoltage);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    moduleSubsystem.module.setDriveVoltage(0.0);
    moduleSubsystem.module.setTurnVoltage(0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
