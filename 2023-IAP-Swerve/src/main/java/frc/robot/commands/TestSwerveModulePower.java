// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SingularModule;

public class TestSwerveModulePower extends CommandBase {

  private SingularModule moduleSubsystem;

  // Create suppliers as object references
  private DoubleSupplier drivePowerSup;
  private DoubleSupplier turnPowerSup;

  /** Creates a new TestSwerveModule. */
  public TestSwerveModulePower(SingularModule moduleSubsystem, DoubleSupplier drivePowerSup, DoubleSupplier turnPowerSup) {

    this.drivePowerSup = drivePowerSup;
    this.turnPowerSup = turnPowerSup;

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
    moduleSubsystem.module.setDriveVoltage(drivePowerSup.getAsDouble()*12.0);
    moduleSubsystem.module.setTurnVoltage(turnPowerSup.getAsDouble()*12.0);
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
