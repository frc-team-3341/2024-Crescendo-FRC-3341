// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.swerve.SwerveModuleIO;
import frc.robot.subsystems.swerve.SwerveModuleIOSim;

public class TestSwerveModulePIDF extends CommandBase {

  SwerveModuleIO module;

  // Create suppliers as object references
  private DoubleSupplier velocitySup;
  private DoubleSupplier angleSup;

  private double maxVelocity = 1.0;

  private double lastTime = Timer.getFPGATimestamp();

  private double angle = 0.0;

  /** Creates a new TestSwerveModule. */
  public TestSwerveModulePIDF() {

    module = new SwerveModuleIOSim(0);

    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    module.setDesiredState(new SwerveModuleState());
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double velocity = maxVelocity
        * MathUtil.applyDeadband(this.velocitySup.getAsDouble(), Constants.SwerveConstants.deadBand);

    if (Timer.getFPGATimestamp() - lastTime >= 0.2) {
      // Either adds or subtracts depending on sign
      angle += Units.degreesToRadians(1.0) * Math.signum(this.angleSup.getAsDouble());
      lastTime = Timer.getFPGATimestamp();
    }

    module.setDesiredState(new SwerveModuleState(velocity, new Rotation2d(angle)));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    module.setDesiredState(new SwerveModuleState());
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
