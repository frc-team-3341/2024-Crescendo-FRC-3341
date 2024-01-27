// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.swerve.Intake;

public class IntakeCommand extends Command {
  /** Creates a new IntakeCommand. */
  private double power;
  private Intake intake;
  public IntakeCommand(double power, Intake intake) {
    this.power = power;
    this.intake = intake;
    addRequirements(intake);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    intake.resetEncoders();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    intake.setSpeedSimple(power);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intake.setSpeedSimple(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return intake.getSensor();
  }
}
