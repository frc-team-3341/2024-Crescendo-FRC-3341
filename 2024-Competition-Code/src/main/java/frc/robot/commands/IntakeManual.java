// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;

public class IntakeManual extends Command {
  /** Creates a new IntakeManual. */
  private double power;
  private Intake intake;
  private Joystick joy;
  public IntakeManual(double power, Intake intake, Joystick joy) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.power = power;
    this.intake = intake;
    this.joy = joy;
    addRequirements(intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

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
    return (joy.getRawButtonPressed(3));
  }
}
