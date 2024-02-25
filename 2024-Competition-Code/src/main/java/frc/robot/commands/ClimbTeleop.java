// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Climber;

public class ClimbTeleop extends Command {

  Climber climber;
  DoubleSupplier joySupplier;
  BooleanSupplier switchToVelocityControl;

  /** Creates a new ClimbTeleop. */
  public ClimbTeleop(Climber climber, DoubleSupplier joystickInput, BooleanSupplier switchToVelocityControl) {
    this.climber = climber;
    this.joySupplier = joystickInput;
    this.switchToVelocityControl = switchToVelocityControl;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(climber);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    climber.extendArmWithVelocity(0.0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    double input = joySupplier.getAsDouble();
    boolean toggle = switchToVelocityControl.getAsBoolean();


    input = MathUtil.applyDeadband(input, 0.05);

    if (toggle) {
      climber.extendArmWithVelocity(-input*Constants.ClimberConstants.maxExtensionVelocity);
    } else {
      climber.extendArmWithPower(-input);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    climber.extendArmWithVelocity(0.0);
    climber.extendArmWithPower(0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
