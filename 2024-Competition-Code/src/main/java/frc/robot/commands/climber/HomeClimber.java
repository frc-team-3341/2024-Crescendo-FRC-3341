// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.climber;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Climber;

public class HomeClimber extends Command {

  Climber climber;

  Timer timer;

  private final double climberTimeout = 0.2;

  /** Creates a new HomeClimber. */
  public HomeClimber(Climber climber) {
    this.climber = climber;

    timer = new Timer();

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(climber);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.reset();
    timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    climber.extendArmWithPower(-0.1);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    climber.extendArmWithPower(0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // If timer is greater than a certain time, then the command will stop for safety reasons
    // I.e. what if the limit switch breaks?
    return climber.reverseLimit.isPressed() || (timer.get() > climberTimeout);
  }
}
