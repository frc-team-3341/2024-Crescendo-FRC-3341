// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Climber;

public class ClimbTeleop extends Command {

  Climber climber;
  Joystick joystick;


  /** Creates a new ClimbTeleop. */
  public ClimbTeleop(Climber climber, Joystick joystick) {
    this.climber = climber;
    this.joystick = joystick;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(climber);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    //Preform homing sequence
    //Move down until the bottom limit switch is pressed
    while (!climber.reverseLimit.isPressed()){
      climber.extendArmWithVelocity(-1);
      climber.resetEncoder();
    }
    //Slightly move the arm up, so it isn't pressing the limit switch <-- be consistent
    climber.extendArmWithVelocity(1);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double yVal = joystick.getY();
    climber.extendArmWithVelocity(-yVal*Constants.ClimberConstants.maxExtensionVelocity);

    boolean nearLowerLimit = 1.9 <= climber.getEncoderInches() || climber.getEncoderInches() <= 2.1;
    boolean nearUpperLimit = 24.45 <= climber.getEncoderInches() || climber.getEncoderInches() <= 24.65;

    //Turning encoder ticks into the distance from the top or the bottom limit switch.
    //Ideal values --> 2 in from the top and bottom limit switches

    if (nearLowerLimit || nearUpperLimit){
      climber.extendArmWithVelocity(0);
    }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    climber.extendArmWithVelocity(0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
