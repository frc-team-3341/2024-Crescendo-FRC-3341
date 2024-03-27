// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.climber;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Climber;

public class BasicClimbTeleop extends Command {

  Climber climber;
  Joystick joystick;

  /** Creates a new ClimbTeleop. */
  public BasicClimbTeleop(Climber climber, Joystick joystick) {
    this.climber = climber;
    this.joystick = joystick;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(climber);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    climber.resetEncoder();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    // Read y value from joystick's Y axis
    // Negate the value of the flight controller's joystick, as it is configured for flight (a different direction than our intuitive understanding)
    double yValue = -joystick.getY();
    
    climber.extendArmWithPower(yValue);

//    boolean nearLowerLimit = (1.9 <= climber.getEncoderInches()) || (climber.getEncoderInches() <= 2.1);
//    boolean nearUpperLimit = (24.45 <= climber.getEncoderInches()) || (climber.getEncoderInches() <= 24.65);
    //Turning encoder ticks into the distance from the top or the bottom limit switch.
    //Ideal values --> 2 in from the top and bottom limit switches

//    if (nearLowerLimit || nearUpperLimit){
//      climber.extendArmWithPower(0);
//    }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    climber.extendArmWithPower(0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
