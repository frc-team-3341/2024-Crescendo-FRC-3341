// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Intake;

public class IntakeComm extends Command {
  /** Creates a new IntakeCommand. */

public Intake intake = new Intake();
public double power = 0;
  public IntakeComm(Intake intake, double power) {
    this.intake = intake;
    this.power = power;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Preferences.initDouble("power", power);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(RobotContainer.getJoystickCommand().getRawButtonPressed(10)){
        intake.setFlywheelPower(power);
        intake.getBeambreak1();
      }
      power = Preferences.getDouble("power", power);
      
    
  }



  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intake.setFlywheelPower(0); //at the end, the speed stops
  }
}
