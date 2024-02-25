// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;

public class IntakeSource extends Command {
  private double upperRPM;
  private double lowerRPM;
  private double intakePower;
  private Shooter shooter;
  private boolean intakeBeamActivated;
  /** Creates a new IntakeSource. */
  public IntakeSource(double upperRPM, double lowerRPM, double intakePower, Shooter shooter) {
    this.upperRPM = upperRPM;
    this.lowerRPM = lowerRPM;
    this.intakePower = intakePower;
    this.shooter = shooter;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    intakeBeamActivated = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(!intakeBeamActivated){
      shooter.setUpperRPM((int)upperRPM);
      shooter.setLowerRPM((int)lowerRPM);
      shooter.setIntakePower(-intakePower);
    }
    else{
      shooter.setUpperRPM(0);
      shooter.setLowerRPM(0);
      shooter.setIntakePower(intakePower);
    }
    if(shooter.getIntakeBeam()){
      intakeBeamActivated = true;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
      shooter.setUpperRPM(0);
      shooter.setLowerRPM(0);
      shooter.setIntakePower(0);
  }

  
  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (intakeBeamActivated && shooter.getShooterBeam());
  }
}
