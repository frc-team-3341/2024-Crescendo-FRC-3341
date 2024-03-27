// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.notemechanism;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;


public class Shoot extends Command {
  private int power;
  private int lowerRPM;
  private Shooter shooter;
  /** Creates a new Shoot. */
  public Shoot(int power, int lowerRPM, Shooter shooter) {
    this.power = power;
    this.shooter = shooter;
    this.lowerRPM = lowerRPM;
    addRequirements(shooter);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //shooter.shootSpeaker(power);
    shooter.shootBoth(power, lowerRPM);
    // if (shooter.setpointReached(shooter.getUpperRPM(), shooter.upperRPM) && (shooter.setpointReached(shooter.getLowerRPM(), shooter.lowerRPM))){
    //   shooter.setintakeSpeed(3000);
    // }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooter.setupperSpeed(0);
    shooter.setlowerSpeed(0);
    shooter.setFeedSimple(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
