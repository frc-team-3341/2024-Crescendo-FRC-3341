// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.notemechanism;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;

public class IntakeBeamBreak extends Command {
  /** Creates a new IntakeCommand. */
  private double power;
  private Shooter shooter;
  public IntakeBeamBreak(double power, Shooter shooter) {
    this.power = power;
    this.shooter = shooter;
    addRequirements(shooter);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //shooter.setintakeSpeed(power);
<<<<<<<< HEAD:2024-Competition-Code/src/main/java/frc/robot/commands/IntakeBeamBreak.java
    shooter.setFeedSimple(power);
========
    shooter.setIntakePower(power);
>>>>>>>> copy.of.competition.code:2024-Competition-Code/src/main/java/frc/robot/commands/notemechanism/IntakeBeamBreak.java
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
<<<<<<<< HEAD:2024-Competition-Code/src/main/java/frc/robot/commands/IntakeBeamBreak.java
    shooter.setintakeSpeed(0);
========
    shooter.setIntakePower(0);
>>>>>>>> copy.of.competition.code:2024-Competition-Code/src/main/java/frc/robot/commands/notemechanism/IntakeBeamBreak.java
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return shooter.getShooterBeam();
  }
}
