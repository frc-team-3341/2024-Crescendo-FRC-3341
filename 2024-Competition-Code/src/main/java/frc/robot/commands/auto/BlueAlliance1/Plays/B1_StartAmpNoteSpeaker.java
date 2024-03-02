// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto.BlueAlliance1.Plays;

import com.pathplanner.lib.util.PIDConstants;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.commands.IntakeBeamBreak;
import frc.robot.commands.Shoot;
import frc.robot.commands.auto.AutoPath;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.swerve.SwerveDrive;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class B1_StartAmpNoteSpeaker extends SequentialCommandGroup {
  SwerveDrive swerve;
  Shooter shooter;
  AutoPath autoPath;
  /** Creates a new B1_StartAmpNoteSpeaker. */
  public B1_StartAmpNoteSpeaker(SwerveDrive swerve, Shooter shooter) {
    this.swerve = swerve;
    this.shooter = shooter;
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new AutoPath("B1 Start to Amp", this.swerve, new PIDConstants(1.0, 0, 0), new PIDConstants(1.0, 0, 0)),
     // new Shoot(120, 80, this.shooter).withTimeout(3.0),
      new AutoPath("B1 Amp to Note", this.swerve, new PIDConstants(1.0, 0, 0), new PIDConstants(1.0, 0, 0)),
     // new IntakeBeamBreak(1.0, this.shooter).withTimeout(3.0), //Automatically stops
      new AutoPath("B1 Note to Speaker", this.swerve, new PIDConstants(1.0, 0, 0), new PIDConstants(1.0, 0, 0))
      //new Shoot(3500, 3500, this.shooter).withTimeout(3.0));
    );
  }
}