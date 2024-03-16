// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.swerve.BackingUpIntoAmp;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.swerve.SwerveDrive;

public class MoveBackIntoAmp extends Command {

  SwerveDrive swerve;

  Pose2d initialPose;

  double currentDisplacement;

  PIDController displacementController = new PIDController(Constants.BackingUpConstants.backingkP,
      Constants.BackingUpConstants.backingkI, Constants.BackingUpConstants.backingkD);

  /** Creates a new MoveBackThreeInches. */
  public MoveBackIntoAmp(SwerveDrive swerve) {

    this.swerve = swerve;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(swerve);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    initialPose = swerve.getPoseFromEstimator();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    double calculatedXVelocity = displacementController.calculate(currentDisplacement,
        Constants.BackingUpConstants.targetBackingDisplacement);

    // Back up by a certain sign of direction
    swerve.driveRelative(new ChassisSpeeds(Constants.BackingUpConstants.direction.sign*calculatedXVelocity, 0, 0));

    // Get current pose
    Pose2d currentPose = swerve.getPoseFromEstimator();

    // Calculate translation between initial and current pose
    Translation2d deltaTranslation = initialPose.minus(currentPose).getTranslation();

    // Get the Pythagorean hypotenuse of the current delta translation
    currentDisplacement = Math.abs(deltaTranslation.getNorm());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    swerve.stopMotors();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (currentDisplacement > Constants.BackingUpConstants.targetBackingDisplacement);
  }
}
