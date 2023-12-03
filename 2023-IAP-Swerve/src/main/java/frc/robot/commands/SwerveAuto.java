// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.swerve.SwerveDrive;

public class SwerveAuto extends SequentialCommandGroup {
  SwerveDrive swerve;

  /**<b>Deprecated</b> in 2024
   * <p>Creates a new SwerveAuto.</p>
   * @param pathName Name of path in RIO's data folder
   * @param swerve SwerveDrive subsystem
  */
  public SwerveAuto(String pathName, SwerveDrive swerve) {
    this.swerve = swerve;
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    // Load path from 2023 PathPlannerLib
    // WARNING: ONLY INSTALL PATHPLANNERLIB FROM 2023 LIBRARIES, NOT 2024
    // THIS CODE WILL BECOME DEPRECATED SOON :)
    PathPlannerTrajectory path = PathPlanner.loadPath(pathName, 4.0, 3.0);

    // Trick: convert to WPILib trajectory via states
    var traj = new Trajectory(path.getStates());

    // Set field's trajectory to the trajectory of the path
    this.swerve.getField().getObject("traj").setTrajectory(traj);

    // Defines a new PPSwerveControllerCommand
    // WILL BECOME DEPRECATED!!
    // TODO: NEED TO TUNE
    var swerveAuto = new PPSwerveControllerCommand(path,
        this.swerve::getPoseFromEstimator,
        new PIDController(1.1, 0, 0),
        new PIDController(1.1, 0, 0),
        new PIDController(1.1, 0, 0),
        this.swerve::driveRelative, this.swerve).andThen(() -> {
          this.swerve.stopMotors();
        });
    addCommands(swerveAuto);
  }
}
