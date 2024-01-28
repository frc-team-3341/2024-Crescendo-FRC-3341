// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.PathPlannerLogging;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.subsystems.swerve.SwerveDrive;

public class SwerveAuto extends SequentialCommandGroup {
  SwerveDrive swerve;

  /**
   * Creates a new SwerveAuto.
   * 
   * @param pathName Name of path in RIO's data folder
   * @param swerve   SwerveDrive subsystem
   */
  public SwerveAuto(String pathName, SwerveDrive swerve) {
    this.swerve = swerve;
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());

    // Load path from 2024 PathPlannerLib
    PathPlannerPath path = PathPlannerPath.fromPathFile(pathName);

    PathPlannerLogging.setLogActivePathCallback((poses) -> {
      // Do whatever you want with the poses here
      // Will automagically re-display the path every time teleop is started
      // Set field's trajectory to the trajectory of the path
      this.swerve.getField().getObject("traj").setPoses(poses);
    });

    // Uses the AutoBuilder to build an auto
    // TODO: NEED TO TUNE

    // Credit for comments: PathPlannerLib wiki!

    AutoBuilder.configureHolonomic(
        this.swerve::getPoseFromEstimator, // Robot pose supplier
        this.swerve::resetPose, // Method to reset odometry (will be called if your auto has a starting pose)
        this.swerve::getRobotRelativeSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
        this.swerve::driveRelative, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
        new HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should likely live in your Constants class
            new PIDConstants(1.0, 0.0, 0.0), // Translation PID constants -> path independent
            new PIDConstants(0.55, 0.0, 0.0), // Rotation PID constants -> more or less path dependent
            Constants.SwerveConstants.maxChassisTranslationalSpeed, // Max module speed, in m/s
            Constants.SwerveConstants.trackWidthHypotenuse, // Drive base radius in meters. Distance from robot center
                                                            // to furthest module.
            new ReplanningConfig() // Default path replanning config. See the API for the options here
        ),
        () -> {
          // Boolean supplier that controls when the path will be mirrored for the red
          // alliance
          // This will flip the path being followed to the red side of the field.
          // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

         // var alliance = DriverStation.getAlliance();
         // if (alliance.isPresent()) {
          //  return alliance.get() == DriverStation.Alliance.Red;
         // }
          return false;
        },
        this.swerve // Reference to this subsystem to set requirements
    );

    var swerveAuto = AutoBuilder.followPath(path);

    // Setting voltage to 0 is necessary in order to stop robot
    addCommands(swerveAuto.finallyDo(() -> {
      swerve.stopMotors();
    }));
  }
}
