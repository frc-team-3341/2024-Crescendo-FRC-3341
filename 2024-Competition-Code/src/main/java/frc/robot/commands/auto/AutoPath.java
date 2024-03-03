package frc.robot.commands.auto;
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.


import java.io.FileNotFoundException;
import java.io.FileReader;
import java.io.IOException;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.PathPlannerLogging;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.swerve.SwerveDrive;

public class AutoPath extends SequentialCommandGroup {

  SwerveDrive swerve;

  Pose2d initialPose;
  /**
   * Creates a new SwerveAuto.
   * 
   * @param pathName Name of path in RIO's data folder
   * @param swerve   SwerveDrive subsystem
   */
  public AutoPath(String pathName, SwerveDrive swerve, PIDConstants translational, PIDConstants rotational, Pose2d initialPose) {
    
    this.swerve = swerve;

    this.initialPose = initialPose;
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
            translational, // Translation PID constants -> path independent
            rotational, // Rotation PID constants -> more or less path dependent
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

          var alliance = DriverStation.getAlliance();
         if (alliance.isPresent()) {
            return alliance.get() == DriverStation.Alliance.Red;
          }
          return false;
        },
        this.swerve // Reference to this subsystem to set requirements
    );


    // Set at initial pose of auto
    swerve.resetPose(initialPose);
    // swerve.resetPose(new Pose2d(new Translation2d(0.71, 6.71), swerve.getRotation()));
    //path.getPreviewStartingHolonomicPose();

  
    var swerveAuto = AutoBuilder.followPath(path);

    // Setting voltage to 0 is necessary in order to stop robot
    addCommands(swerveAuto.finallyDo(() -> {
      swerve.setModulesPositions(0,0); swerve.setModuleVoltages(0, 0);})
      );
  }
}