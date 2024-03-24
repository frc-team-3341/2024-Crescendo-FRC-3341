// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import javax.swing.plaf.basic.BasicComboBoxUI.FocusHandler;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.auto.AutoPath;
import frc.robot.commands.auto.BlueAlliance1.Plays.B1_LeftShoot;
import frc.robot.commands.auto.BlueAlliance1.Plays.B1_LeftSpeakerNote;
import frc.robot.commands.auto.BlueAlliance1.Plays.B1_LeftSpeakerNoteSpeaker;
import frc.robot.commands.auto.BlueAlliance1.Plays.B1_MiddleShoot;
import frc.robot.commands.auto.BlueAlliance1.Plays.B1_MiddleSpeakerNote;
import frc.robot.commands.auto.BlueAlliance1.Plays.B1_MiddleSpeakerNoteSpeaker;
import frc.robot.commands.auto.BlueAlliance1.Plays.B1_RightShoot;
import frc.robot.commands.auto.BlueAlliance1.Plays.B1_RightSpeakerNote;
import frc.robot.commands.auto.BlueAlliance1.Plays.B1_RightSpeakerNoteSpeaker;
import frc.robot.commands.auto.BlueAlliance1.Plays.B1_ShootAndLeave;
import frc.robot.commands.auto.BlueAlliance2.Plays.B2_LeftShoot;
import frc.robot.commands.auto.BlueAlliance2.Plays.B2_LeftSpeakerNote;
import frc.robot.commands.auto.BlueAlliance2.Plays.B2_LeftSpeakerNoteSpeaker;
import frc.robot.commands.auto.BlueAlliance2.Plays.B2_MiddleShoot;
import frc.robot.commands.auto.BlueAlliance2.Plays.B2_MiddleSpeakerNote;
import frc.robot.commands.auto.BlueAlliance2.Plays.B2_MiddleSpeakerNoteSpeaker;
import frc.robot.commands.auto.BlueAlliance2.Plays.B2_RightShoot;
import frc.robot.commands.auto.BlueAlliance2.Plays.B2_RightSpeakerNote;
import frc.robot.commands.auto.BlueAlliance2.Plays.B2_RightSpeakerNoteSpeaker;
import frc.robot.commands.auto.BlueAlliance3.Plays.B3_LeftShoot;
import frc.robot.commands.auto.BlueAlliance3.Plays.B3_LeftSpeakerNote;
import frc.robot.commands.auto.BlueAlliance3.Plays.B3_LeftSpeakerNoteSpeaker;
import frc.robot.commands.auto.BlueAlliance3.Plays.B3_MiddleShoot;
import frc.robot.commands.auto.BlueAlliance3.Plays.B3_MiddleSpeakerNote;
import frc.robot.commands.auto.BlueAlliance3.Plays.B3_MiddleSpeakerNoteSpeaker;
import frc.robot.commands.auto.BlueAlliance3.Plays.B3_RightShoot;
import frc.robot.commands.auto.BlueAlliance3.Plays.B3_RightSpeakerNote;
import frc.robot.commands.auto.BlueAlliance3.Plays.B3_RightSpeakerNoteSpeaker;
import frc.robot.commands.auto.BlueAlliance2.Plays.B2_B1_MiddleSpeakerNoteSpeaker;

import frc.robot.commands.auto.RedAlliance.Plays.R1_LeftSpeakerNoteSpeaker;
import frc.robot.commands.auto.RedAlliance.Plays.R2_MiddleSpeakerNoteSpeaker;
import frc.robot.commands.auto.RedAlliance.Plays.R3_RightSpeakerNoteSpeaker;
import frc.robot.commands.auto.RedAlliance.Plays.R2_R3_MiddleSpeakerNoteSpeaker;
import frc.robot.commands.auto.*;

import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.swerve.SwerveDrive;

/** Add your docs here. */
public class InitializeAutoPaths {
    private final SwerveDrive swerve;
    private final Shooter shooter;

    // Plays:
    private final B1_LeftShoot B1_LeftShoot;
    private final B1_LeftSpeakerNote B1_LeftSpeakerNote;
    private final B1_LeftSpeakerNoteSpeaker B1_LeftSpeakerNoteSpeaker;
    private final B1_MiddleShoot B1_MiddleShoot;
    private final B1_MiddleSpeakerNote B1_MiddleSpeakerNote;
    private final B1_MiddleSpeakerNoteSpeaker B1_MiddleSpeakerNoteSpeaker;
    private final B1_RightShoot B1_RightShoot;
    private final B1_RightSpeakerNote B1_RightSpeakerNote;
    private final B1_RightSpeakerNoteSpeaker B1_RightSpeakerNoteSpeaker;

    private final B2_LeftShoot B2_LeftShoot;
    private final B2_LeftSpeakerNote B2_LeftSpeakerNote;
    private final B2_LeftSpeakerNoteSpeaker B2_LeftSpeakerNoteSpeaker;
    private final B2_MiddleShoot B2_MiddleShoot;
    private final B2_MiddleSpeakerNote B2_MiddleSpeakerNote;
    private final B2_MiddleSpeakerNoteSpeaker B2_MiddleSpeakerNoteSpeaker;
    private final B2_RightShoot B2_RightShoot;
    private final B2_RightSpeakerNote B2_RightSpeakerNote;
    private final B2_RightSpeakerNoteSpeaker B2_RightSpeakerNoteSpeaker;

    private final B3_LeftShoot B3_LeftShoot;
    private final B3_LeftSpeakerNote B3_LeftSpeakerNote;
    private final B3_LeftSpeakerNoteSpeaker B3_LeftSpeakerNoteSpeaker;
    private final B3_MiddleShoot B3_MiddleShoot;
    private final B3_MiddleSpeakerNote B3_MiddleSpeakerNote;
    private final B3_MiddleSpeakerNoteSpeaker B3_MiddleSpeakerNoteSpeaker;
    private final B3_RightShoot B3_RightShoot;
    private final B3_RightSpeakerNote B3_RightSpeakerNote;
    private final B3_RightSpeakerNoteSpeaker B3_RightSpeakerNoteSpeaker;

    private final R1_LeftSpeakerNoteSpeaker R1_LeftSpeakerNoteSpeaker;
    private final R2_MiddleSpeakerNoteSpeaker R2_MiddleSpeakerNoteSpeaker;
    private final R3_RightSpeakerNoteSpeaker R3_RightSpeakerNoteSpeaker;

    private final R2_R3_MiddleSpeakerNoteSpeaker R2_R3_MiddleSpeakerNoteSpeaker;
    private final B1_ShootAndLeave B1_ShootAndLeave;
    private final B2_B1_MiddleSpeakerNoteSpeaker B2_B1_MiddleSpeakerNoteSpeaker;

    private final SendableChooser<Command> autoCommandChooser = new SendableChooser<>();

    public InitializeAutoPaths(SwerveDrive swerve, Shooter shooter) {

        this.swerve = swerve;
        this.shooter = shooter;

        AutoBuilder.configureHolonomic(
        this.swerve::getPoseFromEstimator, // Robot pose supplier
        this.swerve::resetPose, // Method to reset odometry (will be called if your auto has a starting pose)
        this.swerve::getRobotRelativeSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
        this.swerve::driveRelative, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
        new HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should likely live in your Constants class
            new PIDConstants(1,0,0), // Translation PID constants -> path independent
            new PIDConstants(1,0,0), // Rotation PID constants -> more or less path dependent
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
          // if (alliance.isPresent()) {
          //   //SmartDashboard.putBoolean("Red Alliance?:", alliance.get() == DriverStation.Alliance.Red);
          //   return alliance.get() == DriverStation.Alliance.Red;
          // }
          return false;
        },
        this.swerve // Reference to this subsystem to set requirements
        );

        // PLAYS:
        B1_LeftShoot = new B1_LeftShoot(this.swerve, this.shooter);
        B1_LeftSpeakerNote = new B1_LeftSpeakerNote(this.swerve, this.shooter);
        B1_LeftSpeakerNoteSpeaker = new B1_LeftSpeakerNoteSpeaker(this.swerve, this.shooter);
        B1_MiddleShoot = new B1_MiddleShoot(this.swerve, this.shooter);
        B1_MiddleSpeakerNote = new B1_MiddleSpeakerNote(this.swerve, this.shooter);
        B1_MiddleSpeakerNoteSpeaker = new B1_MiddleSpeakerNoteSpeaker(this.swerve, this.shooter);
        B1_RightShoot = new B1_RightShoot(this.swerve, this.shooter);
        B1_RightSpeakerNote = new B1_RightSpeakerNote(this.swerve, this.shooter);
        B1_RightSpeakerNoteSpeaker = new B1_RightSpeakerNoteSpeaker(this.swerve, this.shooter);
        B2_LeftShoot = new B2_LeftShoot(this.swerve, this.shooter);
        B2_LeftSpeakerNote = new B2_LeftSpeakerNote(this.swerve, this.shooter);
        B2_LeftSpeakerNoteSpeaker = new B2_LeftSpeakerNoteSpeaker(this.swerve, this.shooter);
        B2_MiddleShoot = new B2_MiddleShoot(this.swerve, this.shooter);
        B2_MiddleSpeakerNote = new B2_MiddleSpeakerNote(this.swerve, this.shooter);
        B2_MiddleSpeakerNoteSpeaker = new B2_MiddleSpeakerNoteSpeaker(this.swerve, this.shooter);
        B2_RightShoot = new B2_RightShoot(this.swerve, this.shooter);
        B2_RightSpeakerNote = new B2_RightSpeakerNote(this.swerve, this.shooter);
        B2_RightSpeakerNoteSpeaker = new B2_RightSpeakerNoteSpeaker(this.swerve, this.shooter);
        B3_LeftShoot = new B3_LeftShoot(this.swerve, this.shooter);
        B3_LeftSpeakerNote = new B3_LeftSpeakerNote(this.swerve, this.shooter);
        B3_LeftSpeakerNoteSpeaker = new B3_LeftSpeakerNoteSpeaker(this.swerve, this.shooter);
        B3_MiddleShoot = new B3_MiddleShoot(this.swerve, this.shooter);
        B3_MiddleSpeakerNote = new B3_MiddleSpeakerNote(this.swerve, this.shooter);
        B3_MiddleSpeakerNoteSpeaker = new B3_MiddleSpeakerNoteSpeaker(this.swerve, this.shooter);
        B3_RightShoot = new B3_RightShoot(this.swerve, this.shooter);
        B3_RightSpeakerNote = new B3_RightSpeakerNote(this.swerve, this.shooter);
        B3_RightSpeakerNoteSpeaker = new B3_RightSpeakerNoteSpeaker(this.swerve, this.shooter);
        B1_ShootAndLeave = new B1_ShootAndLeave(this.swerve, this.shooter);
        B2_B1_MiddleSpeakerNoteSpeaker = new B2_B1_MiddleSpeakerNoteSpeaker(this.swerve, this.shooter);
        //Red Plays
        R1_LeftSpeakerNoteSpeaker = new R1_LeftSpeakerNoteSpeaker(this.swerve, this.shooter);
        R2_MiddleSpeakerNoteSpeaker = new R2_MiddleSpeakerNoteSpeaker(this.swerve, this.shooter);
        R3_RightSpeakerNoteSpeaker = new R3_RightSpeakerNoteSpeaker(this.swerve, this.shooter);
        R2_R3_MiddleSpeakerNoteSpeaker = new R2_R3_MiddleSpeakerNoteSpeaker(this.swerve, this.shooter);

        // Autonomous command selector
        autoCommandChooser.addOption("B1_LeftShoot", B1_LeftShoot);
        autoCommandChooser.addOption("B1_LeftSpeakerNote", B1_LeftSpeakerNote);
        autoCommandChooser.addOption("B1_LeftSpeakerNoteSpeaker", B1_LeftSpeakerNoteSpeaker);
        autoCommandChooser.addOption("B1_MiddleShoot", B1_MiddleShoot);
        autoCommandChooser.addOption("B1_MiddleSpeakerNote", B1_MiddleSpeakerNote);
        autoCommandChooser.addOption("B1_MiddleSpeakerNoteSpeaker", B1_MiddleSpeakerNoteSpeaker);
        autoCommandChooser.addOption("B1_RightShoot", B1_RightShoot);
        autoCommandChooser.addOption("B1_RightSpeakerNote", B1_RightSpeakerNote);
        autoCommandChooser.addOption("B1_RightSpeakerNoteSpeaker", B1_RightSpeakerNoteSpeaker);
        autoCommandChooser.addOption("B1_ShootAndLeave", B1_ShootAndLeave);
        
        autoCommandChooser.addOption("B2_LeftShoot", B2_LeftShoot);
        autoCommandChooser.addOption("B2_LeftSpeakerNote", B2_LeftSpeakerNote);
        autoCommandChooser.addOption("B2_LeftSpeakerNoteSpeaker", B2_LeftSpeakerNoteSpeaker);
        autoCommandChooser.addOption("B2_MiddleShoot", B2_MiddleShoot);
        autoCommandChooser.addOption("B2_MiddleSpeakerNote", B2_MiddleSpeakerNote);
        autoCommandChooser.addOption("B2_MiddleSpeakerNoteSpeaker", B2_MiddleSpeakerNoteSpeaker);
        autoCommandChooser.addOption("B2_RightShoot", B2_RightShoot);
        autoCommandChooser.addOption("B2_RightSpeakerNote", B2_RightSpeakerNote);
        autoCommandChooser.addOption("B2_RightSpeakerNoteSpeaker", B2_RightSpeakerNoteSpeaker);

        autoCommandChooser.addOption("B3_LeftShoot", B3_LeftShoot);
        autoCommandChooser.addOption("B3_LeftSpeakerNote", B3_LeftSpeakerNote);
        autoCommandChooser.addOption("B3_LeftSpeakerNoteSpeaker", B3_LeftSpeakerNoteSpeaker);
        autoCommandChooser.addOption("B3_MiddleShoot", B3_MiddleShoot);
        autoCommandChooser.addOption("B3_MiddleSpeakerNote", B3_MiddleSpeakerNote);
        autoCommandChooser.addOption("B3_MiddleSpeakerNoteSpeaker", B3_MiddleSpeakerNoteSpeaker);
        autoCommandChooser.addOption("B3_RightShoot", B3_RightShoot);
        autoCommandChooser.addOption("B3_RightSpeakerNote", B3_RightSpeakerNote);
        autoCommandChooser.addOption("B3_RightSpeakerNoteSpeaker", B3_RightSpeakerNoteSpeaker);
        autoCommandChooser.addOption("B2_B1_MiddleSpeakerNoteSpeaker", B2_B1_MiddleSpeakerNoteSpeaker);

        autoCommandChooser.addOption("R1_LeftSpeakerNoteSpeaker", R1_LeftSpeakerNoteSpeaker);
        autoCommandChooser.addOption("R2_MiddleSpeakerNoteSpeaker", R2_MiddleSpeakerNoteSpeaker);
        autoCommandChooser.addOption("R3_RightSpeakerNoteSpeaker", R3_RightSpeakerNoteSpeaker);
        autoCommandChooser.addOption("R2_R3_middleSpeakerNoteSpeaker", R2_R3_MiddleSpeakerNoteSpeaker);

        SmartDashboard.putData(autoCommandChooser);

    }

    public Command getAutonomousCommand() {
        return autoCommandChooser.getSelected();
    }
}
