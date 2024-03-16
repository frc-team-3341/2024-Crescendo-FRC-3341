// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.util.PIDConstants;

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

import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.swerve.SwerveDrive;

/** Add your docs here. */
public class InitializeAutoPaths {
    private final SwerveDrive swerve;
    private final Shooter shooter;
    private final AutoPath B1_LeftSpeakerToNote, B1_MiddleSpeakerToNote, B1_NoteToSpeaker, B1_RightSpeakerToNote;
    private final AutoPath B2_LeftSpeakerToNote, B2_MiddleSpeakerToNote, B2_NoteToSpeaker, B2_RightSpeakerToNote;
    private final AutoPath B3_LeftSpeakerToNote, B3_MiddleSpeakerToNote, B3_NoteToSpeaker, B3_RightSpeakerToNote;


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

    private final SendableChooser<Command> autoCommandChooser = new SendableChooser<>();

    public InitializeAutoPaths(SwerveDrive swerve, Shooter shooter) {
        this.swerve = swerve;
        this.shooter = shooter;
        
        B1_LeftSpeakerToNote = new AutoPath("B1 Left Speaker to Note", this.swerve, new PIDConstants(1.0, 0, 0),
                new PIDConstants(1.0, 0, 0), true);
        B1_MiddleSpeakerToNote = new AutoPath("B1 Middle Speaker to Note", this.swerve, new PIDConstants(1.0, 0, 0),
                new PIDConstants(1.0, 0, 0), true);
        B1_NoteToSpeaker = new AutoPath("B1 Note to Speaker", this.swerve, new PIDConstants(1.0, 0, 0),
                new PIDConstants(1.0, 0, 0), false);
        B1_RightSpeakerToNote = new AutoPath("B1 Right Speaker to Note", this.swerve, new PIDConstants(1.0, 0, 0),
                new PIDConstants(1.0, 0, 0), true);

        B2_LeftSpeakerToNote = new AutoPath("B2 Left Speaker to Note", this.swerve, new PIDConstants(1.0, 0, 0),
                new PIDConstants(1.0, 0, 0), true);
        B2_MiddleSpeakerToNote = new AutoPath("B2 Middle Speaker to Note", this.swerve, new PIDConstants(1.0, 0, 0),
                new PIDConstants(1.0, 0, 0), true);
        B2_NoteToSpeaker = new AutoPath("B2 Note to Speaker", this.swerve, new PIDConstants(1.0, 0, 0),
                new PIDConstants(1.0, 0, 0), false);
        B2_RightSpeakerToNote = new AutoPath("B2 Right Speaker to Note", this.swerve, new PIDConstants(1.0, 0, 0),
                new PIDConstants(1.0, 0, 0), true);

        B3_LeftSpeakerToNote = new AutoPath("B3 Left Speaker to Note", this.swerve, new PIDConstants(1.0, 0, 0),
                new PIDConstants(1.0, 0, 0), true);
        B3_MiddleSpeakerToNote = new AutoPath("B3 Middle Speaker to Note", this.swerve, new PIDConstants(1.0, 0, 0),
                new PIDConstants(1.0, 0, 0), true);
        B3_NoteToSpeaker = new AutoPath("B3 Note to Speaker", this.swerve, new PIDConstants(1.0, 0, 0),
                new PIDConstants(1.0, 0, 0), false);
        B3_RightSpeakerToNote = new AutoPath("B3 Right Speaker to Note", this.swerve, new PIDConstants(1.0, 0, 0),
                new PIDConstants(1.0, 0, 0), true);

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

        


        SmartDashboard.putData(autoCommandChooser);

    }

    public Command getAutonomousCommand() {
        return autoCommandChooser.getSelected();
    }
}
