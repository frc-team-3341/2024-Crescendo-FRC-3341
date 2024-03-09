// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.util.PIDConstants;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.auto.AutoPath;
import frc.robot.commands.auto.BlueAlliance1.Plays.B1_StartAmpNoteSpeaker;
import frc.robot.commands.auto.BlueAlliance1.Plays.B1_StartSpeakerNoteSpeaker;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.swerve.SwerveDrive;

/** Add your docs here. */
public class InitializeAutoPaths {
    private final SwerveDrive swerve;
    private final Shooter shooter;
    private final AutoPath B1_StartToAmp, B1_NoteToSpeaker, B1_SpeakerToNote, B1_NoteToAmp, B1_AmpToNote,
            B1_StartToSpeaker;
    private final AutoPath B2_StartToAmp, B2_NoteToSpeaker, B2_SpeakerToNote, B2_NoteToAmp, B2_AmpToNote,
            B2_StartToSpeaker;
    private final AutoPath B3_StartToAmp, B3_NoteToAmp, B3_NoteToSpeaker, B3_SpeakerToNote, B3_AmpToNote,
            B3_StartToSpeaker;

    private final AutoPath R1_StartToAmp, R1_NoteToSpeaker, R1_SpeakerToNote, R1_NoteToAmp, R1_AmpToNote,
            R1_StartToSpeaker;
    private final AutoPath R2_StartToAmp, R2_NoteToSpeaker, R2_SpeakerToNote, R2_NoteToAmp, R2_AmpToNote,
            R2_StartToSpeaker;
    private final AutoPath R3_StartToAmp, R3_NoteToSpeaker, R3_SpeakerToNote, R3_NoteToAmp, R3_AmpToNote,
            R3_StartToSpeaker;

    // Plays:
    private final B1_StartAmpNoteSpeaker B1_StartAmpNoteSpeaker;
    private final B1_StartSpeakerNoteSpeaker B1_StartSpeakerNoteSpeaker;

    private final SendableChooser<Command> autoCommandChooser = new SendableChooser<>();

    public InitializeAutoPaths(SwerveDrive swerve, Shooter shooter) {
        this.swerve = swerve;
        this.shooter = shooter;
        B1_StartToAmp = new AutoPath("B1 Start to Amp", this.swerve, new PIDConstants(2.0, 0, 0),
                new PIDConstants(2.0, 0, 0));
        B1_NoteToAmp = new AutoPath("B1 Note to Amp", this.swerve, new PIDConstants(1.0, 0, 0),
                new PIDConstants(1.0, 0, 0));
        B1_NoteToSpeaker = new AutoPath("B1 Note to Speaker", this.swerve, new PIDConstants(1.0, 0, 0),
                new PIDConstants(1.0, 0, 0));;
        B1_AmpToNote = new AutoPath("B1 Amp to Note", this.swerve, new PIDConstants(1.0, 0, 0),
                new PIDConstants(1.0, 0, 0));; // Has to be the same name as the path name in the paths folder
        B1_SpeakerToNote = new AutoPath("B1 Speaker to Note", this.swerve, new PIDConstants(1.0, 0, 0),
                new PIDConstants(1.0, 0, 0));;
        B1_StartToSpeaker = new AutoPath("B1 Start to Speaker", this.swerve, new PIDConstants(1.0, 0, 0),
                new PIDConstants(1.0, 0, 0));;

        B2_AmpToNote = new AutoPath("B2 Amp to Note", this.swerve, new PIDConstants(1.0, 0, 0),
                new PIDConstants(1.0, 0, 0));
        B2_NoteToAmp = new AutoPath("B2 Note to Amp", this.swerve, new PIDConstants(1.0, 0, 0),
                new PIDConstants(1.0, 0, 0));
        B2_StartToAmp = new AutoPath("B2 Start to Amp", this.swerve, new PIDConstants(1.0, 0, 0),
                new PIDConstants(1.0, 0, 0));
        B2_NoteToSpeaker = new AutoPath("B2 Note to Speaker", this.swerve, new PIDConstants(1.0, 0, 0),
                new PIDConstants(1.0, 0, 0));
        B2_SpeakerToNote = new AutoPath("B2 Speaker to Note", this.swerve, new PIDConstants(1.0, 0, 0),
                new PIDConstants(1.0, 0, 0));
        B2_StartToSpeaker = new AutoPath("B2 Start to Speaker", this.swerve, new PIDConstants(1.0, 0, 0),
                new PIDConstants(1.0, 0, 0));

        B3_AmpToNote = new AutoPath("B3 Amp to Note", this.swerve, new PIDConstants(1.0, 0, 0),
                new PIDConstants(1.0, 0, 0));
        B3_NoteToAmp = new AutoPath("B3 Note to Amp", this.swerve, new PIDConstants(1.0, 0, 0),
                new PIDConstants(1.0, 0, 0));
        B3_StartToAmp = new AutoPath("B3 Start to Amp", this.swerve, new PIDConstants(1.0, 0, 0),
                new PIDConstants(1.0, 0, 0));
        B3_NoteToSpeaker = new AutoPath("B3 Note to Speaker", this.swerve, new PIDConstants(1.0, 0, 0),
                new PIDConstants(1.0, 0, 0));
        B3_SpeakerToNote = new AutoPath("B3 Speaker to Note", this.swerve, new PIDConstants(1.0, 0, 0),
                new PIDConstants(1.0, 0, 0));
        B3_StartToSpeaker = new AutoPath("B3 Start to Speaker", this.swerve, new PIDConstants(1.0, 0, 0),
                new PIDConstants(1.0, 0, 0));
//manually add in the initial pose for the paths down from here:
        R1_AmpToNote = new AutoPath("R1 Amp to Note", this.swerve, new PIDConstants(1.0, 0, 0),
                new PIDConstants(1.0, 0, 0));
        R1_NoteToAmp = new AutoPath("R1 Note to Amp", this.swerve, new PIDConstants(1.0, 0, 0),
                new PIDConstants(1.0, 0, 0));
        R1_StartToAmp = new AutoPath("R1 Start to Amp", this.swerve, new PIDConstants(1.0, 0, 0),
                new PIDConstants(1.0, 0, 0));
        R1_NoteToSpeaker = new AutoPath("R1 Note to Speaker", this.swerve, new PIDConstants(1.0, 0, 0),
                new PIDConstants(1.0, 0, 0));
        R1_SpeakerToNote = new AutoPath("R1 Speaker to Note", this.swerve, new PIDConstants(1.0, 0, 0),
                new PIDConstants(1.0, 0, 0));
        R1_StartToSpeaker = new AutoPath("R1 Start to Speaker", this.swerve, new PIDConstants(1.0, 0, 0),
                new PIDConstants(1.0, 0, 0));

        R2_AmpToNote = new AutoPath("R2 Amp to Note", this.swerve, new PIDConstants(1.0, 0, 0),
                new PIDConstants(1.0, 0, 0));
        R2_NoteToAmp = new AutoPath("R2 Note to Amp", this.swerve, new PIDConstants(1.0, 0, 0),
                new PIDConstants(1.0, 0, 0));
        R2_StartToAmp = new AutoPath("R2 Start to Amp", this.swerve, new PIDConstants(1.0, 0, 0),
                new PIDConstants(1.0, 0, 0));
        R2_NoteToSpeaker = new AutoPath("R2 Note to Speaker", this.swerve, new PIDConstants(1.0, 0, 0),
                new PIDConstants(1.0, 0, 0));
        R2_SpeakerToNote = new AutoPath("R2 Speaker to Note", this.swerve, new PIDConstants(1.0, 0, 0),
                new PIDConstants(1.0, 0, 0));
        R2_StartToSpeaker = new AutoPath("R2 Start to Speaker", this.swerve, new PIDConstants(1.0, 0, 0),
                new PIDConstants(1.0, 0, 0));

        R3_AmpToNote = new AutoPath("R3 Amp to Note", this.swerve, new PIDConstants(1.0, 0, 0),
                new PIDConstants(1.0, 0, 0));
        R3_NoteToAmp = new AutoPath("R3 Note to Amp", this.swerve, new PIDConstants(1.0, 0, 0),
                new PIDConstants(1.0, 0, 0));
        R3_StartToAmp = new AutoPath("R3 Start to Amp", this.swerve, new PIDConstants(1.0, 0, 0),
                new PIDConstants(1.0, 0, 0));
        R3_NoteToSpeaker = new AutoPath("R3 Note to Speaker", this.swerve, new PIDConstants(1.0, 0, 0),
                new PIDConstants(1.0, 0, 0));
        R3_SpeakerToNote = new AutoPath("R3 Speaker to Note", this.swerve, new PIDConstants(1.0, 0, 0),
                new PIDConstants(1.0, 0, 0));
        R3_StartToSpeaker = new AutoPath("R3 Start to Speaker", this.swerve, new PIDConstants(1.0, 0, 0),
                new PIDConstants(1.0, 0, 0));

        // PLAYS:
        B1_StartAmpNoteSpeaker = new B1_StartAmpNoteSpeaker(this.swerve, this.shooter);
        B1_StartSpeakerNoteSpeaker = new B1_StartSpeakerNoteSpeaker(this.swerve, this.shooter);

        // Autonomous command selector
        autoCommandChooser.addOption("B1_StartToAmp", B1_StartToAmp);
        autoCommandChooser.addOption("B1_NoteToAmp", B1_NoteToAmp);
        autoCommandChooser.addOption("B1_NoteToSpeaker", B1_NoteToSpeaker);
        autoCommandChooser.addOption("B1_AmpToNote", B1_AmpToNote);
        autoCommandChooser.addOption("B1_SpeakerToNote", B1_SpeakerToNote);
        autoCommandChooser.addOption("B1_StartToSpeaker", B1_StartToSpeaker);

        autoCommandChooser.addOption("B2_AmpToNote", B2_AmpToNote);
        autoCommandChooser.addOption("B2_NoteToAmp", B2_NoteToAmp);
        autoCommandChooser.addOption("B2_StartToAmp", B2_StartToAmp);
        autoCommandChooser.addOption("B2_NoteToSpeaker", B2_NoteToSpeaker);
        autoCommandChooser.addOption("B2_SpeakerToNote", B2_SpeakerToNote);
        autoCommandChooser.addOption("B2_StartToSpeaker", B2_StartToSpeaker);

        autoCommandChooser.addOption("B3_AmpToNote", B3_AmpToNote);
        autoCommandChooser.addOption("B3_NoteToAmp", B3_NoteToAmp);
        autoCommandChooser.addOption("B3_StartToAmp", B3_StartToAmp);
        autoCommandChooser.addOption("B3_NoteToSpeaker", B3_NoteToSpeaker);
        autoCommandChooser.addOption("B3_SpeakerToNote", B3_SpeakerToNote);
        autoCommandChooser.addOption("B3_StartToSpeaker", B3_StartToSpeaker);

        autoCommandChooser.addOption("R1_AmpToNote", R1_AmpToNote);
        autoCommandChooser.addOption("R1_NoteToAmp", R1_NoteToAmp);
        autoCommandChooser.addOption("R1_StartToAmp", R1_StartToAmp);
        autoCommandChooser.addOption("R1_NoteToSpeaker", R1_NoteToSpeaker);
        autoCommandChooser.addOption("R1_SpeakerToNote", R1_SpeakerToNote);
        autoCommandChooser.addOption("R1_StartToSpeaker", R1_StartToSpeaker);

        autoCommandChooser.addOption("R2_AmpToNote", R2_AmpToNote);
        autoCommandChooser.addOption("R2_NoteToAmp", R2_NoteToAmp);
        autoCommandChooser.addOption("R2_StartToAmp", R2_StartToAmp);
        autoCommandChooser.addOption("R2_NoteToSpeaker", R2_NoteToSpeaker);
        autoCommandChooser.addOption("R2_SpeakerToNote", R2_SpeakerToNote);
        autoCommandChooser.addOption("R2_StartToSpeaker", R2_StartToSpeaker);

        autoCommandChooser.addOption("R3_AmpToNote", R3_AmpToNote);
        autoCommandChooser.addOption("R3_NoteToAmp", R3_NoteToAmp);
        autoCommandChooser.addOption("R3_StartToAmp", R3_StartToAmp);
        autoCommandChooser.addOption("R3_NoteToSpeaker", R3_NoteToSpeaker);
        autoCommandChooser.addOption("R3_SpeakerToNote", R3_SpeakerToNote);
        autoCommandChooser.addOption("R3_StartToSpeaker", R3_StartToSpeaker);

        autoCommandChooser.addOption("B1_StartAmpNoteSpeaker", B1_StartAmpNoteSpeaker);
        autoCommandChooser.addOption("B1_StartSpeakerNoteSpeaker", B1_StartSpeakerNoteSpeaker);

        autoCommandChooser.setDefaultOption("B1_StartToAmp", B1_StartToAmp);

        SmartDashboard.putData(autoCommandChooser);

    }

    public Command getAutonomousCommand() {
        return autoCommandChooser.getSelected();
    }
}
