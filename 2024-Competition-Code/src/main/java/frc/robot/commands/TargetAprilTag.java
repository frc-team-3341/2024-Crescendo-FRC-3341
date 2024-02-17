package frc.robot.commands;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.PhotonVision.*;
import frc.robot.subsystems.swerve.SwerveDrive;


public class TargetAprilTag extends Command {
    private photonvision photonVision;
    private SwerveDrive swerveDrive;
    public Joystick controller;

    public double[] threshold = {0.2,0.05};
    // {rotation threshold (Degrees), centering threshold (meters)}
    public double ZAngle;
    public double XVal;
    public int moveDirection = 1;
    //Positive means move left, negative means move right

    public int rotationDirection = 1;
    // Positive means clockwise, negative means counter-clockwise
    public boolean rotationAligned = false;
    public boolean centered = false;

    public TargetAprilTag(photonvision photonVision, SwerveDrive swerveDrive, Joystick joy) {
        this.photonVision = photonVision;
        this.swerveDrive = swerveDrive;
        this.controller = joy;

        // each subsystem used by the command must be passed into the
        // addRequirements() method (which takes a vararg of Subsystem)
        addRequirements(this.photonVision, this.swerveDrive);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        //TESTING Procedure #1 Test to see if this code works and infinitely moves the robot to the left
        //TESTING Procedure: Test the centering and rotation independently, then test them together, then add swerve manual control
        if (photonVision.targetExists() && controller.getRawButtonPressed(XboxController.Button.kLeftBumper.value)){

//            while (!rotationAligned){
//                ZAngle = photonVision.getZAngle();
//                if (ZAngle > 0){
//                    rotationDirection = -1;
//                }else{
//                    moveDirection = 1;
//                }
//                //Swerve turn (move right if val is > 180 and to the left if val is < 180) [OLD] ??
//                swerveDrive.drive(new Translation2d(0,0), 5, false, false);
//                if ( (-threshold[0] <= 0) && (0 <= threshold[0]) ){
//                    rotationAligned = true;
//                    swerveDrive.drive(new Translation2d(0,0), 0, false, false);
//                    break;
//                }
//            }

            while (!centered){
                XVal = photonVision.getXOffset();
                if (XVal > 0){
                    moveDirection = -1;
                }else{
                    moveDirection = 1;
                }
                //positive y is left, negative y is right
                swerveDrive.drive(new Translation2d(0, 0.1 * moveDirection), 0, false, false);
                //Move toward center (left or right)
                if ( (-threshold[1] <= XVal) && (XVal <= threshold[1]) ){
                    centered = true;
                    swerveDrive.drive(new Translation2d(0, 0), 0, false, false);
                    break;
                }
            }

        }
    }

    @Override
    public boolean isFinished() {
        // TODO: Make this return true when this Command no longer needs to run execute()
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        swerveDrive.drive(new Translation2d(0,0), 0, false, false);
    }
}
