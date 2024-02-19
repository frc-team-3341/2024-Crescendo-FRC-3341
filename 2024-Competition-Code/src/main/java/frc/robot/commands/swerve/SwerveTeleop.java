package frc.robot.commands.swerve;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.subsystems.swerve.SwerveDrive;
import frc.util.lib.AsymmetricLimiter;
import frc.util.lib.ArcadeJoystickUtil;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

public class SwerveTeleop extends Command {
   // Initialize empty swerve object
   private SwerveDrive swerve;

   // Create suppliers as object references
   private DoubleSupplier inputX;
   private DoubleSupplier inputY;
   private DoubleSupplier x;
   private DoubleSupplier y;
   private DoubleSupplier rotationSup;
   private BooleanSupplier robotCentricSup;
   private DoubleSupplier translationLeftTrigger;

   public boolean setAlliance;

   private double xMult = 1.0;
   private double yMult = 1.0;

   private ArcadeJoystickUtil joyUtil;

   // Slew rate limit controls
   // Positive limit ensures smooth acceleration (1000 * dt * dControl)
   // Negative limit ensures an ability to stop (0 * dt * dControl)
   private AsymmetricLimiter translationLimiter = new AsymmetricLimiter(5.0D, 1000.0D);
   private AsymmetricLimiter rotationLimiter = new AsymmetricLimiter(10.0D, 10.0D);

   /**
    * Creates a SwerveTeleop command, for controlling a Swerve bot.
    * 
    * @param swerve          - the Swerve subsystem
    * @param x               - the translational/x component of velocity (across field)
    * @param y               - the strafe/y component of velocity (up and down on field)
    * @param rotationSup     - the rotational velocity of the chassis
    * @param robotCentricSup - whether to drive as robot centric or not
    */
   public SwerveTeleop(SwerveDrive swerve, DoubleSupplier x, DoubleSupplier y, DoubleSupplier rotationSup, DoubleSupplier translationLeftTrigger,
         BooleanSupplier robotCentricSup, boolean setAlliance, boolean blueAllianceOrNot) {
      this.swerve = swerve;
      // If doesn't want to set alliance

      this.setAlliance = setAlliance;
      this.inputX = x;
      this.inputY = y;
      this.rotationSup = rotationSup;
      this.robotCentricSup = robotCentricSup;
      this.translationLeftTrigger = translationLeftTrigger;
      this.joyUtil = new ArcadeJoystickUtil();
      this.addRequirements(swerve);
   }

   @Override
   public void execute() {

      if (setAlliance) {

         var alliance = Robot.getAlliance();

         if (alliance.isPresent()) {
            // If blue alliance
            if (alliance.get() == DriverStation.Alliance.Red) {
               yMult = -1.0;
               xMult = -1.0;
            } else {
               yMult = 1.0;
               xMult = 1.0;
            }
         }
      }

      this.x = inputX;
      this.y = inputY;

      // Get values of controls and apply deadband
      double xVal = this.x.getAsDouble(); // Flip for XBox support
      double yVal = this.y.getAsDouble();

      // Could consider subtracting this from 1
      double leftTriggerVal = this.translationLeftTrigger.getAsDouble();

      if (leftTriggerVal < 0.1) {
         leftTriggerVal = 0.1;
      }

      xVal = MathUtil.applyDeadband(xVal, Constants.SwerveConstants.deadBand);
      yVal = MathUtil.applyDeadband(yVal, Constants.SwerveConstants.deadBand);

      double rotationVal = this.rotationSup.getAsDouble();
      rotationVal = MathUtil.applyDeadband(rotationVal, Constants.SwerveConstants.deadBand);

      // Apply rate limiting to rotation
      rotationVal = this.rotationLimiter.calculate(rotationVal);

      double[] output = new double[2];
      if (RobotContainer.isXbox) {
         output = joyUtil.regularGamePadControls(xVal, yVal, 
         Constants.SwerveConstants.maxChassisTranslationalSpeed);
      } else {
         // Function to map joystick output to scaled polar coordinates
         output = joyUtil.convertXYToScaledPolar(xVal, yVal,
         Constants.SwerveConstants.maxChassisTranslationalSpeed);
      }

      double newHypot = translationLimiter.calculate(output[0]);

      // Deadband should be applied after calculation of polar coordinates
      newHypot = MathUtil.applyDeadband(newHypot, Constants.SwerveConstants.deadBand);

      double correctedX = leftTriggerVal * xMult * newHypot * Math.cos(output[1]);
      double correctedY =  leftTriggerVal * yMult * newHypot * Math.sin(output[1]);

      // Drive swerve with values
      this.swerve.drive(new Translation2d(correctedX, correctedY),
            rotationVal * Constants.SwerveConstants.maxChassisAngularVelocity,
            this.robotCentricSup.getAsBoolean(), false);
   }

   // Called once the command ends or is interrupted.
   @Override
   public void end(boolean interrupted) {
      
      this.swerve.drive(new Translation2d(0, 0), 0, true, false);

      // PLEASE SET THIS FOR SAFETY!!!
      this.swerve.stopMotors();
   }
}