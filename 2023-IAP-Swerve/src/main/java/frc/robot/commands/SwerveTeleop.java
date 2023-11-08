package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.swerve.SwerveDrive;
import frc.util.lib.AsymmetricLimiter;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

public class SwerveTeleop extends CommandBase {
   // Initialize empty swerve object
   private SwerveDrive swerve;

   // Create suppliers as object references
   private DoubleSupplier x;
   private DoubleSupplier y;
   private DoubleSupplier rotationSup;
   private BooleanSupplier robotCentricSup;

   // Slew rate limit controls
   // Positive limit ensures smooth acceleration (3 * dt * dControl)
   // Negative limit ensures an ability to stop (100 * dt * dControl)
   private AsymmetricLimiter translationLimiter = new AsymmetricLimiter(3.0D, 1000.0D);
   private AsymmetricLimiter rotationLimiter = new AsymmetricLimiter(10.0D, 1000.0D);

   /**
    * Creates a SwerveTeleop command, for controlling a Swerve bot.
    * @param swerve - the Swerve subsystem
    * @param x - the translational/x component of velocity
    * @param y - the strafe/y component of velocity
    * @param rotationSup - the rotational velocity of the chassis
    * @param robotCentricSup - whether to drive as robot centric or not
    */
   public SwerveTeleop(SwerveDrive swerve, DoubleSupplier x, DoubleSupplier y, DoubleSupplier rotationSup, BooleanSupplier robotCentricSup) {
      this.swerve = swerve;
      this.x = x;
      this.y = y;
      this.rotationSup = rotationSup;
      this.robotCentricSup = robotCentricSup;
      this.addRequirements(swerve);
   }

   @Override
   public void execute() {

      // Get values of controls and apply deadband
      double xVal = -this.x.getAsDouble(); // Flip for XBox support
      xVal = MathUtil.applyDeadband(xVal, Constants.SwerveConstants.deadBand);
      

      double yVal = this.y.getAsDouble();
      yVal = MathUtil.applyDeadband(yVal, Constants.SwerveConstants.deadBand);

      double rotationVal = this.rotationSup.getAsDouble();

      // Take the max value to rate limit - max value determines control!
      double limit = Math.max(Math.abs(xVal), Math.abs(yVal));
      limit = translationLimiter.calculate(limit);
      
      // Get Rotation2d/angle of x and y
      Rotation2d angleOfVelocity = new Rotation2d(xVal, yVal);
      
      rotationVal = MathUtil.applyDeadband(rotationVal, Constants.SwerveConstants.deadBand);

      rotationVal = this.rotationLimiter.calculate(rotationVal);

      // Hypotenuse SHOULD NOT be included in slew rate limit (NOMAD had hypotenuse which is dangerous, results in 1.41 factor)
      // Instead, take max slew rate limit (this is the best solution for 2D)
      double maxMagnitude = Constants.SwerveConstants.maxChassisTranslationalSpeed;

      double correctedX = 0.0;
      double correctedY = 0.0;

      // Bugs out at 0.0
      if (yVal != 0 | xVal != 0) {
         // Multiply magnitude by sin and cos for y and x
         // FIX TO MAGNITUDE BASED APPROX: Multiply magnitude's component by max slew rate limit
         correctedX =  limit * maxMagnitude * angleOfVelocity.getCos();
         correctedY = limit * maxMagnitude * angleOfVelocity.getSin();
      }

      // Drive swerve with values
      this.swerve.drive(new Translation2d(correctedX, correctedY),
      rotationVal * Constants.SwerveConstants.maxChassisAngularVelocity, 
      this.robotCentricSup.getAsBoolean(), false);
   }

   // Called once the command ends or is interrupted.
   @Override
   public void end(boolean interrupted) {
      this.swerve.drive(new Translation2d(0, 0), 0, true, false);
   }
}