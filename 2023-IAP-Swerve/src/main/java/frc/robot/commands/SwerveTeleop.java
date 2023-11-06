package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
// TODO: Assymetric limiter!
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.swerve.SwerveDrive;
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
   private SlewRateLimiter translationLimiter = new SlewRateLimiter(10.0D);
   private SlewRateLimiter strafeLimiter = new SlewRateLimiter(10.0D);
   private SlewRateLimiter rotationLimiter = new SlewRateLimiter(10.0D);

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

      // Get values after deadband and rate limiting
      double xVal = this.translationLimiter.calculate(MathUtil.applyDeadband(this.x.getAsDouble(), Constants.SwerveConstants.deadBand));
      double yVal = this.strafeLimiter.calculate(MathUtil.applyDeadband(this.y.getAsDouble(), Constants.SwerveConstants.deadBand));

      // Support for simulation WASD or real Xbox
      xVal *= -1.0;
      
      double rotationVal = this.rotationLimiter.calculate(MathUtil.applyDeadband(this.rotationSup.getAsDouble(), Constants.SwerveConstants.deadBand));

      double angleOfVelocity = Math.atan2(yVal, xVal);


      // When driving, drive so that the magnitude of motion is scaled to a certain number
      double magnitude = Constants.SwerveConstants.maxChassisTranslationalSpeed;
      double correctedStrafe = 0.0;
      double correctedTranslate = 0.0;

      // Bugs out at 0.0
      if (yVal != 0 | xVal != 0) {
         correctedStrafe =  magnitude * Math.sin(angleOfVelocity);
         correctedTranslate = magnitude* Math.cos(angleOfVelocity);
      }

      // Drive swerve with values
      this.swerve.drive(new Translation2d(correctedTranslate, correctedStrafe),
      rotationVal * Constants.SwerveConstants.maxChassisAngularVelocity, 
      this.robotCentricSup.getAsBoolean(), false);
   }

   // Called once the command ends or is interrupted.
   @Override
   public void end(boolean interrupted) {
      this.swerve.drive(new Translation2d(0, 0), 0, true, false);
   }
}