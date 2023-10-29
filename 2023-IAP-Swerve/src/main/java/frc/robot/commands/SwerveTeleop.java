package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
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
   private DoubleSupplier translationSup;
   private DoubleSupplier strafeSup;
   private DoubleSupplier rotationSup;
   private BooleanSupplier robotCentricSup;

   // Slew rate limit controls
   private SlewRateLimiter translationLimiter = new SlewRateLimiter(10.0D);
   private SlewRateLimiter strafeLimiter = new SlewRateLimiter(10.0D);
   private SlewRateLimiter rotationLimiter = new SlewRateLimiter(1.0D);

   /**
    * Creates a SwerveTeleop command, for controlling a Swerve bot.
    * @param swerve - the Swerve subsystem
    * @param translationSup - the translational/x component of velocity
    * @param strafeSup - the strafe/y component of velocity
    * @param rotationSup - the rotational velocity of the chassis
    * @param robotCentricSup - whether to drive as robot centric or not
    */
   public SwerveTeleop(SwerveDrive swerve, DoubleSupplier translationSup, DoubleSupplier strafeSup, DoubleSupplier rotationSup, BooleanSupplier robotCentricSup) {
      this.swerve = swerve;
      this.translationSup = translationSup;
      this.strafeSup = strafeSup;
      this.rotationSup = rotationSup;
      this.robotCentricSup = robotCentricSup;
      this.addRequirements(swerve);
   }

   @Override
   public void execute() {

      // Get values after deadband and rate limiting
      double translationVal = this.translationLimiter.calculate(MathUtil.applyDeadband(this.translationSup.getAsDouble(), Constants.SwerveConstants.deadBand));
      double strafeVal = this.strafeLimiter.calculate(MathUtil.applyDeadband(this.strafeSup.getAsDouble(), Constants.SwerveConstants.deadBand));

      // Support for simulation WASD or real Xbox
      translationVal *= -1.0;
      
      double rotationVal = this.rotationLimiter.calculate(MathUtil.applyDeadband(this.rotationSup.getAsDouble(), Constants.SwerveConstants.deadBand));

      // Drive swerve with values
      this.swerve.drive((new Translation2d(translationVal, strafeVal)).times(Constants.SwerveConstants.maxChassisTranslationalSpeed), rotationVal * Constants.SwerveConstants.maxChassisAngularVelocity, this.robotCentricSup.getAsBoolean(), false);
   }

   // Called once the command ends or is interrupted.
   @Override
   public void end(boolean interrupted) {
      this.swerve.drive(new Translation2d(0, 0), 0, true, false);
   }
}