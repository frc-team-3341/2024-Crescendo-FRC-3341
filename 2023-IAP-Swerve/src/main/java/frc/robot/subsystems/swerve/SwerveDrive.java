package frc.robot.subsystems.swerve;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.SPI.Port;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.util.lib.SwerveUtil;

public class SwerveDrive extends SubsystemBase {
   // Create Navx
   private static AHRS navx = new AHRS(Port.kMXP);

   // Create object representing swerve modules
   private SwerveModuleIO[] moduleIO;

   // Create object that represents swerve module positions (i.e. radians and
   // meters)
   private SwerveModulePosition[] modulePositions = new SwerveModulePosition[4];

   // Create kinematics object
   private SwerveDriveKinematics kinematics;

   // Create poseEstimator object
   private SwerveDrivePoseEstimator poseEstimator;

   // Create swerveOdometry object
   private SwerveDriveOdometry swerveOdometry;

   // Add field to show robot
   private Field2d field;

   /**
    * Creates a new SwerveDrive object. Intended to work both with real modules and
    * simulation.
    * 
    * @param FL Swerve module
    * @param FR Swerve module
    * @param BL Swerve module
    * @param BR Swerve module
    *
    * @author Aric Volman
    */
   public SwerveDrive(SwerveModuleIO FL, SwerveModuleIO FR, SwerveModuleIO BL, SwerveModuleIO BR) {
      // Assign modules to their object
      this.moduleIO = new SwerveModuleIO[] { FL, FR, BL, BR };

      // Iterate through module positions and assign initial values
      modulePositions = SwerveUtil.setModulePositions(moduleIO);

      // Initialize all other objects
      this.kinematics = new SwerveDriveKinematics(SwerveUtil.getModuleTranslations());
      this.poseEstimator = new SwerveDrivePoseEstimator(this.kinematics, new Rotation2d(), this.modulePositions, new Pose2d());
      this.swerveOdometry = new SwerveDriveOdometry(this.kinematics, this.getRotation(), this.modulePositions);
      this.field = new Field2d();
   }

   public void periodic() {
      // Update module positions
      modulePositions = SwerveUtil.setModulePositions(moduleIO);

      // Update odometry, field, and poseEstimator
      this.swerveOdometry.update(this.getRotation(), this.modulePositions);
      this.poseEstimator.update(this.getRotation(), this.modulePositions);
      this.field.setRobotPose(this.getPose());

      // Draw poses of robot's modules in SmartDashboard
      SwerveUtil.drawModulePoses(modulePositions, field, getPose());

      // Put field on SmartDashboard
      SmartDashboard.putData("Field", this.field);
      SmartDashboard.putNumberArray("States", SwerveUtil.getDoubleStates(getActualStates()));
      SmartDashboard.putNumber("Robot Rotation", getPose().getRotation().getRadians());
   }

   public void simulationPeriodic() {
      // Add simulation! Yes, with the Util class, it's that easy!
      SwerveUtil.addSwerveSimulation(moduleIO, getActualStates(), kinematics);
   }

   /**
    * Drive either field oriented, or not field oriented
    * 
    * @param translation   Vector of x-y velocity in m/s
    * @param rotation      Rotation psuedovector in rad/s
    * @param fieldRelative Whether or not the robot should drive field relative
    * @param isOpenLoop    Whether or not to control robot with closed or open loop
    *                      control
    */
   public void drive(Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop) {

      ChassisSpeeds speeds = fieldRelative
            ? ChassisSpeeds.fromFieldRelativeSpeeds(translation.getX(), translation.getY(), rotation,
                  this.getRotation())
            : new ChassisSpeeds(translation.getX(), translation.getY(), rotation);

      speeds = SwerveUtil.discretize(speeds, -5.0);

      SmartDashboard.putNumber("Setpoint Magnitude Vel",
            Math.sqrt(Math.pow(speeds.vxMetersPerSecond, 2) + Math.pow(speeds.vyMetersPerSecond, 2)));

      SwerveModuleState[] swerveModuleStates = this.kinematics.toSwerveModuleStates(speeds);

      // MUST USE SECOND TYPE OF METHOD
      SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, speeds,
            Constants.SwerveConstants.maxWheelLinearVelocityMeters, Constants.SwerveConstants.maxChassisTranslationalSpeed,
            Constants.SwerveConstants.maxChassisAngularVelocity);

      for (int i = 0; i < swerveModuleStates.length; i++) {
         this.moduleIO[i].setDesiredState(swerveModuleStates[i]);
      }

   }

   /**
    * Drive the robot for PathPlannerLib
    */
   public void driveRelative(ChassisSpeeds speeds) {
      speeds = SwerveUtil.discretize(speeds, -5.0);

      SmartDashboard.putNumber("MagnitudeVel",
            Math.sqrt(Math.pow(speeds.vxMetersPerSecond, 2) + Math.pow(speeds.vyMetersPerSecond, 2)));

      SwerveModuleState[] swerveModuleStates = this.kinematics.toSwerveModuleStates(speeds);

      // MUST USE SECOND TYPE OF METHOD
      SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, speeds, Constants.SwerveConstants.maxWheelLinearVelocityMeters,
            Constants.SwerveConstants.maxChassisTranslationalSpeed,
            Constants.SwerveConstants.maxChassisAngularVelocity);

      for (int i = 0; i < swerveModuleStates.length; i++) {
         this.moduleIO[i].setDesiredState(swerveModuleStates[i]);
      }
   }

   /**
    * Gets the SwerveModuleState[] for our use in code.
    */
   public SwerveModuleState[] getSetpointStates() {
      SwerveModuleState[] states = new SwerveModuleState[moduleIO.length];

      for (int i = 0; i < states.length; i++) {
         states[i] = this.moduleIO[i].getSetpointModuleState();
      }

      return states;
   }

   /**
    * Gets the actual SwerveModuleState[] for our use in code
    */
   public SwerveModuleState[] getActualStates() {
      SwerveModuleState[] states = new SwerveModuleState[moduleIO.length];

      for (int i = 0; i < states.length; i++) {
         states[i] = this.moduleIO[i].getActualModuleState();
      }

      return states;
   }

   public void stopMotors() {
      for (int i = 0; i < 4; i++) {
         moduleIO[i].setDriveVoltage(0);
         moduleIO[i].setTurnVoltage(0);
      }
   }

   /**
   * Get heading of Navx. Negative because Navx is CW positive.
   */
    public double getHeading() {
      return -navx.getRotation2d().getDegrees();
   }

   /**
    * Get rate of rotation of Navx. Negative because Navx is CW positive.
    */
   public double getTurnRate() {
      return -navx.getRate();
   }

   /**
    * Get Rotation2d of Navx. Positive value (CCW positive default).
    */
   public Rotation2d getRotation() {
      return navx.getRotation2d();
   }

   /**
    * Get Pose2d of poseEstimator.
    */
   public Pose2d getPose() {
      return poseEstimator.getEstimatedPosition();
   }

   /**
    * Reset pose of robot to pose
    */
   public void resetPose(Pose2d pose) {
      poseEstimator.resetPosition(new Rotation2d(), modulePositions, pose);
   }

   /**
    * Get chassis speeds for PathPlannerLib
    */
   public ChassisSpeeds getRobotRelativeSpeeds() {
      return ChassisSpeeds.fromFieldRelativeSpeeds(kinematics.toChassisSpeeds(getActualStates()), getRotation());
   }

   /** Gets field */
   public Field2d getField() {
      return field;
   }

}