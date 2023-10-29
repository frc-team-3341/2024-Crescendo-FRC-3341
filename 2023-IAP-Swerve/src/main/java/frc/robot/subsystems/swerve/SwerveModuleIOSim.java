package frc.robot.subsystems.swerve;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;

public class SwerveModuleIOSim implements SwerveModuleIO {

   // Create drive and turn sim motors
   private final DCMotorSim driveSim = new DCMotorSim(DCMotor.getNEO(1), Constants.SwerveConstants.driveGearRatio,
         0.01); // 0.01
   private final DCMotorSim turnSim = new DCMotorSim(DCMotor.getNEO(1), Constants.SwerveConstants.turnGearRatio,
         0.000001);

   // Create PID controllers with constants
   // Note lack of feedforward
   // Native unit: volt
   private final PIDController drivePID = new PIDController(12.0, 0.0, 0.06);
   private final PIDController turnPID = new PIDController(5.0, 0.0, 0.06);

   // Create variables to hold driving and turning voltage
   private double driveVolts = 0.0;
   private double turnVolts = 0.0;

   // Object to hold swerve module state
   private SwerveModuleState state = new SwerveModuleState();

   // Number of swerve module
   private int num = 0;

   /**
    * Creates a simulated Swerve module. No motor ports necessary - all simulation
    * does is uses objects.
    * 
    * @param num - Number, starting from 0 and ending at 3, of swerve module
    */
   public SwerveModuleIOSim(int num) {
      this.num = num;
      turnPID.enableContinuousInput(-Math.PI, Math.PI);
   }

   public void setDriveVoltage(double voltage) {
      this.driveVolts = MathUtil.clamp(voltage, -12.0, 12.0);
      this.driveSim.setInputVoltage(this.driveVolts);
   }

   public void setTurnVoltage(double voltage) {
      this.turnVolts = MathUtil.clamp(voltage, -12.0, 12.0);
      this.turnSim.setInputVoltage(this.turnVolts);
   }

   public void setDesiredState(SwerveModuleState state) {

      // Optimize state so that movement is minimized
      state = SwerveModuleState.optimize(state, new Rotation2d(this.turnSim.getAngularPositionRad()));

      // Set internal state as passed-in state
      this.state = state;

      double velocity;
      if (!(DriverStation.isDisabled() | DriverStation.isEStopped())) {
         // Set setpoint of the drive PID controller
         this.drivePID.setSetpoint(state.speedMetersPerSecond);

         // Find measurement in m/s and calculate PID action
         velocity = this.driveSim.getAngularVelocityRadPerSec() * Math.PI
               * Constants.SwerveConstants.wheelDiameterMeters / Constants.SwerveConstants.driveGearRatio;
         double output = this.drivePID.calculate(velocity);

         // Output in volts to motor
         driveVolts = output;
         this.driveSim.setInputVoltage(driveVolts);
         SmartDashboard.putNumber("Closed Loop Drive #" + this.num, driveVolts);
         SmartDashboard.putNumber("DriveVel #" + this.num, state.speedMetersPerSecond);
      } else {
         // Output in volts to motor
         driveVolts = 0.0;
         this.driveSim.setInputVoltage(0.0);
         SmartDashboard.putNumber("Closed Loop Drive #" + this.num, 0.0);
      }

      // Turn with PID in volts
      this.turnPID.setSetpoint(state.angle.getRadians());
      SmartDashboard.putNumber("Turning Angle #" + this.num,
            Units.radiansToDegrees(this.turnSim.getAngularPositionRad()));

      double turnOutput = this.turnPID.calculate(this.turnSim.getAngularPositionRad());
      this.turnSim.setInputVoltage(turnOutput);
      turnVolts = turnOutput;
   }

   public SwerveModulePosition getPosition() {
      // Get position of swerve module in meters and radians
      SwerveModulePosition position = new SwerveModulePosition(this.driveSim.getAngularPositionRad() * Math.PI
            * Constants.SwerveConstants.wheelDiameterMeters / Constants.SwerveConstants.driveGearRatio,
            new Rotation2d(this.turnSim.getAngularPositionRad()));
      SmartDashboard.putNumber("Wheel Displacement #" + this.num, position.distanceMeters);
      return position;
   }

   public SwerveModuleState getSetpointModuleState() {
      // Returns module state
      return this.state;
   }

   public SwerveModuleState getActualModuleState() {
      double velocity = this.driveSim.getAngularVelocityRadPerSec() * Math.PI
            * Constants.SwerveConstants.wheelDiameterMeters / Constants.SwerveConstants.driveGearRatio;
      double rotation = this.turnSim.getAngularPositionRad();
      return new SwerveModuleState(velocity, Rotation2d.fromRadians(rotation));
   }

   public void updateSim() {
      // Updates sim with certain dt of 0.02
      this.driveSim.update(0.02);
      this.turnSim.update(0.02);
   }

}