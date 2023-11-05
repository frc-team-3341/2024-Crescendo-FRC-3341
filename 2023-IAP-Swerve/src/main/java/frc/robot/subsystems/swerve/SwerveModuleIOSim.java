package frc.robot.subsystems.swerve;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.Constants.ModuleConstants;
import frc.util.lib.CTREModuleState;

public class SwerveModuleIOSim implements SwerveModuleIO {

   // FlywheelSim is more tuneable for feedforward purposes
   private final LinearSystem<N1, N1, N1> flywheelPlant =
   LinearSystemId.identifyVelocitySystem(0.1261, 0.001);


   // Create drive and turn sim motors
   private final FlywheelSim driveSim = new FlywheelSim(flywheelPlant, DCMotor.getNEO(0), Constants.SwerveConstants.driveGearRatio);
    
   private final DCMotorSim turnSim = new DCMotorSim(DCMotor.getNEO(1), Constants.SwerveConstants.turnGearRatio,
         0.000001);

   // Create PID controllers with constants
   // Note lack of feedforward
   // Native unit: volt
   private final PIDController drivePID = new PIDController(3, 0, 0.00);

   // FIX: DO NOT USE DERIVATIVE
   // When approaching the end points, the derivative experiences discontinuity
   // Aka the WPILib PID's derivative is not implemented well
   private final PIDController turnPID = new PIDController(3.0, 0.0, 0.00);

   // Create variables to hold driving and turning voltage
   private double driveVolts = 0.0;
   private double turnVolts = 0.0;

   private double integratedPosition = 0.0;

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
      turnPID.enableContinuousInput(0, 2*Math.PI);
   }

   public void setDriveVoltage(double voltage) {
      this.driveVolts = MathUtil.clamp(voltage, -12.0, 12.0);
      
      this.driveSim.setInputVoltage(this.driveVolts);

      SmartDashboard.putNumber("Drive Volts #" + this.num, this.driveVolts);
   }

   public void setTurnVoltage(double voltage) {
      this.turnVolts = MathUtil.clamp(voltage, -12.0, 12.0);
      this.turnSim.setInputVoltage(this.turnVolts);

      SmartDashboard.putNumber("Turn Volts #" + this.num, this.turnVolts);
   }

   public void setDesiredState(SwerveModuleState state) {

      // Optimize state so that movement is minimized
      // FIX FOR SIM: Use CTREModuleState to optimize for 0 to 360
      state = CTREModuleState.optimize(state, new Rotation2d(getTurnPositionInRad()));

      // Set internal state as passed-in state
      this.state = state;

      double velocity;
      if (!(DriverStation.isDisabled() | DriverStation.isEStopped())) {
         

         // Set setpoint of the drive PID controller
         this.drivePID.setSetpoint(state.speedMetersPerSecond);

         // Find measurement in m/s and calculate PID action
         velocity = this.driveSim.getAngularVelocityRadPerSec() * Math.PI
               * Constants.SwerveConstants.wheelDiameterMeters / Constants.SwerveConstants.driveGearRatio;
   

         double output = this.drivePID.calculate(velocity) + (12.0/Constants.ModuleConstants.maxFreeWheelSpeedMeters)*state.speedMetersPerSecond;

         // Output in volts to motor
         setDriveVoltage(output);
         SmartDashboard.putNumber("Closed Loop Drive #" + this.num, driveVolts);
         SmartDashboard.putNumber("Drive Vel #" + this.num, velocity);
         SmartDashboard.putNumber("Setpoint Drive Vel #" + this.num, state.speedMetersPerSecond);
      } else {
         // Output in volts to motor
         driveVolts = 0.0;
         setDriveVoltage(0.0);
         SmartDashboard.putNumber("Closed Loop Drive #" + this.num, 0.0);
      }

      // Turn with PID in volts
      this.turnPID.setSetpoint(state.angle.getRadians());
      SmartDashboard.putNumber("Turning Angle #" + this.num,
            Units.radiansToDegrees(getTurnPositionInRad()));
      SmartDashboard.putNumber("Raw Turn Pos #" + num, getTurnPositionInRad());

      double turnOutput = this.turnPID.calculate(getTurnPositionInRad());
      setTurnVoltage(turnOutput);
   }

   public SwerveModulePosition getPosition() {
      // Get position of swerve module in meters and radians
      SwerveModulePosition position = new SwerveModulePosition(integratedPosition,
            new Rotation2d(getTurnPositionInRad()));
      SmartDashboard.putNumber("Wheel Displacement #" + this.num, position.distanceMeters);
      return position;
   }

   public SwerveModuleState getSetpointModuleState() {
      // Returns module state
      return this.state;
   }

   public double getTurnPositionInRad() {
      // FIX: Modulo the angle of the sim by 2PI to wrap it around to 0!
      double rawAngle = Math.signum(this.turnSim.getAngularPositionRad())
            * (Math.abs(this.turnSim.getAngularPositionRad()) % (2 * Math.PI));
      return rawAngle;
   }

   public SwerveModuleState getActualModuleState() {
      double velocity = this.driveSim.getAngularVelocityRadPerSec() * Math.PI
            * Constants.SwerveConstants.wheelDiameterMeters / Constants.SwerveConstants.driveGearRatio;
      double rotation = getTurnPositionInRad();
      return new SwerveModuleState(velocity, Rotation2d.fromRadians(rotation));
   }

   public void updateSim() {
      // Updates sim with certain dt of 0.02
      this.driveSim.update(0.02);
      this.turnSim.update(0.02);

      integratedPosition += 0.02 * this.driveSim.getAngularVelocityRadPerSec() * Math.PI
      * Constants.SwerveConstants.wheelDiameterMeters / Constants.SwerveConstants.driveGearRatio;
      SmartDashboard.putNumber("Mod constant", ModuleConstants.maxFreeWheelSpeedMeters);
   }

   public int getNum() {
      return num;
   }

}