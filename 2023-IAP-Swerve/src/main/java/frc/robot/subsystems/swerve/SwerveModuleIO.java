package frc.robot.subsystems.swerve;

import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

/**
 * Creates a SwerveModuleIO interface. This is extendable and is used for
 * different module types and simulations.
 * 
 * @author Aric Volman
 */
public interface SwerveModuleIO {

   /**
    * Set swerve module's state in m/s and radians.
    * 
    * @param state State of module
    */
   default void setDesiredState(SwerveModuleState state) {
   }

   /**
    * Gets swerve module's set state
    */
   default SwerveModuleState getSetpointModuleState() {
      return null;
   }

   /**
    * Gets swerve module's real state
    */
   default SwerveModuleState getActualModuleState() {
      return null;
   }

   /**
    * Sets voltage of swerve module's drive motor (-12.0 to 12.0).
    * 
    * @param volts Voltage to set
    */
   default void setDriveVoltage(double volts) {
   }

   /**
    * Sets voltage of swerve module's turn motor (-12.0 to 12.0).
    * 
    * @param volts Voltage to set
    */
   default void setTurnVoltage(double volts) {
   }

   /**
    * Sets brake mode of swerve module's drive motor.
    * 
    * @param enable Enables brake mode if true
    */
   default void setDriveBrakeMode(boolean enable) {
   }

   /**
    * Sets brake mode of swerve module's turn motor.
    * 
    * @param enable Enables brake mode if true
    */
   default void setTurnBrakeMode(boolean enable) {
   }

   /**
    * Gets swerve module's position as an object.
    */
   default SwerveModulePosition getPosition() {
      return null;
   }

   /**
    * Resets encoders of swerve drive motor.
    */
   default void resetEncoders() {
   }

   /**
    * Updates simulation motors by a constant dt (0.02)
    */
   default void updateSim() {
   }

   /**
    * Gets number of module
    */
   default int getNum() {
      return 0;
   }

   /**
    * Gets turn position in radians
    */
   default double getTurnPositionInRad() {
      return 0.0;
   }
}