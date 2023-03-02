package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {
  @AutoLog
  public static class IntakeIOInputs {
    //public double positionRad = 0.0;
    public double wheelVelocityRPM = 0.0;
    public double motorVelocityRPM = 0.0;
    public double appliedVolts = 0.0;
    public double currentAmps =  0.0;
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(IntakeIOInputs inputs) {
  }

  /** Run closed loop at the specified velocity. */
  public default void setVelocity(double velocityRPM, double ffVolts) {
  }

  /** Stop in open loop. */
  public default void stop() {
  }

  public default void holdCurrent(int amps, double voltage) {
  }

  public default void configurePID(double kP, double kI, double kD) {
  }

}
