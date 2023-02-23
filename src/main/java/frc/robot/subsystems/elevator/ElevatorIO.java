package frc.robot.subsystems.elevator;

import org.littletonrobotics.junction.AutoLog;

public interface ElevatorIO {
  @AutoLog
  public static class ElevatorIOInputs {
    //at output shaft
    public double positionSetPointInch = 0.0;
    public double positionRad = 0.0;
    public double velocityRadPerSec = 0.0;
    public double appliedVolts = 0.0;
    public double currentAmps =  0.0;
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(ElevatorIOInputs inputs) {
  }

  /** Run closed loop at the specified velocity. */
  public default void setPosition(double positionInch, double ffVolts) {

  }

  /** Stop in open loop. */
  public default void stop() {
  }

  public default void holdCurrent(int amps) {
  }

  public default void configurePID(double kP, double kI, double kD) {
  }

}
