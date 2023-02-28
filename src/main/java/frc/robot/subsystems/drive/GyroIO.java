package frc.robot.subsystems.drive;

import org.littletonrobotics.junction.AutoLog;
//import edu.wpi.first.math.geometry.Rotation2d;

public interface GyroIO {
  @AutoLog
  public static class GyroIOInputs {
      //public boolean connected = false;
      //public double yaw;
      public double yawDegrees;
      public double rollDegrees;
      public double pitchDegrees;
  }


  public default void updateInputs(GyroIOInputs inputs) {}

  public default void zeroGyro() {}
  public default void calibrateGyro() {}
  public default void additionalRotation(double rotation) {}
}