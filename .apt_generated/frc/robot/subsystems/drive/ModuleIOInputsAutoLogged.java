package frc.robot.subsystems.drive;

import java.lang.Cloneable;
import java.lang.Override;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public class ModuleIOInputsAutoLogged extends ModuleIO.ModuleIOInputs implements LoggableInputs, Cloneable {
  @Override
  public void toLog(LogTable table) {
    table.put("DriveMotorPositionDistanceMeters", driveMotorPositionDistanceMeters);
    table.put("DriveMotorStateMetersPerSecond", driveMotorStateMetersPerSecond);
    table.put("AngleMotorPositionDegrees", angleMotorPositionDegrees);
    table.put("CancoderDegrees", cancoderDegrees);
  }

  @Override
  public void fromLog(LogTable table) {
    driveMotorPositionDistanceMeters = table.getDouble("DriveMotorPositionDistanceMeters", driveMotorPositionDistanceMeters);
    driveMotorStateMetersPerSecond = table.getDouble("DriveMotorStateMetersPerSecond", driveMotorStateMetersPerSecond);
    angleMotorPositionDegrees = table.getDouble("AngleMotorPositionDegrees", angleMotorPositionDegrees);
    cancoderDegrees = table.getDouble("CancoderDegrees", cancoderDegrees);
  }

  public ModuleIOInputsAutoLogged clone() {
    ModuleIOInputsAutoLogged copy = new ModuleIOInputsAutoLogged();
    copy.driveMotorPositionDistanceMeters = this.driveMotorPositionDistanceMeters;
    copy.driveMotorStateMetersPerSecond = this.driveMotorStateMetersPerSecond;
    copy.angleMotorPositionDegrees = this.angleMotorPositionDegrees;
    copy.cancoderDegrees = this.cancoderDegrees;
    return copy;
  }
}
