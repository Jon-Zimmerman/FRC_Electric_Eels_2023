package frc.robot.subsystems.drive;

import java.lang.Cloneable;
import java.lang.Override;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public class LimelightIOInputsAutoLogged extends LimelightIO.LimelightIOInputs implements LoggableInputs, Cloneable {
  @Override
  public void toLog(LogTable table) {
    table.put("Latency", latency);
    table.put("BotPoseWPI", botPoseWPI);
  }

  @Override
  public void fromLog(LogTable table) {
    latency = table.getDouble("Latency", latency);
    botPoseWPI = table.getDoubleArray("BotPoseWPI", botPoseWPI);
  }

  public LimelightIOInputsAutoLogged clone() {
    LimelightIOInputsAutoLogged copy = new LimelightIOInputsAutoLogged();
    copy.latency = this.latency;
    copy.botPoseWPI = this.botPoseWPI.clone();
    return copy;
  }
}
