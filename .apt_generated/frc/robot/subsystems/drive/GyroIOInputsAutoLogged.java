package frc.robot.subsystems.drive;

import java.lang.Cloneable;
import java.lang.Override;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public class GyroIOInputsAutoLogged extends GyroIO.GyroIOInputs implements LoggableInputs, Cloneable {
  @Override
  public void toLog(LogTable table) {
    table.put("YawDegrees", yawDegrees);
    table.put("RollDegrees", rollDegrees);
  }

  @Override
  public void fromLog(LogTable table) {
    yawDegrees = table.getDouble("YawDegrees", yawDegrees);
    rollDegrees = table.getDouble("RollDegrees", rollDegrees);
  }

  public GyroIOInputsAutoLogged clone() {
    GyroIOInputsAutoLogged copy = new GyroIOInputsAutoLogged();
    copy.yawDegrees = this.yawDegrees;
    copy.rollDegrees = this.rollDegrees;
    return copy;
  }
}
