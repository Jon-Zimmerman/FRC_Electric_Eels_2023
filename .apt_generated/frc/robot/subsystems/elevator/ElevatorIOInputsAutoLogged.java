package frc.robot.subsystems.elevator;

import java.lang.Cloneable;
import java.lang.Override;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public class ElevatorIOInputsAutoLogged extends ElevatorIO.ElevatorIOInputs implements LoggableInputs, Cloneable {
  @Override
  public void toLog(LogTable table) {
    table.put("PositionSetPointInch", positionSetPointInch);
    table.put("PositionRad", positionRad);
    table.put("PositionInch", positionInch);
    table.put("VelocityRadPerSec", velocityRadPerSec);
    table.put("AppliedVolts", appliedVolts);
    table.put("CurrentAmps", currentAmps);
  }

  @Override
  public void fromLog(LogTable table) {
    positionSetPointInch = table.getDouble("PositionSetPointInch", positionSetPointInch);
    positionRad = table.getDouble("PositionRad", positionRad);
    positionInch = table.getDouble("PositionInch", positionInch);
    velocityRadPerSec = table.getDouble("VelocityRadPerSec", velocityRadPerSec);
    appliedVolts = table.getDouble("AppliedVolts", appliedVolts);
    currentAmps = table.getDouble("CurrentAmps", currentAmps);
  }

  public ElevatorIOInputsAutoLogged clone() {
    ElevatorIOInputsAutoLogged copy = new ElevatorIOInputsAutoLogged();
    copy.positionSetPointInch = this.positionSetPointInch;
    copy.positionRad = this.positionRad;
    copy.positionInch = this.positionInch;
    copy.velocityRadPerSec = this.velocityRadPerSec;
    copy.appliedVolts = this.appliedVolts;
    copy.currentAmps = this.currentAmps;
    return copy;
  }
}
