package frc.robot.subsystems.elevator;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Elevator extends SubsystemBase {
  private final ElevatorIO io;
  private final ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();
  private final ElevatorFeedforward ffModel;

  /** Creates a new elevator. */
  public Elevator(ElevatorIO io) {
    this.io = io;

    // Switch constants based on mode (the physics simulator is treated as a
    // separate robot with different tuning)
    switch (Constants.getMode()) {
      case SIM:
        ffModel = new ElevatorFeedforward(Constants.ElevatorSubsystem.ks, Constants.ElevatorSubsystem.kv,
            Constants.ElevatorSubsystem.kg);
            io.configurePID(Constants.ElevatorSubsystem.kP, Constants.ElevatorSubsystem.kI,
            Constants.ElevatorSubsystem.kD);
        break;
      case REAL:
      case REPLAY:
      default:
        ffModel = new ElevatorFeedforward(Constants.ElevatorSubsystem.ks, Constants.ElevatorSubsystem.kv,
            Constants.ElevatorSubsystem.kg);
        io.configurePID(Constants.ElevatorSubsystem.kP, Constants.ElevatorSubsystem.kI,
            Constants.ElevatorSubsystem.kD);
    }
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.getInstance().processInputs("elevator", inputs);

    // Log elevator speed in RPM
    Logger.getInstance().recordOutput("ElevatorSpeedRPM", getVelocityRPM());
    Logger.getInstance().recordOutput("ElevatorSetpointInch", inputs.positionSetPointInch);
    Logger.getInstance().recordOutput("PosErrorInch", getError());
  }

  public void setPosition(double positionInch) {
    var velocityRPM = getVelocityRPM();
    var velocityRadPerSec = Units.rotationsPerMinuteToRadiansPerSecond(velocityRPM);
    io.setPosition(positionInch, ffModel.calculate(velocityRadPerSec));

  }
  public double getError() {
    Logger.getInstance().recordOutput("PosErrorInch",inputs.positionSetPointInch - inputs.positionInch);
    return Math.abs(inputs.positionSetPointInch - inputs.positionInch);
  }

  public void elevatorBottom() {
    setPosition(Constants.ElevatorSubsystem.elevatorPosBottom);
    //while(getError()>Constants.ElevatorSubsystem.autoPositionErrorInch){}

  }

  public void elevatorMid() {
    setPosition(Constants.ElevatorSubsystem.elevatorPosMid);
    //while(getError()>Constants.ElevatorSubsystem.autoPositionErrorInch){}
  }

  public void elevatorLoading() {
    setPosition(Constants.ElevatorSubsystem.elevatorPosLoading);
    //while(getError()>Constants.ElevatorSubsystem.autoPositionErrorInch){}
  }

  public void elevatorTop() {
    setPosition(Constants.ElevatorSubsystem.elevatorPosTop);
    
    //while(getError()>Constants.ElevatorSubsystem.autoPositionErrorInch){}
  }

  /** Stops the elevator. */
  public void stop() {
    io.stop();
  }

  /** Returns the current velocity in RPM. */
  public double getVelocityRPM() {
    return Units.radiansPerSecondToRotationsPerMinute(inputs.velocityRadPerSec);
  }

}
