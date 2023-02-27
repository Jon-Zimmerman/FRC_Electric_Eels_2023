package frc.robot.subsystems.elevator;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Elevator extends SubsystemBase {
  private final ElevatorIO io;
  private final ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();
  private static final double maxLinearVelocityInchPerSec = Constants.ElevatorSubsystem.maxLinearVelocityInchPerSec;
  private static final double maxLinearAccelerationInchPerSec = Constants.ElevatorSubsystem.maxLinearAccelerationInchPerSec;

  private final TrapezoidProfile.Constraints m_constraints = new TrapezoidProfile.Constraints(
      maxLinearVelocityInchPerSec, maxLinearAccelerationInchPerSec);
  private TrapezoidProfile.State m_goal = new TrapezoidProfile.State();
  private TrapezoidProfile.State m_setpoint = new TrapezoidProfile.State();

  private final ElevatorFeedforward ffModel;

  private double positionSetPointInch = 0.0;

  /** Creates a new elevator. */
  public Elevator(ElevatorIO io) {
    this.io = io;

    ffModel = new ElevatorFeedforward(Constants.ElevatorSubsystem.ks, Constants.ElevatorSubsystem.kg,
        Constants.ElevatorSubsystem.kv);
    io.configurePID(Constants.ElevatorSubsystem.kP, Constants.ElevatorSubsystem.kI,
        Constants.ElevatorSubsystem.kD);

  }
 
  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.getInstance().processInputs("elevator", inputs);
    // TODO fix setPosition()

    // Log elevator speed in RPM
    // Logger.getInstance().recordOutput("ElevatorSpeedRPM",
    // getVelocityRPMFromRadsPerSec());
    // Logger.getInstance().recordOutput("ElevatorSetpointInch",
    // inputs.positionSetPointInch);

    var profile = new TrapezoidProfile(m_constraints, m_goal, m_setpoint);
    m_setpoint = profile.calculate(Constants.simLoopPeriodSecs);

    io.setPosition(m_setpoint.position, ffModel.calculate(m_setpoint.velocity));
    Logger.getInstance().recordOutput("ElevatorPosErrorInch", getError());

  }

  public void setPositionSetPoint(double positionInch) {
    m_goal = new TrapezoidProfile.State(positionInch, 0);
    positionSetPointInch = positionInch;

  }

  public double getError() {
    return Math.abs(inputs.positionElevatorSetPointInch - inputs.positionElevatorInch);
  }

  public void elevatorBottom() {
    setPositionSetPoint(Constants.ElevatorSubsystem.elevatorPosBottom);
    // while(getError()>Constants.ElevatorSubsystem.autoPositionErrorInch){}

  }

  public void elevatorMid() {
    setPositionSetPoint(Constants.ElevatorSubsystem.elevatorPosMid);
    // while(getError()>Constants.ElevatorSubsystem.autoPositionErrorInch){}
  }

  public void elevatorLoading() {
    setPositionSetPoint(Constants.ElevatorSubsystem.elevatorPosLoading);
    // while(getError()>Constants.ElevatorSubsystem.autoPositionErrorInch){}
  }

  public void elevatorTop() {
    setPositionSetPoint(Constants.ElevatorSubsystem.elevatorPosTop);

    // while(getError()>Constants.ElevatorSubsystem.autoPositionErrorInch){}
  }

  /** Stops the elevator. */
  public void stop() {
    io.stop();
  }

  // /** Returns the current velocity in RPM. */
  // public double getVelocityRPMFromRadsPerSec() {
  // return Units.radiansPerSecondToRotationsPerMinute(inputs.velocityRadPerSec);
  // }

}
