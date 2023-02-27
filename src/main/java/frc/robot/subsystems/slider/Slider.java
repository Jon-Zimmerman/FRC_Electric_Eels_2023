package frc.robot.subsystems.slider;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Slider extends SubsystemBase {
  private final SliderIO io;
  private final SliderIOInputsAutoLogged inputs = new SliderIOInputsAutoLogged();
  private static final double maxLinearVelocityInchPerSec = Constants.SliderSubsystem.maxLinearVelocityInchPerSec;
  private static final double maxLinearAccelerationInchPerSec = Constants.SliderSubsystem.maxLinearAccelerationInchPerSec;

  private final TrapezoidProfile.Constraints m_constraints = new TrapezoidProfile.Constraints(
      maxLinearVelocityInchPerSec, maxLinearAccelerationInchPerSec);
  private TrapezoidProfile.State m_goal = new TrapezoidProfile.State();
  private TrapezoidProfile.State m_setpoint = new TrapezoidProfile.State();

  private final ElevatorFeedforward ffModel;

  private double positionSetPointInch = 0.0;

  /** Creates a new slider. */
  public Slider(SliderIO io) {
    this.io = io;

    ffModel = new ElevatorFeedforward(Constants.SliderSubsystem.ks, Constants.SliderSubsystem.kg,
        Constants.SliderSubsystem.kv);
    io.configurePID(Constants.SliderSubsystem.kP, Constants.SliderSubsystem.kI,
        Constants.SliderSubsystem.kD);

  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.getInstance().processInputs("slider", inputs);
    // TODO fix setPosition()

    // Log slider speed in RPM
    // Logger.getInstance().recordOutput("SliderSpeedRPM",
    // getVelocityRPMFromRadsPerSec());
    // Logger.getInstance().recordOutput("SliderSetpointInch",
    // inputs.positionSetPointInch);

    var profile = new TrapezoidProfile(m_constraints, m_goal, m_setpoint);
    m_setpoint = profile.calculate(Constants.simLoopPeriodSecs);

    io.setPosition(m_setpoint.position, ffModel.calculate(m_setpoint.velocity));
    Logger.getInstance().recordOutput("SliderPosErrorInch", getError());

  }

  public void setPositionSetPoint(double positionInch) {
    m_goal = new TrapezoidProfile.State(positionInch, 0);
    positionSetPointInch = positionInch;

  }

  public double getError() {
    return Math.abs(inputs.positionSliderSetPointInch - inputs.positionSliderInch);
  }

  // public void sliderIn() {
  //   setPositionSetPoint(Constants.SliderSubsystem.sliderIn);
  //   // while(getError()>Constants.SliderSubsystem.autoPositionErrorInch){}

  // }

  // public void sliderOut() {
  //   setPositionSetPoint(Constants.SliderSubsystem.sliderOut);
  //   // while(getError()>Constants.SliderSubsystem.autoPositionErrorInch){}
  // }


  /** Stops the slider. */
  public void stop() {
    io.stop();
  }

  public boolean atSetpoint() {
    return ((Math.abs(m_goal.position
        - inputs.positionSliderInch)) < Constants.SliderSubsystem.allowableErrorInch);
  }
  // /** Returns the current velocity in RPM. */
  // public double getVelocityRPMFromRadsPerSec() {
  // return Units.radiansPerSecondToRotationsPerMinute(inputs.velocityRadPerSec);
  // }

}
