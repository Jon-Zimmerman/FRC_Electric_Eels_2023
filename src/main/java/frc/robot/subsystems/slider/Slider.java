package frc.robot.subsystems.slider;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Slider extends SubsystemBase {
  private final SliderIO io;
  private final SliderIOInputsAutoLogged inputs = new SliderIOInputsAutoLogged();
  private final ElevatorFeedforward ffModel;

  /** Creates a new slider. */
  public Slider(SliderIO io) {
    this.io = io;

    // Switch constants based on mode (the physics simulator is treated as a
    // separate robot with different tuning)
    switch (Constants.getMode()) {
      case SIM:
        ffModel = new ElevatorFeedforward(Constants.SliderSubsystem.ks, Constants.SliderSubsystem.kv,
            Constants.SliderSubsystem.kg);
            io.configurePID(Constants.SliderSubsystem.kP, Constants.SliderSubsystem.kI,
            Constants.SliderSubsystem.kD);
        break;
      case REAL:
      case REPLAY:
      default:
        ffModel = new ElevatorFeedforward(Constants.SliderSubsystem.ks, Constants.SliderSubsystem.kv,
            Constants.SliderSubsystem.kg);
        io.configurePID(Constants.SliderSubsystem.kP, Constants.SliderSubsystem.kI,
            Constants.SliderSubsystem.kD);
    }
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.getInstance().processInputs("slider", inputs);

    // Log slider speed in RPM
    Logger.getInstance().recordOutput("SliderVelRPM", getVelocityRPM());
    Logger.getInstance().recordOutput("SliderPosInch", inputs.positionInch);
    Logger.getInstance().recordOutput("SliderSetpointInch", inputs.positionSetPointInch);
  }

  public void runPosition(double positionInch) {
    var velocityRPM = getVelocityRPM();
    var velocityRadPerSec = Units.rotationsPerMinuteToRadiansPerSecond(velocityRPM);
    io.setPosition(positionInch, ffModel.calculate(velocityRadPerSec));

    // Log slider setpoint

  }

  public void sliderIn() {
    runPosition(Constants.SliderSubsystem.sliderIn);
  }

  public void sliderOut() {
    runPosition(Constants.SliderSubsystem.sliderOut);
  }

  /** Stops the slider. */
  public void stop() {
    io.stop();
  }

  /** Returns the current velocity in RPM. */
  public double getVelocityRPM() {
    return Units.radiansPerSecondToRotationsPerMinute(inputs.velocityRadPerSec);
  }
}
