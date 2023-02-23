package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;


public class Intake extends SubsystemBase {
  private final IntakeIO io;
  private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();
  private final SimpleMotorFeedforward ffModel;

  /** Creates a new Intake. */
  public Intake(IntakeIO io) {
    this.io = io;

    // Switch constants based on mode (the physics simulator is treated as a
    // separate robot with different tuning)
    switch (Constants.getMode()) {
      case REAL:
        ffModel = new SimpleMotorFeedforward(Constants.IntakeSubsystem.ks, Constants.IntakeSubsystem.kv);
        io.configurePID(Constants.IntakeSubsystem.kP, Constants.IntakeSubsystem.kI,
            Constants.IntakeSubsystem.kD);
        break;
      case REPLAY:
        ffModel = new SimpleMotorFeedforward(Constants.IntakeSubsystem.ks, Constants.IntakeSubsystem.kv);
        io.configurePID(Constants.IntakeSubsystem.kP, Constants.IntakeSubsystem.kI,
            Constants.IntakeSubsystem.kD);
        break;
      case SIM:
        ffModel = new SimpleMotorFeedforward(0.1, 0.05);
        io.configurePID(0.5, 0.0, 0.0);
        break;
      default:
      ffModel = new SimpleMotorFeedforward(Constants.IntakeSubsystem.ks, Constants.IntakeSubsystem.kv);
      io.configurePID(Constants.IntakeSubsystem.kP, Constants.IntakeSubsystem.kI,
          Constants.IntakeSubsystem.kD);
        break;
    }
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.getInstance().processInputs("Intake", inputs);

    // Log intake speed in RPM
    Logger.getInstance().recordOutput("IntakeSpeedRPM", getVelocityRPM());
  }

  /** Run closed loop at the specified velocity. */
  public void runVelocity(double velocityRPM) {
    var velocityRadPerSec = Units.rotationsPerMinuteToRadiansPerSecond(velocityRPM);
    io.setVelocity(velocityRadPerSec, ffModel.calculate(velocityRadPerSec));

    // Log intake setpoint
    Logger.getInstance().recordOutput("IntakeSetpointRPM", velocityRPM);
  }

  public void intakeIn() {
    // TODO conemode eval intake in direction
    // if conemode
    runVelocity(Constants.IntakeSubsystem.intakeInConeVelRPM);
  }

  public void intakeOut() {
    // TODO conemode eval intake out direction
    // if conemode
    runVelocity(Constants.IntakeSubsystem.intakeOutConeVelRPM);
  }

  /** Stops the intake. */
  public void holdCurrent() {
    io.holdCurrent(Constants.IntakeSubsystem.holdCurrentAmps);
  }

  /** Stops the intake. */
  public void stop() {
    io.stop();
  }

  /** Returns the current velocity in RPM. */
  public double getVelocityRPM() {
    return Units.radiansPerSecondToRotationsPerMinute(inputs.velocityRadPerSec);
  }
}
