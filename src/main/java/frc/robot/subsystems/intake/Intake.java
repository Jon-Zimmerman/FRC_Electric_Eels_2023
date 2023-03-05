package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
//import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake extends SubsystemBase {
  private final IntakeIO io;
  private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();
  private final SimpleMotorFeedforward ffModel;
  private boolean ConeMode;
  private double wheelVelocitySetPointRPM;
  public double wheelVelocityRPM = 0.0;
  public double motorVelocityRPM = 0.0;
  private static final double gearRatio = Constants.IntakeSubsystem.gearRatio;

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
        ffModel = new SimpleMotorFeedforward(0.0, 0.00);
        io.configurePID(Constants.IntakeSubsystem.kP, Constants.IntakeSubsystem.kI,
            Constants.IntakeSubsystem.kD);
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
  public void runVelocity(double wheelVelocitySetRPM) {
    wheelVelocitySetPointRPM = wheelVelocitySetRPM;

    io.setVelocity(wheelVelocitySetPointRPM*gearRatio, 0.0); //ffModel.calculate(wheelVelocitySetRPM*gearRatio)

    // Log intake setpoint
    Logger.getInstance().recordOutput("IntakeSetpointRPM", wheelVelocitySetRPM);
  }

  public void intakeIn() {
    if(ConeMode){
      runVelocity(Constants.IntakeSubsystem.intakeInConeVelRPM);
    }
    else{
      runVelocity(Constants.IntakeSubsystem.intakeInCubeVelRPM);
    }

  }

  public void intakeOut() {  
    if(ConeMode){
      runVelocity(Constants.IntakeSubsystem.intakeOutConeVelRPM);
    }
    else{
      runVelocity(Constants.IntakeSubsystem.intakeOutCubeVelRPM);
    }
  }

  /** Stops the intake. */
  public void holdCurrent() {
    if(ConeMode){
      io.holdCurrent(Constants.IntakeSubsystem.holdConeCurrentAmps,-8.0);
    }
    else{
      io.holdCurrent(Constants.IntakeSubsystem.holdCubeCurrentAmps, 8.0);
    }



  }

  /** Stops the intake. */
  public void stop() {
    io.stop();
  }

  /** Returns the current velocity in RPM. */
  public double getVelocityRPM() {
    return inputs.motorVelocityRPM/gearRatio;
  }

  public void flipIntakeMode() {
    ConeMode = !ConeMode;
  }

}
