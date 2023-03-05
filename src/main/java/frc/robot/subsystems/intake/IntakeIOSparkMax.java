package frc.robot.subsystems.intake;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxPIDController.ArbFFUnits;

//import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotController;
import frc.robot.Constants;

public class IntakeIOSparkMax implements IntakeIO {
  //private static final double gearRatio = Constants.IntakeSubsystem.gearRatio;
  private final CANSparkMax intakeMotor;
  // private final CANSparkMax follower;
  private final RelativeEncoder intakeEncoder;

  private final SparkMaxPIDController intakePidController;
  private double motorVelocitySetPointRPM = 0.0;

  public double motorVelocityRPM = 0.0;

  public IntakeIOSparkMax() {
    intakeMotor = new CANSparkMax(Constants.IntakeSubsystem.deviceID, MotorType.kBrushless);
    intakeEncoder = intakeMotor.getEncoder();
    intakePidController = intakeMotor.getPIDController();

    // follower.burnFlash();
  }

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    // inputs.positionRad = Units.rotationsToRadians(intakeEncoder.getPosition() /
    // gearRatio);
    inputs.motorVelocityRPM = intakeEncoder.getVelocity();
    inputs.appliedVolts = intakeMotor.getAppliedOutput() * RobotController.getBatteryVoltage();
    inputs.currentAmps = intakeMotor.getOutputCurrent();
  }

  @Override
  public void setVelocity(double motorVelocitySetRPM, double ffVolts) {
    motorVelocitySetPointRPM = motorVelocitySetRPM;
    intakePidController.setReference(
        motorVelocitySetPointRPM,
        ControlType.kVelocity, 0, ffVolts, ArbFFUnits.kVoltage);
  }

  @Override
  public void stop() {
    intakeMotor.stopMotor();
  }

  @Override
  public void holdCurrent(int amps, double voltage) {
    stop();
    intakeMotor.set(voltage / 12.0);
    intakeMotor.setSmartCurrentLimit(amps);
  }

  public void configurePID(double kP, double kI, double kD) {
    intakeMotor.restoreFactoryDefaults();
    intakeMotor.setInverted(Constants.IntakeSubsystem.isInverted);
    intakeMotor.enableVoltageCompensation(12.0);
    intakeMotor.setSmartCurrentLimit(Constants.IntakeSubsystem.maxCurrentAmps);

    intakePidController.setP(kP);
    intakePidController.setI(kI);
    intakePidController.setD(kD);
    intakePidController.setIZone(Constants.IntakeSubsystem.kIz);
    intakePidController.setFF(Constants.IntakeSubsystem.kFF);
    intakePidController.setOutputRange(Constants.IntakeSubsystem.kMinOutput,
        Constants.IntakeSubsystem.kMaxOutput);

    intakeMotor.burnFlash();
  }
}
