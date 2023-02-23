package frc.robot.subsystems.elevator;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
//or sparkmaxrelativeencoder?
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxPIDController.ArbFFUnits;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotController;
import frc.robot.Constants;

public class ElevatorIOSparkMax implements ElevatorIO {
  private static final double GEAR_RATIO = Constants.ElevatorSubsystem.gearRatio;
  private final CANSparkMax elevatorMotor;
  // private final CANSparkMax follower;
  private final RelativeEncoder elevatorEncoder;

  private final SparkMaxPIDController elevatorPidController;

  public ElevatorIOSparkMax() {
    elevatorMotor = new CANSparkMax(Constants.ElevatorSubsystem.deviceID, MotorType.kBrushless);
    elevatorEncoder = elevatorMotor.getEncoder();
    elevatorPidController = elevatorMotor.getPIDController();

    // follower.burnFlash();
  }

  @Override
  public void updateInputs(ElevatorIOInputs inputs) {
    inputs.positionRad = Units.rotationsToRadians(elevatorEncoder.getPosition() / GEAR_RATIO);
    inputs.velocityRadPerSec = Units.rotationsPerMinuteToRadiansPerSecond(
        elevatorEncoder.getVelocity() / GEAR_RATIO);
    inputs.appliedVolts = elevatorMotor.getAppliedOutput() * RobotController.getBatteryVoltage();
    inputs.currentAmps = elevatorMotor.getOutputCurrent();
  }

  @Override
  public void setPosition(double positionInch, double ffVolts) {
    double setPointRotations = positionInch / (Math.PI * Constants.ElevatorSubsystem.sprocketDiameterInch) * GEAR_RATIO;
    elevatorPidController.setReference(setPointRotations, ControlType.kPosition, 0, ffVolts, ArbFFUnits.kVoltage);
  }

  @Override
  public void stop() {
    // maybe unsafe with elevator falling back?
    elevatorMotor.stopMotor();
  }

  public void configurePID(double kP, double kI, double kD) {
    elevatorMotor.restoreFactoryDefaults();
    elevatorMotor.setInverted(false);
    elevatorMotor.enableVoltageCompensation(12.0);
    elevatorMotor.setSmartCurrentLimit(Constants.ElevatorSubsystem.maxCurrentAmps);

    elevatorPidController.setP(kP);
    elevatorPidController.setI(kI);
    elevatorPidController.setD(kD);
    elevatorPidController.setIZone(Constants.ElevatorSubsystem.kIz);
    elevatorPidController.setFF(Constants.ElevatorSubsystem.kFF);
    elevatorPidController.setOutputRange(Constants.ElevatorSubsystem.kMinOutput,
        Constants.ElevatorSubsystem.kMaxOutput);

    int smartMotionSlot = 0;
    elevatorPidController.setSmartMotionMaxVelocity(Constants.ElevatorSubsystem.maxAngularVelocityRPM, smartMotionSlot);
    elevatorPidController.setSmartMotionMinOutputVelocity(Constants.ElevatorSubsystem.minOutputVelocityRPM,
        smartMotionSlot);
    elevatorPidController.setSmartMotionMaxAccel(Constants.ElevatorSubsystem.maxAngularAccRPMPerSec, smartMotionSlot);
    elevatorPidController.setSmartMotionAllowedClosedLoopError(
        Constants.ElevatorSubsystem.allowableSmartMotionPosErrorCounts, smartMotionSlot);

    elevatorMotor.burnFlash();
  }
}
