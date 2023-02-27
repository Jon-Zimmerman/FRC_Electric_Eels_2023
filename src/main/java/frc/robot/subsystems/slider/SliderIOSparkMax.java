package frc.robot.subsystems.slider;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxPIDController.ArbFFUnits;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotController;
import frc.robot.Constants;



public class SliderIOSparkMax implements SliderIO {
  private static final double GEAR_RATIO = Constants.SliderSubsystem.gearRatio;
  private final CANSparkMax sliderMotor;
  //private final CANSparkMax follower;
  private final RelativeEncoder sliderEncoder;



  private final SparkMaxPIDController sliderPidController;

  public SliderIOSparkMax() {
    sliderMotor= new CANSparkMax(Constants.SliderSubsystem.deviceID, MotorType.kBrushless);
    sliderEncoder = sliderMotor.getEncoder();
    sliderPidController = sliderMotor.getPIDController();

    //follower.burnFlash();
  }

  @Override
  public void updateInputs(SliderIOInputs inputs) {
    //inputs.positionRad = Units.rotationsToRadians(sliderEncoder.getPosition() / GEAR_RATIO);
    //inputs.velocityRadPerSec = Units.rotationsPerMinuteToRadiansPerSecond(
    //  sliderEncoder.getVelocity() / GEAR_RATIO);
    inputs.appliedVolts = sliderMotor.getAppliedOutput() * RobotController.getBatteryVoltage();
    inputs.currentAmps = sliderMotor.getOutputCurrent();
  }

  @Override
  public void setPosition(double positionInch, double ffVolts) {
    double setPointRotations = positionInch/(Math.PI*Constants.SliderSubsystem.sprocketDiameterInch)*GEAR_RATIO;
    sliderPidController.setReference(setPointRotations,ControlType.kPosition, 0, ffVolts, ArbFFUnits.kVoltage);
  }

  @Override
  public void stop() {
    //maybe unsafe with slider falling back?
    sliderMotor.stopMotor();
  }
  
  public void configurePID(double kP, double kI, double kD) {
    sliderMotor.restoreFactoryDefaults();
    sliderMotor.setInverted(false);
    sliderMotor.enableVoltageCompensation(12.0);
    sliderMotor.setSmartCurrentLimit(Constants.SliderSubsystem.maxCurrentAmps);

    sliderPidController.setP(kP);
    sliderPidController.setI(kI);
    sliderPidController.setD(kD);
    sliderPidController.setIZone(Constants.SliderSubsystem.kIz);
    sliderPidController.setFF(Constants.SliderSubsystem.kF);
    sliderPidController.setOutputRange(Constants.SliderSubsystem.kMinOutput, Constants.SliderSubsystem.kMaxOutput);


    int smartMotionSlot = 0;
    // sliderPidController.setSmartMotionMaxVelocity(Constants.SliderSubsystem.maxAngularVelocityRPM, smartMotionSlot);
    // sliderPidController.setSmartMotionMinOutputVelocity(Constants.SliderSubsystem.minOutputVelocityRPM, smartMotionSlot);
    // sliderPidController.setSmartMotionMaxAccel(Constants.SliderSubsystem.maxAngularAccRPMPerSec, smartMotionSlot);
    // sliderPidController.setSmartMotionAllowedClosedLoopError(Constants.SliderSubsystem.allowableSmartMotionPosErrorCounts, smartMotionSlot);


    sliderMotor.burnFlash();
  }
}
