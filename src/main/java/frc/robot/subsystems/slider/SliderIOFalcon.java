package frc.robot.subsystems.slider;



import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotController;
import frc.robot.Constants;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;



public class SliderIOFalcon implements SliderIO {
  private static final double GEAR_RATIO = Constants.SliderSubsystem.gearRatio;
  private final WPI_TalonFX  sliderMotor;
  //private final CANSparkMax follower;


  public SliderIOFalcon() {
    sliderMotor= new WPI_TalonFX(Constants.SliderSubsystem.deviceID, "rio"); //change rio to canivore device name 
    configurePID();
    //follower.burnFlash();
  }

  @Override
  public void updateInputs(SliderIOInputs inputs) {
    inputs.positionRad = Units.rotationsToRadians(sliderMotor.getSelectedSensorPosition() / GEAR_RATIO);
    inputs.velocityRadPerSec = Units.rotationsPerMinuteToRadiansPerSecond(
      sliderMotor.getSelectedSensorVelocity() * 10.0 /2048.0* GEAR_RATIO);
    inputs.appliedVolts = sliderMotor.getMotorOutputPercent() * RobotController.getBatteryVoltage();
    inputs.currentAmps = sliderMotor.getSupplyCurrent();
  }

  @Override
  public void setPosition(double positionInch, double ffVolts) {
    double setPointSensorCounts = positionInch/(Math.PI*Constants.SliderSubsystem.sprocketDiameterInch)/GEAR_RATIO* 2048*10.0;
    sliderMotor.set(TalonFXControlMode.MotionMagic,setPointSensorCounts);
  }

  @Override
  public void stop() {
    //maybe unsafe with slider falling back out?
    sliderMotor.stopMotor();
  }
  
  public void configurePID() {
    
    sliderMotor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor,0,0);
    sliderMotor.configNeutralDeadband(0.04,0);
    sliderMotor.setSensorPhase(false);
    sliderMotor.setInverted(false);

		sliderMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 10, Constants.SliderSubsystem.kTimeoutMs);
		sliderMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, 10, Constants.SliderSubsystem.kTimeoutMs);

		/* Set the peak and nominal outputs */
		sliderMotor.configNominalOutputForward(0, Constants.SliderSubsystem.kTimeoutMs);
		sliderMotor.configNominalOutputReverse(0, Constants.SliderSubsystem.kTimeoutMs);
		sliderMotor.configPeakOutputForward(1, Constants.SliderSubsystem.kTimeoutMs);
		sliderMotor.configPeakOutputReverse(-1, Constants.SliderSubsystem.kTimeoutMs);

    sliderMotor.configStatorCurrentLimit(new StatorCurrentLimitConfiguration(true,Constants.SliderSubsystem.maxCurrentAmps,30,1.0));
    sliderMotor.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true,Constants.SliderSubsystem.maxCurrentAmps/2.0,20,0.5));

		/* Set Motion Magic gains in slot0 - see documentation */
		sliderMotor.selectProfileSlot(0, 0);
		sliderMotor.config_kF(0, Constants.SliderSubsystem.kF, Constants.SliderSubsystem.kTimeoutMs);
		sliderMotor.config_kP(0, Constants.SliderSubsystem.kP, Constants.SliderSubsystem.kTimeoutMs);
		sliderMotor.config_kI(0, Constants.SliderSubsystem.kI, Constants.SliderSubsystem.kTimeoutMs);
		sliderMotor.config_kD(0, Constants.SliderSubsystem.kD, Constants.SliderSubsystem.kTimeoutMs);
    double CruiseVel = Constants.SliderSubsystem.maxLinearVelocityInchPS /
    ( Constants.SliderSubsystem.sprocketDiameterInch * Math.PI) * 100.0/1000.0 * (float)Constants.SliderSubsystem.SensorResolution;
    double CruiseAcc= Constants.SliderSubsystem.maxLinearAccInchPSPerSec /
    ( Constants.SliderSubsystem.sprocketDiameterInch * Math.PI) * 100.0/1000.0 * (float)Constants.SliderSubsystem.SensorResolution;
		
    /* Set acceleration and vcruise velocity - see documentation */
		sliderMotor.configMotionCruiseVelocity(CruiseVel, Constants.SliderSubsystem.kTimeoutMs);
		sliderMotor.configMotionAcceleration(CruiseAcc, Constants.SliderSubsystem.kTimeoutMs);

		/* Zero the sensor once on robot boot up */
		sliderMotor.setSelectedSensorPosition(0, 0, Constants.SliderSubsystem.kTimeoutMs);
    
    
  }
}
