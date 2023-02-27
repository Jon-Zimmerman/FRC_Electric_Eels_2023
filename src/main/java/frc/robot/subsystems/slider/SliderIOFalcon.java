package frc.robot.subsystems.slider;



import edu.wpi.first.wpilibj.RobotController;
import frc.robot.Constants;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;



public class SliderIOFalcon implements SliderIO {
  private static final double gearRatio = Constants.SliderSubsystem.gearRatio;
  private static final double sensorResolution = Constants.SliderSubsystem.sensorResolution;
  private static final double sprocketDiameterInch = Constants.SliderSubsystem.sprocketDiameterInch;
   private final WPI_TalonFX  sliderMotor;
  //private final CANSparkMax follower;


  public SliderIOFalcon() {
    sliderMotor= new WPI_TalonFX(Constants.SliderSubsystem.deviceID, "rio"); //change rio to canivore device name 

    //follower.burnFlash();
  }
  private double positionSliderSetPointInch = 0.0;
  private double positionSliderInch = 0.0;
  private double velocitySliderInchPerSec = 0.0;
  private double positionMotorSetPointRot = 0.0;
  private double positionMotorShaftRot = 0.0;
  private double velocityMotorRPM = 0.0;
  private double appliedVolts = 0.0;
  private double currentAmps = 0.0;
  @Override
  public void updateInputs(SliderIOInputs inputs) {
    updateState();
    inputs.positionSliderSetPointInch = positionSliderSetPointInch;
    inputs.positionSliderInch = positionSliderInch;
    inputs.velocitySliderInchPerSec = velocitySliderInchPerSec;
    inputs.positionMotorSetPointRot = positionMotorSetPointRot;
    inputs.positionMotorShaftRot = positionMotorShaftRot;
    inputs.velocityMotorRPM = velocityMotorRPM;
    inputs.appliedVolts = appliedVolts;
    inputs.currentAmps = currentAmps;

  }

  @Override
  public void setPosition(double positionInch, double ffVolts) {


    double setPointSensorCounts = positionInch/(sprocketDiameterInch)/gearRatio* sensorResolution*10.0;
    sliderMotor.set(TalonFXControlMode.MotionMagic,setPointSensorCounts);
  }
  
  @Override
  public void updateState() {
    positionMotorShaftRot = sliderMotor.getSelectedSensorPosition() / gearRatio;
    velocityMotorRPM = sliderMotor.getSelectedSensorVelocity() * 10.0 /sensorResolution * gearRatio;
    positionSliderInch = positionMotorShaftRot/gearRatio*sprocketDiameterInch*Math.PI;
    velocitySliderInchPerSec = velocityMotorRPM/gearRatio*sprocketDiameterInch*Math.PI;
    appliedVolts = sliderMotor.getMotorOutputPercent() * RobotController.getBatteryVoltage();
    currentAmps = sliderMotor.getSupplyCurrent();
  }

  @Override
  public void stop() {
    //maybe unsafe with slider falling back out?
    sliderMotor.stopMotor();
  }

  @Override
  public void configurePID(double kP, double kI, double kD) {
    
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
		sliderMotor.config_kP(0, kP, Constants.SliderSubsystem.kTimeoutMs);
		sliderMotor.config_kI(0, kI, Constants.SliderSubsystem.kTimeoutMs);
		sliderMotor.config_kD(0, kD, Constants.SliderSubsystem.kTimeoutMs);
    double CruiseVel = Constants.SliderSubsystem.maxLinearVelocityInchPerSec /
    ( Constants.SliderSubsystem.sprocketDiameterInch * Math.PI) * 100.0/1000.0 * (float)Constants.SliderSubsystem.sensorResolution;
    double CruiseAcc= Constants.SliderSubsystem.maxLinearAccelerationInchPerSec /
    ( Constants.SliderSubsystem.sprocketDiameterInch * Math.PI) * 100.0/1000.0 * (float)Constants.SliderSubsystem.sensorResolution;
		
    /* Set acceleration and vcruise velocity - see documentation */
		sliderMotor.configMotionCruiseVelocity(CruiseVel, Constants.SliderSubsystem.kTimeoutMs);
		sliderMotor.configMotionAcceleration(CruiseAcc, Constants.SliderSubsystem.kTimeoutMs);

		/* Zero the sensor once on robot boot up */
		sliderMotor.setSelectedSensorPosition(0, 0, Constants.SliderSubsystem.kTimeoutMs);
    
    
  }
}