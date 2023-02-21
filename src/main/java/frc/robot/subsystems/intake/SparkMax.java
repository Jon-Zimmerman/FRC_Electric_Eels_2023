package frc.robot.subsystems.intake;

//import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;

import org.littletonrobotics.junction.LoggedRobot;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class SparkMax extends LoggedRobot {
  
  public CANSparkMax extend_motor, rotate__motor,grab_motor;
  public SparkMaxPIDController em_pidController,rm_pidController,gm_pidController;
  public RelativeEncoder em_encoder,rm_encoder,gm_encoder;
  
  
  
//  private boolean elevatorState =false;
  //@Override
  public void maxInit() {
    // initialize motor
    extend_motor = new CANSparkMax(Constants.Actuators.Neo0.deviceID, MotorType.kBrushless);
    rotate__motor= new CANSparkMax(Constants.Actuators.Neo1.deviceID, MotorType.kBrushless);
    grab_motor= new CANSparkMax(Constants.Actuators.Neo2.deviceID, MotorType.kBrushless);
    /**
     * The restoreFactoryDefaults method can be used to reset the configuration parameters
     * in the SPARK MAX to their factory default state. If no argument is passed, these
     * parameters will not persist between power cycles
     */
    extend_motor.restoreFactoryDefaults();
    rotate__motor.restoreFactoryDefaults();
    grab_motor.restoreFactoryDefaults();
    /**
     * In order to use PID functionality for a controller, a SparkMaxPIDController object
     * is constructed by calling the getPIDController() method on an existing
     * CANSparkMax object
     */
    em_pidController = extend_motor.getPIDController();
    rm_pidController = rotate__motor.getPIDController();
    gm_pidController = grab_motor.getPIDController();
    // Encoder object created to display position values
    em_encoder = extend_motor.getEncoder();
    rm_encoder = rotate__motor.getEncoder();
    gm_encoder = grab_motor.getEncoder();
    

    // set PID coefficients
    em_pidController.setP(Constants.Actuators.Neo0.kP);
    em_pidController.setI(Constants.Actuators.Neo0.kI);
    em_pidController.setD(Constants.Actuators.Neo0.kD);
    em_pidController.setIZone(Constants.Actuators.Neo0.kIz);
    em_pidController.setFF(Constants.Actuators.Neo0.kFF);
    em_pidController.setOutputRange(Constants.Actuators.Neo0.kMinOutput, Constants.Actuators.Neo0.kMaxOutput);

    rm_pidController.setP(Constants.Actuators.Neo1.kP);
    rm_pidController.setI(Constants.Actuators.Neo1.kI);
    rm_pidController.setD(Constants.Actuators.Neo1.kD);
    rm_pidController.setIZone(Constants.Actuators.Neo1.kIz);
    rm_pidController.setFF(Constants.Actuators.Neo1.kFF);
    rm_pidController.setOutputRange(Constants.Actuators.Neo1.kMinOutput, Constants.Actuators.Neo1.kMaxOutput);

    gm_pidController.setP(Constants.Actuators.Neo2.kP);
    gm_pidController.setI(Constants.Actuators.Neo2.kI);
    gm_pidController.setD(Constants.Actuators.Neo2.kD);
    gm_pidController.setIZone(Constants.Actuators.Neo2.kIz);
    gm_pidController.setFF(Constants.Actuators.Neo2.kFF);
    gm_pidController.setOutputRange(Constants.Actuators.Neo2.kMinOutput, Constants.Actuators.Neo2.kMaxOutput);

    // // display PID coefficients on SmartDashboard
    // SmartDashboard.putNumber("P Gain", kP);
    // SmartDashboard.putNumber("I Gain", kI);
    // SmartDashboard.putNumber("D Gain", kD);
    // SmartDashboard.putNumber("I Zone", kIz);
    // SmartDashboard.putNumber("Feed Forward", kFF);
    // SmartDashboard.putNumber("Max Output", kMaxOutput);
    // SmartDashboard.putNumber("Min Output", kMinOutput);
    // SmartDashboard.putNumber("Set Rotations", 0);
  }

  //@Override
  // public void teleopMax() {
  //   // read PID coefficients from SmartDashboard
  //   // double p = SmartDashboard.getNumber("P Gain", 0);
  //   // double i = SmartDashboard.getNumber("I Gain", 0);
  //   // double d = SmartDashboard.getNumber("D Gain", 0);
  //   // double iz = SmartDashboard.getNumber("I Zone", 0);
  //   // double ff = SmartDashboard.getNumber("Feed Forward", 0);
  //   // double max = SmartDashboard.getNumber("Max Output", 0);
  //   // double min = SmartDashboard.getNumber("Min Output", 0);
  //    double rotations = SmartDashboard.getNumber("Set Rotations", 0);

  //   // // if PID coefficients on SmartDashboard have changed, write new values to controller
  //   // if((p != kP)) { m_pidController.setP(p); kP = p; }
  //   // if((i != kI)) { m_pidController.setI(i); kI = i; }
  //   // if((d != kD)) { m_pidController.setD(d); kD = d; }
  //   // if((iz != kIz)) { m_pidController.setIZone(iz); kIz = iz; }
  //   // if((ff != kFF)) { m_pidController.setFF(ff); kFF = ff; }
  //   // if((max != kMaxOutput) || (min != kMinOutput)) { 
  //   //   m_pidController.setOutputRange(min, max); 
  //   //   kMinOutput = min; kMaxOutput = max; 
  //    // }

  //   /**
  //    * PIDController objects are commanded to a set point using the 
  //    * SetReference() method.
  //    * 
  //    * The first parameter is the value of the set point, whose units vary
  //    * depending on the control type set in the second parameter.
  //    * 
  //    * The second parameter is the control type can be set to one of four 
  //    * parameters:
  //    *  com.revrobotics.CANSparkMax.ControlType.kDutyCycle
  //    *  com.revrobotics.CANSparkMax.ControlType.kPosition
  //    *  com.revrobotics.CANSparkMax.ControlType.kVelocity
  //    *  com.revrobotics.CANSparkMax.ControlType.kVoltage
  //    */


  //   m_pidController.setReference(rotations, CANSparkMax.ControlType.kPosition);
    
  //   SmartDashboard.putNumber("SetPoint", rotations);
  //   SmartDashboard.putNumber("ProcessVariable", m_encoder.getPosition());
  // }

  //Toggle elevator from bottom to top
  
    public void extend(double pos){
      em_pidController.setReference(pos, CANSparkMax.ControlType.kPosition);
    }
    // public double square_a_number(double input){
    //   return input*input;
    // }


    // double out = square_a_number(2);

    
    
}