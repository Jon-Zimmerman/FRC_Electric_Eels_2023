package frc.robot.subsystems.slider;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import frc.robot.Constants;

public class SliderIOSim implements SliderIO {
  private FlywheelSim flywheelSim = new FlywheelSim(DCMotor.getNEO(1), 1.0, 0.00004);
  private PIDController pid = new PIDController(0.0, 0.0, 0.0);

  private boolean closedLoop = false;
  private double ffVolts = 0.0;
  private double appliedVolts = 0.0;
  private double positionRotations = 0.0;
  private double positionSetPointInch = 0.0;
  @Override
  public void updateInputs(SliderIOInputs inputs) {
    inputs.appliedVolts = appliedVolts;
    if (closedLoop) {
      //appliedVolts = MathUtil.clamp(
      //    pid.calculate(flywheelSim.getAngularVelocityRadPerSec()) + ffVolts, -12.0,
      //    12.0);
      //flywheelSim.setInputVoltage(appliedVolts);
    }
    inputs.positionSetPointInch = positionSetPointInch;
    flywheelSim.update(0.02);
    //double elevator position
    inputs.positionRad += flywheelSim.getAngularVelocityRadPerSec() * Constants.simLoopPeriodSecs;
    inputs.positionInch += flywheelSim.getAngularVelocityRadPerSec() * Constants.simLoopPeriodSecs*
    Constants.ElevatorSubsystem.sprocketDiameterInch/(2*Math.PI);
    inputs.velocityRadPerSec = flywheelSim.getAngularVelocityRadPerSec();
    inputs.appliedVolts = appliedVolts;
    inputs.currentAmps = flywheelSim.getCurrentDrawAmps();

  }

  @Override
  public void setPosition(double positionInch, double ffVolts) {
    pid.setSetpoint(positionInch/(Constants.ElevatorSubsystem.sprocketDiameterInch*Math.PI));
    double setPointRotations = positionInch / (Math.PI * Constants.ElevatorSubsystem.sprocketDiameterInch) * Constants.ElevatorSubsystem.gearRatio;
    positionSetPointInch = positionInch;
    appliedVolts = MathUtil.clamp(
      pid.calculate(setPointRotations), -12.0,
      12.0);
    flywheelSim.setInputVoltage(appliedVolts);
  }



  @Override
  public void stop() {
    closedLoop = false;
    appliedVolts = 0.0;
    flywheelSim.setInputVoltage(0.0);
  }

  public void configurePID(double kP, double kI, double kD) {
    pid.setPID(kP, kI, kD);
  }
}

