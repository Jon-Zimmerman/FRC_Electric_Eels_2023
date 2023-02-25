package frc.robot.subsystems.elevator;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import frc.robot.Constants;

public class ElevatorIOSim implements ElevatorIO {
  private FlywheelSim flywheelSim = new FlywheelSim(DCMotor.getNEO(1), 1.0, 0.00001);
  private PIDController pid = new PIDController(0.0, 0.0, 0.0);

  private boolean closedLoop = false;
  private double ffVolts = 0.0;
  private double appliedVolts = 0.0;

  private double positionSetPointInch = 2.0;
  
  private double positionInch = 0.0;
  @Override
  public void updateInputs(ElevatorIOInputs inputs) {
    if (closedLoop) {
      appliedVolts = MathUtil.clamp(
          pid.calculate(flywheelSim.getAngularVelocityRadPerSec()) + ffVolts, -12.0,
          12.0);
      flywheelSim.setInputVoltage(appliedVolts);
    }
    inputs.positionSetPointInch = positionSetPointInch;
    flywheelSim.update(0.02);
    //double elevator position
    inputs.positionRad += flywheelSim.getAngularVelocityRadPerSec() * Constants.simLoopPeriodSecs;

    inputs.positionInch += flywheelSim.getAngularVelocityRadPerSec() * Constants.simLoopPeriodSecs/ (2*Math.PI) *
    Constants.ElevatorSubsystem.sprocketDiameterInch*Math.PI/25;
    positionInch +=flywheelSim.getAngularVelocityRadPerSec() * Constants.simLoopPeriodSecs/ (2*Math.PI) *
    Constants.ElevatorSubsystem.sprocketDiameterInch*Math.PI/25;
    inputs.velocityRadPerSec = flywheelSim.getAngularVelocityRadPerSec();
    inputs.appliedVolts = appliedVolts;
    inputs.currentAmps = flywheelSim.getCurrentDrawAmps();

    double rotationsMeasurement = positionInch/(Constants.ElevatorSubsystem.sprocketDiameterInch*Math.PI)*Constants.ElevatorSubsystem.gearRatio;
    appliedVolts = MathUtil.clamp(
      pid.calculate(rotationsMeasurement), -12.0,
      12.0);
    flywheelSim.setInputVoltage(appliedVolts);
  }

  @Override
  public void setPosition(double positionSetInch, double ffVolts) {
    double setPointRotationsOutput = positionSetInch/(Constants.ElevatorSubsystem.sprocketDiameterInch*Math.PI)*Constants.ElevatorSubsystem.gearRatio;
    pid.setSetpoint(setPointRotationsOutput);
    positionSetPointInch = positionSetInch;

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
