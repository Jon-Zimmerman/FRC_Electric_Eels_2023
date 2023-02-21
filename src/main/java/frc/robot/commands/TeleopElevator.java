package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.subsystems.intake.SparkMax;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxAbsoluteEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax;

public class TeleopElevator extends CommandBase {
  private SparkMax sMax;
  private DoubleSupplier position;
  private BooleanSupplier elevatorpos1;

  public TeleopElevator(SparkMax sMax, BooleanSupplier elevatorpos1) {
    this.sMax = sMax;
    this.position = position;
    this.elevatorpos1 = elevatorpos1;
    // addRequirements(sMax);

  }

  public void execute() {
    // Get initial postion top/bottom
    // double current_rotations = SparkMax.m_encoder.getPosition();
    double pos = position.getAsDouble();
    boolean elevatorpos1bool = elevatorpos1.getAsBoolean();
    // todo check that request does not exceed max or min

    // if (current_rotations == 0) {
    // m_pidController.setReference(max_rotations,
    // CANSparkMax.ControlType.kPosition);
    // }
    // else if (current_rotations == 5) {
    // m_pidController.setReference(min_rotations,
    // CANSparkMax.ControlType.kPosition);
    // }
    // double position = position.getAsDouble();
    // if (elevatorpos1bool)
    // sMax.em_pidController.setReference(pos, CANSparkMax.ControlType.kPosition);

  }

}