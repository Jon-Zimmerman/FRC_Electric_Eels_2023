package frc.robot.commands;
//import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.elevator.Elevator;

public class ElevatorGoToPosition extends CommandBase {
    

  private final Elevator m_elevator;
  private final double m_setpoint;
  private final double goal_tolerance;
  /**
   * Create a new ElevatorGoToPosition command.
   *
   * @param setpoint The setpoint to set the elevator to
   * @param elevator The elevator to use
   */
  public ElevatorGoToPosition(double setpointInch, double goalTolerance, Elevator elevator) {
    m_elevator = elevator;
    m_setpoint = setpointInch;
    goal_tolerance = goalTolerance;
    addRequirements(m_elevator);
  }

  // Called just before this Command runs the first time
  @Override
  public void initialize() {
    m_elevator.setPositionSetPoint(m_setpoint);

  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  public boolean isFinished() {
    
    return m_elevator.atSetpoint(goal_tolerance);
  }
}