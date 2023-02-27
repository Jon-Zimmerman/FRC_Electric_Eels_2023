package frc.robot.commands;
//import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.slider.Slider;

public class SliderGoToPosition extends CommandBase {
    

  private final Slider m_slider;
  private final double m_setpoint;

  /**
   * Create a new SliderGoToPosition command.
   *
   * @param setpoint The setpoint to set the slider to
   * @param elevator The slider to use
   */
  public SliderGoToPosition(double setpointInch, Slider slider) {
    m_slider = slider;
    m_setpoint = setpointInch;
    addRequirements(m_slider);
  }

  // Called just before this Command runs the first time
  @Override
  public void initialize() {
    m_slider.setPositionSetPoint(m_setpoint);

  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  public boolean isFinished() {
    
    return m_slider.atSetpoint();
  }
}