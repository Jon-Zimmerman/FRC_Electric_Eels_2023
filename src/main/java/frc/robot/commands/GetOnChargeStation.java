package frc.robot.commands;
//import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.subsystems.drive.Swerve;

public class GetOnChargeStation extends CommandBase {
    
  private final Swerve m_swerve;
  public GetOnChargeStation(Swerve swerve) {
    m_swerve = swerve;
    addRequirements(m_swerve);
  }

  // Called just before this Command runs the first time
  @Override
  public void initialize() {
    m_swerve.driveOntoChargeStation();

  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  public boolean isFinished() {
    return true;
  }
}


