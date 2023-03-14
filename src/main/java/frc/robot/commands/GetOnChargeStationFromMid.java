package frc.robot.commands;

//import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.subsystems.drive.Swerve;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;

public class GetOnChargeStationFromMid extends CommandBase {

  private final Swerve m_swerve;
  private final Timer timecheck;
  private boolean onRamp;
  private boolean balanced;
  private boolean failed;
  private boolean parked;

  private double roll;
  private final double stopThresholdDegrees = 3.0;
  private final double initialTriggerDegrees = 9.0;
  private final double approachTimeLimit = 4.0;
  private final double balancingTimeLimit = 8.0;
  private final Translation2d stop = new Translation2d(0.0, 0.0); 
  private final Translation2d approachTranslation = new Translation2d(-1.0, 0); // meters per second;
  private final Translation2d balancingTranslation = new Translation2d(-0.3, 0); // meters per second;

  public GetOnChargeStationFromMid(Swerve swerve) {
    m_swerve = swerve;
    addRequirements(m_swerve);
    timecheck = new Timer();
  }

  // Called just before this Command runs the first time
  @Override
  public void initialize() {
    onRamp = false;
    balanced = false;
    failed = false;
    parked = false;
    timecheck.start();

  }

  @Override
  public void execute() {
    roll = -m_swerve.gyroInputs.rollDegrees;
    if(!failed){
      if (!onRamp ) {
        if (!(timecheck.hasElapsed(approachTimeLimit))) { // cascade logic to decrease calls to timer
          m_swerve.drive(approachTranslation, 0.0, false, true);
          if (roll > initialTriggerDegrees ) {
            onRamp = true;// set rampEngaged = true
            timecheck.reset();
          }
        }
      } else {
        if (!balanced) {
          if (!(timecheck.hasElapsed(balancingTimeLimit))) {
            if (roll > stopThresholdDegrees) {
              m_swerve.drive(balancingTranslation, 0.0, false,true);
            } else if (roll < -stopThresholdDegrees) {
              m_swerve.drive(balancingTranslation.times(-1.0), 0.0, false,true);
            } else {
              balanced = true;
            }
          }
        } else {
          // sit here till end of teleop bc we think we are balanced
          if ((roll < stopThresholdDegrees) && (roll > -stopThresholdDegrees)) {
            
            m_swerve.drive(stop, 0.0, false , true);
            parked = true;
          } else {
            balanced = false;
          }
        }
      }
    }
   
    // if(m_swerve.getPose().getX()>7.0 || m_swerve.getPose().getX()<2.0){
    //   failed = true;
    //   m_swerve.drive(stop, 0.0, false , false);
    // }
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  public boolean isFinished() {
    return parked;
  }
}
