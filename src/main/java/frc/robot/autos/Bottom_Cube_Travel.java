package frc.robot.autos;

import frc.robot.subsystems.drive.Swerve;

import java.util.List;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.StartEndCommand;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants;
import frc.robot.commands.SliderGoToPosition;
import frc.robot.commands.ElevatorGoToPosition;


import frc.robot.subsystems.intake.Intake;

import frc.robot.subsystems.elevator.Elevator;

import frc.robot.subsystems.slider.Slider;

public class Bottom_Cube_Travel extends SequentialCommandGroup {
    List<PathPlannerTrajectory> bottomPath= PathPlanner.loadPathGroup("Bottom_Path", new PathConstraints(Constants.AutoConstants.kMaxSpeedMetersPerSecond, Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared));
    
    final Command followBottomPathCommand;

    public Bottom_Cube_Travel(Swerve s_Swerve, Intake intake, Elevator elevator,Slider slider){
        followBottomPathCommand = s_Swerve.swerveAutoBuilder.fullAuto (bottomPath);
        
        
        addCommands(
        new InstantCommand(() -> intake.setIntakeModeCube()),
        s_Swerve.swerveAutoBuilder.resetPose(bottomPath.get(0)),
        new ElevatorGoToPosition(Constants.ElevatorSubsystem.elevatorPosTop,5.0,elevator).withTimeout(3.0),
        new SliderGoToPosition(Constants.SliderSubsystem.sliderOut,0.5,slider).withTimeout(3.0),
        new StartEndCommand(() ->  intake.intakeOut(),intake::stop,intake).withTimeout(0.5), //make time based
        new SliderGoToPosition(Constants.SliderSubsystem.sliderIn,5.0,slider).withTimeout(3.0),
        new ElevatorGoToPosition(Constants.ElevatorSubsystem.elevatorPosBottom,6.0,elevator).withTimeout(3.0),
        followBottomPathCommand
        
        );
    }

}