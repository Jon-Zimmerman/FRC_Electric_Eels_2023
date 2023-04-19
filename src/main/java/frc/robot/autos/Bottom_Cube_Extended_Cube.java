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

public class Bottom_Cube_Extended_Cube extends SequentialCommandGroup {
    List<PathPlannerTrajectory> Path01= PathPlanner.loadPathGroup("Bottom_Ext_01", new PathConstraints(Constants.AutoConstants.kMaxSpeedMetersPerSecond, Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared));
    List<PathPlannerTrajectory> Path02= PathPlanner.loadPathGroup("Bottom_Extended_02", new PathConstraints(Constants.AutoConstants.kMaxSpeedMetersPerSecond, Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared));
    List<PathPlannerTrajectory> Path03= PathPlanner.loadPathGroup("Bottom_Extended_03", new PathConstraints(Constants.AutoConstants.kMaxSpeedMetersPerSecond, Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared));
    List<PathPlannerTrajectory> Path04= PathPlanner.loadPathGroup("Bottom_Extended_04", new PathConstraints(Constants.AutoConstants.kMaxSpeedMetersPerSecond, Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared));

    final Command Path01Command,Path02Command,Path03Command,Path04Command;

    public Bottom_Cube_Extended_Cube(Swerve s_Swerve, Intake intake, Elevator elevator,Slider slider){
        Path01Command = s_Swerve.swerveAutoBuilder.fullAuto (Path01);
        Path02Command = s_Swerve.swerveAutoBuilder.fullAuto (Path02);
        Path03Command = s_Swerve.swerveAutoBuilder.fullAuto (Path03);
        Path04Command = s_Swerve.swerveAutoBuilder.fullAuto (Path04);
        
        
        
        addCommands(
            // place first Cube
        new InstantCommand(() -> intake.setIntakeModeCone()),
        new InstantCommand(() ->  intake.intakeIn()),
        s_Swerve.swerveAutoBuilder.resetPose(Path01.get(0)),
        new ElevatorGoToPosition(Constants.ElevatorSubsystem.elevatorPosTop,7.0,elevator).withTimeout(3.0),
        new SliderGoToPosition(Constants.SliderSubsystem.sliderOut,1.5,slider).withTimeout(3.0),
        new StartEndCommand(() ->  intake.intakeOut(),intake::stop,intake).withTimeout(0.3), //make time based
        new SliderGoToPosition(Constants.SliderSubsystem.sliderIn,5.0,slider).withTimeout(3.0),
        new ElevatorGoToPosition(Constants.ElevatorSubsystem.elevatorPosBottom,6.0,elevator).withTimeout(3.0),
        Path01Command,

        // pickup cube
        new SliderGoToPosition(Constants.SliderSubsystem.sliderOut,0.5,slider).withTimeout(3.0),

        new InstantCommand(() ->  intake.intakeIn()), //make time based
        Path02Command, // start the second path so that we move and run intake
        new InstantCommand(() ->  intake.holdCurrent()),
        new SliderGoToPosition(Constants.SliderSubsystem.sliderIn,5.0,slider).withTimeout(3.0),
        // turn around
        Path03Command,
        //extend elevator
        new ElevatorGoToPosition(Constants.ElevatorSubsystem.elevatorPosTop,5.0,elevator).withTimeout(3.0),
        new SliderGoToPosition(Constants.SliderSubsystem.sliderOut,0.5,slider).withTimeout(3.0),
        //place second cube on mid
        Path04Command,
        new StartEndCommand(() ->  intake.intakeOut(),intake::stop,intake).withTimeout(0.5), //make time based
        new SliderGoToPosition(Constants.SliderSubsystem.sliderIn,5.0,slider).withTimeout(3.0)
        //new ElevatorGoToPosition(Constants.ElevatorSubsystem.elevatorPosBottom,6.0,elevator).withTimeout(3.0)
        // ready for teleop
        );
    }

}