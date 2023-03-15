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
import frc.robot.commands.GetOnChargeStationFromGrid;

//import java.util.HashMap;
//import frc.robot.commands.GoToElevatorTop;

import frc.robot.subsystems.intake.Intake;

import frc.robot.subsystems.elevator.Elevator;

import frc.robot.subsystems.slider.Slider;

public class One_Cone_Mid_Path_Balance extends SequentialCommandGroup {
    List<PathPlannerTrajectory> pathGroup = PathPlanner.loadPathGroup("Mid_Path", new PathConstraints(Constants.AutoConstants.kMaxSpeedMetersPerSecond, Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared));
    //final HashMap<String, Command> eventMap = new HashMap<String, Command>();
    final Command followMidPath;
    public One_Cone_Mid_Path_Balance(Swerve s_Swerve, Intake intake, Elevator elevator,Slider slider){
        //Could try and use this method which allows you to put points in pathplanner GUI,
        //But not really any easier because you can't add them before the path starts.
        //eventMap.put("ElevatorTop", new InstantCommand(() -> elevator.elevatorTop()));
        //OR
        // eventMap.put("ElevatorTop", new GoToEleatorTop());
        //But I have no idea why you would want to do this
        //eventMap.put("intakeIn", new InstantCommand(() -> intake.intakeIn()));
        

        followMidPath = s_Swerve.swerveAutoBuilder.fullAuto (pathGroup);

        //s_Swerve.resetPose(
        
        addCommands(
        new InstantCommand(() -> intake.setIntakeModeCube()),
        s_Swerve.swerveAutoBuilder.resetPose(pathGroup.get(0)),
        new ElevatorGoToPosition(Constants.ElevatorSubsystem.elevatorPosTop,5.0,elevator).withTimeout(3.0),
        new SliderGoToPosition(Constants.SliderSubsystem.sliderOut,0.5,slider).withTimeout(3.0),
        new StartEndCommand(() ->  intake.intakeOut(),intake::stop,intake).withTimeout(1.5), //make time based

        new SliderGoToPosition(Constants.SliderSubsystem.sliderIn,5.0,slider).withTimeout(3.0),
        followMidPath,   
        new ElevatorGoToPosition(Constants.ElevatorSubsystem.elevatorPosBottom,6.0,elevator).withTimeout(3.0),
  
        new GetOnChargeStationFromGrid(s_Swerve)
        );
    }

}