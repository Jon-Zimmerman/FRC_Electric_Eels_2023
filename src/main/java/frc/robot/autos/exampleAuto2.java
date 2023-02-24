package frc.robot.autos;

import frc.robot.subsystems.drive.Swerve;

import java.util.List;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.StartEndCommand;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import edu.wpi.first.wpilibj2.command.Command;


import java.util.HashMap;
//import frc.robot.commands.GoToElevatorTop;

import frc.robot.subsystems.intake.Intake;

import frc.robot.subsystems.elevator.Elevator;

import frc.robot.subsystems.slider.Slider;

public class exampleAuto2 extends SequentialCommandGroup {
    List<PathPlannerTrajectory> pathGroup = PathPlanner.loadPathGroup("Path1", new PathConstraints(3, 1));
    final HashMap<String, Command> eventMap = new HashMap<String, Command>();
    final Command followPath1;
    public exampleAuto2(Swerve s_Swerve, Intake intake, Elevator elevator,Slider slider){
        //eventMap.put("ElevatorTop", new InstantCommand(() -> elevator.elevatorTop()));
        //OR
        // eventMap.put("ElevatorTop", new GoToEleatorTop());
        //But I have no idea why you would want to do this
        //eventMap.put("intakeIn", new InstantCommand(() -> intake.intakeIn()));
        followPath1 = s_Swerve.swerveAutoBuilder.fullAuto (pathGroup);

        addCommands(
        new InstantCommand(() -> elevator.elevatorTop(), elevator).withTimeout(1.0),
        new StartEndCommand(() -> slider.sliderOut(), () ->  intake.intakeOut(),  slider,intake).withTimeout(1.0),
        new StartEndCommand(() -> slider.sliderIn(), () ->  elevator.elevatorBottom(),  slider,elevator).withTimeout(1.0),
        followPath1);
    }
    // public Command getCommand(){
    //     return fullAuto;
    // }
}