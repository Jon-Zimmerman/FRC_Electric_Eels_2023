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

import frc.robot.Constants;
import frc.robot.commands.GetOnChargeStation;

import java.util.HashMap;
//import frc.robot.commands.GoToElevatorTop;



public class One_Cone_And_Balance_Chassis extends SequentialCommandGroup {
    List<PathPlannerTrajectory> pathGroup = PathPlanner.loadPathGroup("Path3", new PathConstraints(Constants.AutoConstants.kMaxSpeedMetersPerSecond, Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared));
    final HashMap<String, Command> eventMap = new HashMap<String, Command>();
    final Command followPath1;
    public One_Cone_And_Balance_Chassis(Swerve s_Swerve){
        //Could try and use this method which allows you to put points in pathplanner GUI,
        //But not really any easier because you can't add them before the path starts.
        //eventMap.put("ElevatorTop", new InstantCommand(() -> elevator.elevatorTop()));
        //OR
        // eventMap.put("ElevatorTop", new GoToEleatorTop());
        //But I have no idea why you would want to do this
        //eventMap.put("intakeIn", new InstantCommand(() -> intake.intakeIn()));
        

        followPath1 = s_Swerve.swerveAutoBuilder.fullAuto (pathGroup);

        //s_Swerve.resetPose(
        
        addCommands(
        s_Swerve.swerveAutoBuilder.resetPose(pathGroup.get(0)),
        followPath1,
        new GetOnChargeStation(s_Swerve)
        );
    }

}