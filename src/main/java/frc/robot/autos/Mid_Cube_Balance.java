package frc.robot.autos;

import frc.robot.subsystems.drive.GyroIOInputsAutoLogged;
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
//import frc.robot.commands.GetOnChargeStationFromGrid;
import frc.robot.commands.GetOnChargeStationFromGrid;

//import java.util.HashMap;
//import frc.robot.commands.GoToElevatorTop;

import frc.robot.subsystems.intake.Intake;

import frc.robot.subsystems.elevator.Elevator;

import frc.robot.subsystems.slider.Slider;

public class Mid_Cube_Balance extends SequentialCommandGroup {
    List<PathPlannerTrajectory> backOffPath = PathPlanner.loadPathGroup("Back_Off_Grid_Bal", new PathConstraints(Constants.AutoConstants.kMaxSpeedMetersPerSecond, Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared));
    List<PathPlannerTrajectory> balancePath= PathPlanner.loadPathGroup("Optimal_Balance", new PathConstraints(Constants.AutoConstants.kMaxSpeedMetersPerSecond, Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared));
    //final HashMap<String, Command> eventMap = new HashMap<String, Command>();
    final Command backOffGridCommand;
    final Command balanceCommand;
    public final GyroIOInputsAutoLogged gyroInputs = new GyroIOInputsAutoLogged();
    public Mid_Cube_Balance(Swerve s_Swerve, Intake intake, Elevator elevator,Slider slider){
   
        backOffGridCommand = s_Swerve.swerveAutoBuilder.fullAuto (backOffPath);
        balanceCommand = s_Swerve.swerveAutoBuilder.fullAuto (balancePath);
        //s_Swerve.resetPose(
        
        addCommands(
        new InstantCommand(() -> intake.setIntakeModeCone()),
        s_Swerve.swerveAutoBuilder.resetPose(backOffPath.get(0)),
        new ElevatorGoToPosition(Constants.ElevatorSubsystem.elevatorPosTop,5.0,elevator).withTimeout(3.0),
        new SliderGoToPosition(Constants.SliderSubsystem.sliderOut,0.5,slider).withTimeout(3.0),
        new StartEndCommand(() ->  intake.intakeOut(),intake::stop,intake).withTimeout(0.75), //make time based
        backOffGridCommand,
        new SliderGoToPosition(Constants.SliderSubsystem.sliderIn,5.0,slider).withTimeout(3.0),
        ////followMidPath,   
        new ElevatorGoToPosition(Constants.ElevatorSubsystem.elevatorPosBottom,6.0,elevator).withTimeout(3.0),
        balanceCommand,
        new GetOnChargeStationFromGrid(s_Swerve)
        );
        
    }

}