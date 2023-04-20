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

public class TopConeGambit extends SequentialCommandGroup {
    List<PathPlannerTrajectory> SpicyMB_Drive_To_Top_Cube= PathPlanner.loadPathGroup("Speedy Meatball Step 1", new PathConstraints(Constants.AutoConstants.kMaxSpeedMetersPerSecond, Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared));
    List<PathPlannerTrajectory> SpicyMB_Pickup_Cube= PathPlanner.loadPathGroup("Speedy Meatball Step 2", new PathConstraints(Constants.AutoConstants.kMaxSpeedMetersPerSecond, Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared));
    List<PathPlannerTrajectory> SpicyMB_Drive_To_Grid= PathPlanner.loadPathGroup("Speedy Meatball Step 3", new PathConstraints(Constants.AutoConstants.kMaxSpeedMetersPerSecond, Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared));
    List<PathPlannerTrajectory> PlaceCubePath= PathPlanner.loadPathGroup("Speedy Meatball Step 4", new PathConstraints(Constants.AutoConstants.kMaxSpeedMetersPerSecond, Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared));
    //List<PathPlannerTrajectory> Path04= PathPlanner.loadPathGroup("Top_Extended_04", new PathConstraints(Constants.AutoConstants.kMaxSpeedMetersPerSecond, Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared));

    final Command DriveToTopCubeCommand,PickupCubeCommand,DriveToGridCommand, PlaceCubeCommand;

    public TopConeGambit(Swerve s_Swerve, Intake intake, Elevator elevator,Slider slider){
        DriveToTopCubeCommand = s_Swerve.swerveAutoBuilder.fullAuto (SpicyMB_Drive_To_Top_Cube);
        PickupCubeCommand = s_Swerve.swerveAutoBuilder.fullAuto (SpicyMB_Pickup_Cube);
        DriveToGridCommand = s_Swerve.swerveAutoBuilder.fullAuto (SpicyMB_Drive_To_Grid);
        PlaceCubeCommand= s_Swerve.swerveAutoBuilder.fullAuto (PlaceCubePath);
        
        
        addCommands(
            // place first Cone
        new InstantCommand(() -> intake.setIntakeModeCone()),
        s_Swerve.swerveAutoBuilder.resetPose(SpicyMB_Drive_To_Top_Cube.get(0)),
        new InstantCommand(() ->  intake.intakeIn()), // keep cone in
        new ElevatorGoToPosition(Constants.ElevatorSubsystem.elevatorPosTop,15.0,elevator).withTimeout(3.0),
        new SliderGoToPosition(Constants.SliderSubsystem.sliderOut,1.5,slider).withTimeout(3.0),
        new StartEndCommand(() ->  intake.intakeOut(),intake::stop,intake).withTimeout(0.3), //make time based
        new SliderGoToPosition(Constants.SliderSubsystem.sliderIn,10.0,slider).withTimeout(3.0),
        new ElevatorGoToPosition(Constants.ElevatorSubsystem.elevatorPosBottom,50.0,elevator).withTimeout(3.0),
        ////Drive to Cube
        DriveToTopCubeCommand,

        new InstantCommand(() -> intake.setIntakeModeCube()),
        // pickup cube
        new InstantCommand(() ->  intake.intakeIn()),
        new SliderGoToPosition(Constants.SliderSubsystem.sliderOut,2.0,slider).withTimeout(3.0), 
         //make time based
        PickupCubeCommand, // start the second path so that we move and run intake
        new InstantCommand(() ->  intake.holdCurrent()),
        new SliderGoToPosition(Constants.SliderSubsystem.sliderIn,7.0,slider).withTimeout(1.0),

        DriveToGridCommand,


        //extend elevator
        new ElevatorGoToPosition(Constants.ElevatorSubsystem.elevatorPosTop,60.0,elevator).withTimeout(3.0),
        new SliderGoToPosition(Constants.SliderSubsystem.sliderOut,7,slider).withTimeout(3.0),
        PlaceCubeCommand
        //new StartEndCommand(() ->  intake.intakeOut(),intake::stop,intake).withTimeout(0.3),
        //new SliderGoToPosition(Constants.SliderSubsystem.sliderIn,0.5,slider).withTimeout(3.0)

        // ready for teleop
        );
    }

}