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

public class Cone_Speedy_Meatball extends SequentialCommandGroup {
    List<PathPlannerTrajectory> SpicyMB_Drive_To_Top_Cone= PathPlanner.loadPathGroup("SpicyMB_Drive_To_Top_Cone", new PathConstraints(Constants.AutoConstants.kMaxSpeedMetersPerSecond, Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared));
    List<PathPlannerTrajectory> SpicyMB_Pickup_Cone= PathPlanner.loadPathGroup("SpicyMB_Pickup_Cone", new PathConstraints(Constants.AutoConstants.kMaxSpeedMetersPerSecond, Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared));
    List<PathPlannerTrajectory> SpicyMB_Drive_To_Grid= PathPlanner.loadPathGroup("SpicyMB_Drive_To_Grid", new PathConstraints(Constants.AutoConstants.kMaxSpeedMetersPerSecond, Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared));
    //List<PathPlannerTrajectory> Path04= PathPlanner.loadPathGroup("Top_Extended_04", new PathConstraints(Constants.AutoConstants.kMaxSpeedMetersPerSecond, Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared));

    final Command SpicyMBDriveToTopConeCommand,SpicyMBPickupConeCommand,SpicyMBDriveToGridCommand;

    public Cone_Speedy_Meatball(Swerve s_Swerve, Intake intake, Elevator elevator,Slider slider){
        SpicyMBDriveToTopConeCommand = s_Swerve.swerveAutoBuilder.fullAuto (SpicyMB_Drive_To_Top_Cone);
        SpicyMBPickupConeCommand = s_Swerve.swerveAutoBuilder.fullAuto (SpicyMB_Pickup_Cone);
        SpicyMBDriveToGridCommand = s_Swerve.swerveAutoBuilder.fullAuto (SpicyMB_Drive_To_Grid);
        
        
        
        addCommands(
            // place first Cube
        new InstantCommand(() -> intake.setIntakeModeCone()),
        s_Swerve.swerveAutoBuilder.resetPose(SpicyMB_Drive_To_Top_Cone.get(0)),
        new InstantCommand(() ->  intake.intakeIn()),
        new ElevatorGoToPosition(Constants.ElevatorSubsystem.elevatorPosTop,7.0,elevator).withTimeout(3.0),
        new SliderGoToPosition(Constants.SliderSubsystem.sliderOut,1.5,slider).withTimeout(3.0),
        new StartEndCommand(() ->  intake.intakeOut(),intake::stop,intake).withTimeout(0.4), //make time based
        new SliderGoToPosition(Constants.SliderSubsystem.sliderIn,10.0,slider).withTimeout(3.0),
        new ElevatorGoToPosition(Constants.ElevatorSubsystem.elevatorPosBottom,13.0,elevator).withTimeout(3.0),
        SpicyMBDriveToTopConeCommand,

        new InstantCommand(() -> intake.setIntakeModeCube()),
        // pickup second cone
        new InstantCommand(() ->  intake.intakeIn()),
        new SliderGoToPosition(Constants.SliderSubsystem.sliderOut,2.0,slider).withTimeout(3.0), 
         //make time based
        SpicyMBPickupConeCommand, // start the second path so that we move and run intake
        new InstantCommand(() ->  intake.holdCurrent()),
        new SliderGoToPosition(Constants.SliderSubsystem.sliderIn,3.0,slider).withTimeout(1.0),

        SpicyMBDriveToGridCommand,


        //extend elevator
        new ElevatorGoToPosition(Constants.ElevatorSubsystem.elevatorPosTop,5.0,elevator).withTimeout(3.0),
        new SliderGoToPosition(Constants.SliderSubsystem.sliderOut,0.5,slider).withTimeout(3.0)
        //new StartEndCommand(() ->  intake.intakeOut(),intake::stop,intake).withTimeout(0.5),
        //new SliderGoToPosition(Constants.SliderSubsystem.sliderIn,0.5,slider).withTimeout(3.0)

        // ready for teleop
        );
    }

}