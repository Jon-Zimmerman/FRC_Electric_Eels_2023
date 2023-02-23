package frc.robot.autos;

import frc.robot.Constants;
import frc.robot.subsystems.drive.Swerve;


import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

import frc.robot.subsystems.intake.Intake;

import frc.robot.subsystems.elevator.Elevator;

import frc.robot.subsystems.slider.Slider;

public class exampleAuto2 extends SequentialCommandGroup {
    List<PathPlannerTrajectory> pathGroup = PathPlanner.loadPathGroup("Path1", new PathConstraints(4, 3));
    
    public exampleAuto2(Swerve s_Swerve, Intake intake, Elevator elevator,Slider slider){
        addCommands(
            new InstantCommand(() -> s_Swerve.swerveAutoBuilder.fullAuto (pathGroup))
        );
    }
}