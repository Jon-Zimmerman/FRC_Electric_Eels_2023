package frc.robot.autos;

import frc.robot.subsystems.drive.Swerve;

import java.util.List;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;


import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.Constants;

public class Test_Odometry extends SequentialCommandGroup {
    List<PathPlannerTrajectory> backOffPath = PathPlanner.loadPathGroup("1_Meter_Drive", new PathConstraints(Constants.AutoConstants.kMaxSpeedMetersPerSecond, Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared));
    final Command drivedistance;
    public Test_Odometry(Swerve s_Swerve) {
        drivedistance = s_Swerve.swerveAutoBuilder.fullAuto (backOffPath);
        addCommands(drivedistance);
    }
    
}
