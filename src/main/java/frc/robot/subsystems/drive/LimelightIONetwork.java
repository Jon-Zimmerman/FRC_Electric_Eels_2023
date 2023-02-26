package frc.robot.subsystems.drive;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

public class LimelightIONetwork implements LimelightIO{
    NetworkTable table;
    public LimelightIONetwork() {
        table = NetworkTableInstance.getDefault().getTable("limelight");
    }
    public void updateInputs(LimelightIOInputs inputs) {
        if (DriverStation.getAlliance() == Alliance.Red) {
            inputs.botPoseWPI = table.getEntry("botpose_wpired").getDoubleArray(new double[6]);
        }
        else if (DriverStation.getAlliance() == Alliance.Blue) {
            inputs.botPoseWPI = table.getEntry("botpose_wpiblue").getDoubleArray(new double[6]);
        }
        inputs.latency = inputs.botPoseWPI[6];
    }

}
