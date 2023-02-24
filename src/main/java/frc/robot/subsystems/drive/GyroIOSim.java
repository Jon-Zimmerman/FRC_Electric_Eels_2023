package frc.robot.subsystems.drive;

//import edu.wpi.first.math.geometry.Rotation2d;
//import frc.robot.Constants;
import java.util.function.DoubleSupplier;
import edu.wpi.first.math.MathUtil;
import frc.robot.Constants;

public class GyroIOSim implements GyroIO{
    private DoubleSupplier rotationSup;
    public GyroIOSim(DoubleSupplier rotation){
        
        this.rotationSup = rotation;
    }
    public void updateInputs(GyroIOInputs inputs) {
        double rotationVal = MathUtil.applyDeadband(rotationSup.getAsDouble(), Constants.stickDeadband);
        inputs.yawDegrees += rotationVal*Constants.Swerve.maxAngularVelocity*Constants.simLoopPeriodSecs*360.0/(2.0*Math.PI);
        //inputs.yaw+= rotationVal*Constants.Swerve.maxAngularVelocity*Constants.simLoopPeriodSecs;
    }
    public void zeroGyro(){
    }

    public void calibrateGyro() {
    }
}
