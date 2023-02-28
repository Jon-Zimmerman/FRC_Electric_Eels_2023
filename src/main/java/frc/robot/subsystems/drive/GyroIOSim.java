package frc.robot.subsystems.drive;

//import edu.wpi.first.math.geometry.Rotation2d;
//import frc.robot.Constants;
import java.util.function.DoubleSupplier;
import edu.wpi.first.math.MathUtil;
import frc.robot.Constants;

public class GyroIOSim implements GyroIO{
    private DoubleSupplier rotationSup;
    private double yawDegrees;
    public GyroIOSim(DoubleSupplier rotation){
        
        this.rotationSup = rotation;
    }
    public void updateInputs(GyroIOInputs inputs) {
        double rotationVal = MathUtil.applyDeadband(rotationSup.getAsDouble(), Constants.rotationStickDeadband);
        yawDegrees += rotationVal*Constants.Swerve.maxAngularVelocity*Constants.simLoopPeriodSecs*360.0/(2.0*Math.PI);
        double yawDegreesMod = yawDegrees%360.0;
        if(yawDegreesMod>0.0){
            if(yawDegreesMod <180.0){
                yawDegrees = yawDegreesMod;
            }
            if(yawDegreesMod >180.0){
                yawDegrees = yawDegreesMod-360.0;
            }
        }
        if(yawDegreesMod<0.0){
            if(yawDegreesMod <-180.0){
                yawDegrees = yawDegreesMod+360.0;
                
            }
            if(yawDegreesMod >-180.0){
                yawDegrees = yawDegreesMod;
            }
        }
        inputs.yawDegrees = yawDegrees;
        //inputs.yaw+= rotationVal*Constants.Swerve.maxAngularVelocity*Constants.simLoopPeriodSecs;
    }
    public void zeroGyro(){
    }

    public void calibrateGyro() {
    }
    public void additionalRotation(double rotation) {
        yawDegrees += rotation*Constants.Swerve.maxAngularVelocity*Constants.simLoopPeriodSecs*360.0/(2.0*Math.PI);;
    }
}
