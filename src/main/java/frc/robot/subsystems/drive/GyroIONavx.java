package frc.robot.subsystems.drive;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SerialPort;
import frc.robot.Constants;

public class GyroIONavx implements GyroIO{
    AHRS ahrs;
    
    public GyroIONavx () {
        ahrs = new AHRS(SerialPort.Port.kUSB);//edu.wpi.first.wpilibj.SerialPort.Port
        calibrateGyro();
        zeroGyro();
    }
    public void updateInputs(GyroIOInputs inputs) {
        inputs.yawDegrees = (Constants.Swerve.invertGyro) ? 360.0 - ahrs.getYaw() : ahrs.getYaw();
        inputs.rollDegrees = ahrs.getRoll();
        inputs.pitchDegrees = ahrs.getPitch();
        //inputs.yaw = (Constants.Swerve.invertGyro) ? 360.0 - ahrs.getYaw() : ahrs.getYaw();
        //inputs.connected = get status
    }
    public void zeroGyro(){
        ahrs.zeroYaw();
    }

    public void calibrateGyro() {
        ahrs.calibrate();
    }
}


