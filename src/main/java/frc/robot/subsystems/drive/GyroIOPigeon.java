package frc.robot.subsystems.drive;
import com.ctre.phoenix.sensors.Pigeon2;
import frc.robot.Constants;

public class GyroIOPigeon implements GyroIO{
    public Pigeon2 gyro;
    
    public GyroIOPigeon () {
        gyro = new Pigeon2(Constants.Swerve.pigeonID,"Drivetrain");
        //ahrs = new AHRS(SerialPort.Port.kUSB);//edu.wpi.first.wpilibj.SerialPort.Port
        calibrateGyro();
        zeroGyro();
    }
    public void updateInputs(GyroIOInputs inputs) {
        //inputs.yawDegrees = (Constants.Swerve.invertGyro) ? 360.0 - ahrs.getYaw() : ahrs.getYaw();
        inputs.yawDegrees = (!Constants.Swerve.invertGyro) ? gyro.getYaw()*-1.0 : gyro.getYaw();
        // if (ahrs.isMagnetometerCalibrated()) {
        //     //      // We will only get valid fused headings if the magnetometer is calibrated
        //        inputs.yawDegrees =  ahrs.getFusedHeading();
        //        }
        //     //
        //     //    // We have to invert the angle of the NavX so that rotating the robot counter-clockwise makes the angle increase.
        //        return (ahrs.getAngle() *-1);
        //       }
        inputs.rollDegrees = -gyro.getPitch()-1.0;
        inputs.pitchDegrees = gyro.getRoll();
        //inputs.yaw = (Constants.Swerve.invertGyro) ? 360.0 - ahrs.getYaw() : ahrs.getYaw();
        //inputs.connected = get status
    }
    public void zeroGyro(){
        gyro.setYaw(0);
    }

    public void calibrateGyro() {
        gyro.configFactoryDefault();
    }
}


