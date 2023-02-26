package frc.robot.subsystems.drive;

import org.littletonrobotics.junction.AutoLog;



import edu.wpi.first.math.kinematics.SwerveModuleState;

public interface ModuleIO {

    @AutoLog
    public static class ModuleIOInputs {
        /** Updates the set of loggable inputs. */
        //public SwerveModuleState state = new SwerveModuleState();
        //public SwerveModulePosition position = new SwerveModulePosition();
        public double driveMotorPositionDistanceMeters = 0.0;

        public double driveMotorStateMetersPerSecond = 0.0;

        public double angleMotorPositionDegrees = 0.0;

        public double cancoderDegrees = 0.0;

    }

    public default void updateInputs(ModuleIOInputs inputs) {
    }

    public default void setDesiredState(SwerveModuleState desiredState, boolean isOpenLoop) {}
    //public default void setSpeed(SwerveModuleState desiredState, boolean isOpenLoop) {}
    //public default void setSpeed(SwerveModuleState desiredState, boolean isOpenLoop) {}

    //public default void setAngle(SwerveModuleState desiredState) {}

    public default void resetToAbsolute() {}

    //private default void configAngleEncoder() {}

    //private default void configAngleMotor() {}

    //public default void configDriveMotor() {}

}
