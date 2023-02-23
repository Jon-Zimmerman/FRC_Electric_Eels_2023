package frc.robot.subsystems.drive;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.lib.util.SwerveModuleConstants;
import frc.robot.Constants;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.lib.util.CTREModuleState;

public class ModuleIOSim implements ModuleIO {
    public int moduleNumber;
    private Rotation2d angleOffset;
    private Rotation2d lastAngle;

    SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(Constants.Swerve.driveKS, Constants.Swerve.driveKV,
            Constants.Swerve.driveKA);

    private FlywheelSim driveSim = new FlywheelSim(DCMotor.getFalcon500(1), 1.0, 0.004);
    private FlywheelSim turnSim = new FlywheelSim(DCMotor.getFalcon500(1), 1.0, 0.0002);

    private double drivePositionRad;
    private double driveVelocityRadPerSec;
    private double turnAbsolutePositionRad;
    private double turnRelativePositionRad;
    private double test2 = 0.0;

    public ModuleIOSim(int moduleNumber, SwerveModuleConstants moduleConstants) {
        this.moduleNumber = moduleNumber;
        this.angleOffset = moduleConstants.angleOffset;
        resetToAbsolute();
        lastAngle = getState().angle;
    }

    @Override
    public void updateInputs(ModuleIOInputs inputs) {
        driveSim.update(Constants.simLoopPeriodSecs);
        turnSim.update(Constants.simLoopPeriodSecs);

        // Motor measurements not module measurements
        double angleDifference = (turnSim.getAngularVelocityRadPerSec() * Constants.simLoopPeriodSecs);
        turnAbsolutePositionRad += angleDifference;
        turnRelativePositionRad += angleDifference;

        while (turnAbsolutePositionRad < 0) {
            turnAbsolutePositionRad += 2.0 * Math.PI;
        }
        while (turnAbsolutePositionRad > 2.0 * Math.PI) {
            turnAbsolutePositionRad -= 2.0 * Math.PI;
        }

        drivePositionRad = drivePositionRad + (driveSim.getAngularVelocityRadPerSec() * Constants.simLoopPeriodSecs);
        driveVelocityRadPerSec = driveSim.getAngularVelocityRadPerSec();

        // Note these vars below are both actually the output of their respective
        // geartrains ie distance the module
        // has traveled linearly and the angle of the wheel not the motors themselves
        inputs.driveMotorPositionDistanceMeters = falconRadsToMeters(drivePositionRad,
                Constants.Swerve.wheelCircumference,
                Constants.Swerve.driveGearRatio);

        inputs.angleMotorPositionDegrees = test2;
        Logger.getInstance().recordOutput("test2" + moduleNumber, test2);

        inputs.driveMotorStateMetersPerSecond = falconRadsPerSecToMetersPerSec(driveVelocityRadPerSec,
                Constants.Swerve.wheelCircumference, Constants.Swerve.driveGearRatio);

        inputs.cancoderDegrees = turnAbsolutePositionRad * 360.0 / (2.0 * Math.PI);

    }

    public void setDesiredState(SwerveModuleState desiredState, boolean isOpenLoop) {
        /*
         * This is a custom optimize function, since default WPILib optimize assumes
         * continuous controller which CTRE and Rev onboard is not
         */
        desiredState = CTREModuleState.optimize(desiredState, getState().angle);
        setAngle(desiredState);
        setSpeed(desiredState, isOpenLoop);
    }
    // public static double clamp(double val, double min, double max) {
    // return Math.max(min, Math.min(max, val));
    // }

    private void setSpeed(SwerveModuleState desiredState, boolean isOpenLoop) {
        if (isOpenLoop) {
            double percentOutput = desiredState.speedMetersPerSecond / Constants.Swerve.maxSpeed;

            percentOutput = percentOutput * Constants.Swerve.maxOpenLoopThrottle; // reduce open loop throttle to a
                                                                                  // reasonable number becasue
                                                                                  // desiredState speed is too much

            SmartDashboard.putNumber("Throttle %", percentOutput * 100.0);
            //Logger.getInstance().recordOutput("OL Drive M" + moduleNumber + " % Throttle", percentOutput);
            driveSim.setInputVoltage(percentOutput*12.0);
            // mDriveMotor.set(ControlMode.PercentOutput, percentOutput);
        } else {
            //double velocity = Conversions.MPSToFalcon(desiredState.speedMetersPerSecond,
            //        Constants.Swerve.wheelCircumference, Constants.Swerve.driveGearRatio);
            
            double percentOutput = desiredState.speedMetersPerSecond / Constants.Swerve.maxSpeed;
            //Logger.getInstance().recordOutput("CL Drive M" + moduleNumber + " % Throttle", percentOutput);
            driveSim.setInputVoltage(percentOutput*12.0);
            
            // mDriveMotor.set(ControlMode.Velocity, velocity,
            // DemandType.ArbitraryFeedForward,
            // feedforward.calculate(desiredState.speedMetersPerSecond));
        }
    }

    private void setAngle(SwerveModuleState desiredState) {
        Rotation2d angle = (Math.abs(desiredState.speedMetersPerSecond) <= (Constants.Swerve.maxSpeed * 0.01))
                ? lastAngle
                : desiredState.angle; // Prevent rotating module if speed is less then 1%. Prevents Jittering.
        //double percentOutput = (angle.getDegrees()-lastAngle.getDegrees());
        //Logger.getInstance().recordOutput("CL Turn M" + moduleNumber + " % Throttle", percentOutput);
        //turnSim.setInputVoltage(percentOutput);
        // mAngleMotor.set(ControlMode.Position,
        // Conversions.degreesToFalcon(angle.getDegrees(),
        // Constants.Swerve.angleGearRatio));
        test2 = angle.getDegrees();
        lastAngle = angle;
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(falconRadsPerSecToMetersPerSec(driveVelocityRadPerSec,
                Constants.Swerve.wheelCircumference, Constants.Swerve.driveGearRatio),
                getAngle());
    }

    private Rotation2d getAngle() {
        return Rotation2d.fromDegrees(falconRadsToWheelAngleDegs(turnRelativePositionRad,
                Constants.Swerve.angleGearRatio));
    }

    public Rotation2d getCanCoder() {
        return Rotation2d.fromDegrees(turnAbsolutePositionRad * 360.0 / (2.0 * Math.PI));
    }

    public void resetToAbsolute() {
        double absolutePosition = WheelAngleDegsTofalconRads(getCanCoder().getDegrees() - angleOffset.getDegrees(),
                Constants.Swerve.angleGearRatio);
        // mAngleMotor.setSelectedSensorPosition(absolutePosition);
        turnRelativePositionRad = absolutePosition;
    }

    public static double falconRadsToMeters(double rads, double circumference, double gearRatio) {
        return rads * (circumference / (gearRatio * 2.0 * Math.PI));
    }

    public static double falconRadsPerSecToMetersPerSec(double radsPerSec, double circumference, double gearRatio) {
        return radsPerSec * (circumference / (gearRatio * 2.0 * Math.PI));
    }

    public static double falconRadsToWheelAngleDegs(double rads, double gearRatio) {
        //double test = rads * (1.0 / (gearRatio * 2.0 * Math.PI));
        double deg = rads *36.0 * (1.0 / (gearRatio * 2.0 * Math.PI));
        return deg;
    }

    public static double WheelAngleDegsTofalconRads(double degs, double gearRatio) {
        return degs / (1.0 / (gearRatio * 2.0 * Math.PI));
    }
}
