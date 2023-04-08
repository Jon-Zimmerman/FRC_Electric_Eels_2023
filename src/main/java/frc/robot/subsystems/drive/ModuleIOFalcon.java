
//SwerveIOModule    

package frc.robot.subsystems.drive;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.lib.math.Conversions;
import frc.lib.util.CTREModuleState;
import frc.lib.util.SwerveModuleConstants;
import frc.robot.Constants;
import frc.robot.Robot;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.CANCoder;

public class ModuleIOFalcon implements ModuleIO {
    public int moduleNumber;
    private Rotation2d angleOffset;
    private Rotation2d lastAngle;
    private Rotation2d angleMotorAngle;
    private SwerveModuleState driveMotorState;

    private TalonFX mAngleMotor;
    private TalonFX mDriveMotor;
    private CANCoder angleEncoder;


    SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(Constants.Swerve.driveKS, Constants.Swerve.driveKV,
            Constants.Swerve.driveKA);

    public ModuleIOFalcon(int moduleNumber, SwerveModuleConstants moduleConstants) {
        this.moduleNumber = moduleNumber;
        this.angleOffset = moduleConstants.angleOffset;

        /* Angle Encoder Config */
        angleEncoder = new CANCoder(moduleConstants.cancoderID,"Drivetrain");
        configAngleEncoder();

        /* Angle Motor Config */
        mAngleMotor = new TalonFX(moduleConstants.angleMotorID,"Drivetrain");
        configAngleMotor();

        /* Drive Motor Config */
        mDriveMotor = new TalonFX(moduleConstants.driveMotorID,"Drivetrain");
        configDriveMotor();

        lastAngle = getState().angle;
    }

    @Override
    public void updateInputs(ModuleIOInputs inputs) {
        // Note these are actually the output of their respective geartrains ie
        // distance the module has traveled linearly and the angle of the wheel not the motors themselves
        inputs.driveMotorPositionDistanceMeters = Conversions.falconToMeters(mDriveMotor.getSelectedSensorPosition(),
                Constants.Swerve.wheelCircumference,
                Constants.Swerve.driveGearRatio);

        inputs.driveMotorStateMetersPerSecond = driveMotorState.speedMetersPerSecond;

        inputs.angleMotorPositionDegrees = angleMotorAngle.getDegrees();

        inputs.cancoderDegrees = getCancoder();
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

    private void setSpeed(SwerveModuleState desiredState, boolean isOpenLoop) {
        if (isOpenLoop) {
            double percentOutput = desiredState.speedMetersPerSecond / Constants.Swerve.maxSpeed;

            percentOutput = percentOutput * Constants.Swerve.maxOpenLoopThrottle; // reduce open loop throttle to a reasonable number because
                                                                                  // desiredState speed is too much
            SmartDashboard.putNumber("Throttle %", percentOutput * 100.0);
            mDriveMotor.set(ControlMode.PercentOutput, percentOutput);
        } else {
            double velocity = Conversions.MPSToFalcon(desiredState.speedMetersPerSecond,
                    Constants.Swerve.wheelCircumference, Constants.Swerve.driveGearRatio);
            mDriveMotor.set(ControlMode.Velocity, velocity, DemandType.ArbitraryFeedForward,
                    feedforward.calculate(desiredState.speedMetersPerSecond));
        }
    }

    private void setAngle(SwerveModuleState desiredState) {
        Rotation2d desiredAngle = (Math.abs(desiredState.speedMetersPerSecond) <= (Constants.Swerve.maxSpeed * Constants.Swerve.JitterCutoff))
                ? lastAngle
                : desiredState.angle; // Prevent rotating module if speed is less then 1%. Prevents Jittering.

        mAngleMotor.set(ControlMode.Position,
                Conversions.degreesToFalcon(desiredAngle.getDegrees(), Constants.Swerve.angleGearRatio));
        lastAngle = desiredAngle;
    }

    public SwerveModuleState getState() {
        SwerveModuleState state = new SwerveModuleState(
                Conversions.falconToMPS(mDriveMotor.getSelectedSensorVelocity(), Constants.Swerve.wheelCircumference,
                        Constants.Swerve.driveGearRatio),
                getAngle());
        driveMotorState = state;
        return state;
    }

    private Rotation2d getAngle() {
        Rotation2d angle = Rotation2d.fromDegrees(
                Conversions.falconToDegrees(mAngleMotor.getSelectedSensorPosition(), Constants.Swerve.angleGearRatio));
        angleMotorAngle = angle;
        return angle;
    }

    public double getCancoder() {
        return angleEncoder.getAbsolutePosition();
    }

    public void resetToAbsolute() {
        double absolutePosition = Conversions.degreesToFalcon(getCancoder() - angleOffset.getDegrees(),
                Constants.Swerve.angleGearRatio);
        Timer.delay(0.25);
        mAngleMotor.setSelectedSensorPosition(absolutePosition);
    }

    private void configAngleEncoder() {
        angleEncoder.configFactoryDefault();
        angleEncoder.configAllSettings(Robot.ctreConfigs.swerveCanCoderConfig);
    }

    private void configAngleMotor() {
        mAngleMotor.configFactoryDefault();
        mAngleMotor.configAllSettings(Robot.ctreConfigs.swerveAngleFXConfig);
        mAngleMotor.setInverted(Constants.Swerve.angleMotorInvert);
        mAngleMotor.setNeutralMode(Constants.Swerve.angleNeutralMode);
        resetToAbsolute();
        Timer.delay(1.5);
        resetToAbsolute();
    }

    private void configDriveMotor() {
        mDriveMotor.configFactoryDefault();
        mDriveMotor.configAllSettings(Robot.ctreConfigs.swerveDriveFXConfig);
        mDriveMotor.setInverted(Constants.Swerve.driveMotorInvert);
        mDriveMotor.setNeutralMode(Constants.Swerve.driveNeutralMode);
        mDriveMotor.setSelectedSensorPosition(0);
    }


}