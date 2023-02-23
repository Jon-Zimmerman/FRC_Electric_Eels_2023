package frc.robot;

import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import frc.lib.util.COTSFalconSwerveConstants;
import frc.lib.util.SwerveModuleConstants;
import edu.wpi.first.wpilibj.RobotBase;

import frc.lib.util.Alert;
import frc.lib.util.Alert.AlertType;

public final class Constants {
    // public static final Mode currentMode = Mode.REAL;
    private static final RobotType robot = RobotType.SIM;
    public static final double simLoopPeriodSecs = 0.02;
    public static final boolean tuningMode = false;

    public static final double stickDeadband = 0.1;

    public static final class SparkMax {
        // High and low rotations for dead reckoning (not sure if degrees or rads yet)
        public static final double top = 100;
        public static final double bottom = 0;
    }

    public static final class Swerve {
        public static final int pigeonID = 1;
        public static final boolean invertGyro = true; // Always ensure Gyro is CCW+ CW-
        public static final boolean teleopIsOpenLoop = false;
        public static final COTSFalconSwerveConstants chosenModule = COTSFalconSwerveConstants
                .SDSMK4i(COTSFalconSwerveConstants.driveGearRatios.SDSMK4i_L2);

        /* Drivetrain Constants */
        public static final double trackWidth = Units.inchesToMeters(19);
        public static final double wheelBase = Units.inchesToMeters(28.875);
        public static final double wheelCircumference = chosenModule.wheelCircumference;

        /*
         * Swerve Kinematics
         * No need to ever change this unless you are not doing a traditional
         * rectangular/square 4 module swerve
         */
        public static final SwerveDriveKinematics swerveKinematics = new SwerveDriveKinematics(
                new Translation2d(wheelBase / 2.0, trackWidth / 2.0),
                new Translation2d(wheelBase / 2.0, -trackWidth / 2.0),
                new Translation2d(-wheelBase / 2.0, trackWidth / 2.0),
                new Translation2d(-wheelBase / 2.0, -trackWidth / 2.0));

        /* Module Gear Ratios */
        public static final double driveGearRatio = chosenModule.driveGearRatio;
        public static final double angleGearRatio = chosenModule.angleGearRatio;

        /* Motor Inverts */
        public static final boolean angleMotorInvert = chosenModule.angleMotorInvert;
        public static final boolean driveMotorInvert = chosenModule.driveMotorInvert;

        /* Angle Encoder Invert */
        public static final boolean canCoderInvert = chosenModule.canCoderInvert;

        /* Swerve Current Limiting */
        public static final int angleContinuousCurrentLimit = 35;
        public static final int anglePeakCurrentLimit = 45;
        public static final double anglePeakCurrentDuration = 0.1;
        public static final boolean angleEnableCurrentLimit = true;

        public static final int driveContinuousCurrentLimit = 35;
        public static final int drivePeakCurrentLimit = 45;
        public static final double drivePeakCurrentDuration = 0.1;
        public static final boolean driveEnableCurrentLimit = true;

        /*
         * These values are used by the drive falcon to ramp in open loop and closed
         * loop driving.
         * We found a small open loop ramp (0.25) helps with tread wear, tipping, etc
         */
        public static final double openLoopRamp = 0.25;
        public static final double closedLoopRamp = 0.25;

        /* Angle Motor PID Values */
        public static final double angleKP = chosenModule.angleKP;
        public static final double angleKI = chosenModule.angleKI;
        public static final double angleKD = chosenModule.angleKD;
        public static final double angleKF = chosenModule.angleKF;

        /* Drive Motor PID Values */
        public static final double driveKP = 0.05;
        public static final double driveKI = 0.0;
        public static final double driveKD = 0.0;
        public static final double driveKF = 0.0;

        /*
         * Drive Motor Characterization Values
         * Divide SYSID values by 12 to convert from volts to percent output for CTRE
         */
        public static final double driveKS = (0.09 / 12);
        public static final double driveKV = (0.70 / 12);
        public static final double driveKA = (0.3 / 12);

        /* Swerve Profiling Values */
        /** Meters per Second */
        public static final double maxSpeed = 4;
        /** Radians per Second */
        public static final double maxAngularVelocity = (2 * Math.PI);// 1;
        // maximum *decimal*, 0 to 1 throttle to clamp to in swervemodule.java
        public static final double maxOpenLoopThrottle = 1.0;
        /* Neutral Modes */
        public static final NeutralMode angleNeutralMode = NeutralMode.Coast;
        public static final NeutralMode driveNeutralMode = NeutralMode.Brake;

        /* Module Specific Constants */

        /* Back Right Module - Module 3 */
        public static final class Mod3 {
            public static final int driveMotorID = 0;
            public static final int angleMotorID = 2;
            public static final int canCoderID = 8;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(95.27);
            public static final SwerveModuleConstants constants = new SwerveModuleConstants(driveMotorID, angleMotorID,
                    canCoderID, angleOffset);
        }

        /* Back Left Module - Module 2 */
        public static final class Mod2 {
            public static final int driveMotorID = 1;
            public static final int angleMotorID = 3;
            public static final int canCoderID = 9;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(13.44);
            public static final SwerveModuleConstants constants = new SwerveModuleConstants(driveMotorID, angleMotorID,
                    canCoderID, angleOffset);
        }

        /* Front Left Module - Module 0 */
        public static final class Mod0 {
            public static final int driveMotorID = 5;
            public static final int angleMotorID = 7;
            public static final int canCoderID = 10;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(236.95);
            public static final SwerveModuleConstants constants = new SwerveModuleConstants(driveMotorID, angleMotorID,
                    canCoderID, angleOffset);
        }

        /* Front Right Module - Module 1 */
        public static final class Mod1 {
            public static final int driveMotorID = 6;
            public static final int angleMotorID = 4;
            public static final int canCoderID = 11;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(200.83);
            public static final SwerveModuleConstants constants = new SwerveModuleConstants(driveMotorID, angleMotorID,
                    canCoderID, angleOffset);
        }
    }

    // Elevator Extender Motor
    public static final class ElevatorSubsystem {
        public static final int deviceID = 12;
        public static final int min = 0;
        public static final int max = 100;

        // FeedForward Control
        public static final double ks = 0.1;
        public static final double kv = 0.05;
        public static final double kg = 0.05;

        public static final double kP = 0.1;
        public static final double kI = 1e-4;
        public static final double kD = 1;
        public static final double kIz = 0;
        public static final double kFF = 0;
        public static final double kMaxOutput = 1;
        public static final double kMinOutput = -1;

        public static final double gearRatio = 25.0;
        public static final double sprocketDiameterInch = 2.0;

        //motor shaft details
        public static final int maxCurrentAmps = 10;
        public static final double maxAngularVelocityRPM = 1000.0;
        public static final double maxAngularAccRPMPerSec = 10.0;
        public static final double minOutputVelocityRPM = 100.0;
        public static final double allowableSmartMotionPosErrorCounts = 100.0;

        public static final double elevatorSoftLimitLower = 0;
        public static final double elevatorSoftLimitUpper = 0;
        public static final double elevatorPosBottom = 0.0;
        public static final double elevatorPosMid = 40.0;
        public static final double elevatorPosLoading = 70.0;
        public static final double elevatorPosTop = 100.0;

    }

    // Elevator Slider Motor
    public static final class SliderSubsystem {
        public static final int deviceID = 13;

        // FeedForward Control
        public static final double ks = 0.1;
        public static final double kv = 0.05;
        public static final double kg = 0.05;

        public static final double gearRatio = 2.0;
        public static final double sprocketDiameterInch = 2.0;

        public static final double kP = 0.1;
        public static final double kI = 1e-4;
        public static final double kD = 1.0;
        public static final double kIz = 0.0;
        public static final double kFF = 0.0;
        public static final double kMaxOutput = 1.0;
        public static final double kMinOutput = -1.0;

        public static final int maxCurrentAmps = 10;
        public static final double maxLinearVelocityMPS = 10.0;
        public static final double maxAngularVelocityRPM = 1000.0;
        public static final double maxAngularAccRPMPerSec = 10.0;
        public static final double minOutputVelocityRPM = 100.0;
        public static final double allowableSmartMotionPosErrorCounts = 100.0;

        public static final double sliderSoftLimitLower = 0.0;
        public static final double sliderSoftLimitUpper = 0.0;
        public static final double sliderIn = 0.0;
        public static final double sliderOut = 0.0;

    }

    // Grabber motor
    public static final class IntakeSubsystem {
        public static final int deviceID = 14;

        // FeedForward Control
        public static final double ks = 0.1;
        public static final double kv = 0.05;
        // Closed Loop Control
        public static final double kP = 0.05;
        public static final double kI = 1e-4;
        public static final double kD = 0.0;
        public static final double kIz = 0;
        public static final double kFF = 0;
        public static final double kMaxOutput = 1;
        public static final double kMinOutput = -1;

        public static final int maxCurrentAmps = 10;
        public static final double maxAngularVelocityRPM = 10.0;
        public static final int holdCurrentAmps = 2;

        public static final double intakeInCubeVelRPM = 100.0;
        public static final double intakeInConeVelRPM = 100.0;
        public static final double intakeOutCubeVelRPM = 100.0;
        public static final double intakeOutConeVelRPM = 100.0;
    }

    public static final class AutoConstants {
        public static final double kMaxSpeedMetersPerSecond = 0.25;
        public static final double kMaxAccelerationMetersPerSecondSquared = 0.25;
        public static final double kMaxAngularSpeedRadiansPerSecond = (2 * Math.PI) / 4;
        public static final double kMaxAngularSpeedRadiansPerSecondSquared = (2 * Math.PI) / 4;

        public static final double kPXController = 0.1;
        public static final double kPYController = 0.1;
        public static final double kPThetaController = 0.1;

        /* Constraint for the motion profilied robot angle controller */
        public static final TrapezoidProfile.Constraints kThetaControllerConstraints = new TrapezoidProfile.Constraints(
                kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
    }

    private static final Alert invalidRobotAlert = new Alert(
            "Invalid robot selected, using competition robot as default.",
            AlertType.ERROR);

    public static RobotType getRobot() {
        if (RobotBase.isReal()) {
            if (robot == RobotType.SIM) { // Invalid robot selected
                invalidRobotAlert.set(true);
                return RobotType.REAL;
            } else {
                return robot;
            }
        } else {
            return robot;
        }
    }

    public static Mode getMode() {
        switch (getRobot()) {
            case REAL:
                return RobotBase.isReal() ? Mode.REAL : Mode.REPLAY;

            case SIM:
                return Mode.SIM;

            default:
                return Mode.REAL;
        }
    }

    public static enum RobotType {
        REAL, SIM
    }

    public static enum Mode {
        REAL, REPLAY, SIM
    }
}
