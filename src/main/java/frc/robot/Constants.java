package frc.robot;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.pathplanner.lib.auto.PIDConstants;

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
    private static final RobotType robot = RobotType.REAL;
    public static boolean enableLimelight = false;
    public static boolean LimelightAffectsOdometry = false;
    public static boolean enableLockWheelsAt45= true; //Not currently implemented at all, value does nothing
    public static boolean enableLockToHeading= true;
    public static double acceptableLimelightMergeDistMeters = 3; //distance from back of grid in X direction to allow tag inputs
    public static final double simLoopPeriodSecs = 0.02;
    public static final boolean tuningMode = false;

    public static final double translationStickDeadband = 0.08;
    public static final double rotationStickDeadband = 0.08;

    public static final class Swerve {
        
        public static final boolean invertGyro = true; // Always ensure Gyro is CCW+ CW-
        public static final boolean teleopIsOpenLoop = false;
        public static final boolean fieldRelative = true; //note changing to false will not change operation mode

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
        public static final int angleContinuousCurrentLimit = 25;
        public static final int anglePeakCurrentLimit = 32;
        public static final double anglePeakCurrentDuration = 0.1;
        public static final boolean angleEnableCurrentLimit = true;

        public static final int driveContinuousCurrentLimit = 25;
        public static final int drivePeakCurrentLimit = 32;
        public static final double drivePeakCurrentDuration = 0.1;
        public static final boolean driveEnableCurrentLimit = true;

        /*
         * These values are used by the drive falcon to ramp in open loop and closed
         * loop driving.
         * We found a small open loop ramp (0.25) helps with tread wear, tipping, etc
         */
        public static final double openLoopRamp = 0.5;
        public static final double closedLoopRamp = 0.5;
        public static final double JitterCutoff = 0.015; //jitter cutoff value to stop the motors from turning at low speeds and being unable to start due to low starting torque

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
        public static final double maxSpeed = 7.0;
        /** Radians per Second */
        public static final double maxAngularVelocity = (Math.PI)*1.25;// 1;
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
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(44.208);
            public static final SwerveModuleConstants constants = new SwerveModuleConstants(driveMotorID, angleMotorID,
                    canCoderID, angleOffset);
        }

        /* Back Left Module - Module 2 */
        public static final class Mod2 {
            public static final int driveMotorID = 1;
            public static final int angleMotorID = 3;
            public static final int canCoderID = 9;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(349.892);
            public static final SwerveModuleConstants constants = new SwerveModuleConstants(driveMotorID, angleMotorID,
                    canCoderID, angleOffset);
        }

        /* Front Left Module - Module 0 */
        public static final class Mod0 {
            public static final int driveMotorID = 5;
            public static final int angleMotorID = 7;
            public static final int canCoderID = 10;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(197.050);
            public static final SwerveModuleConstants constants = new SwerveModuleConstants(driveMotorID, angleMotorID,
                    canCoderID, angleOffset);
        }

        /* Front Right Module - Module 1 */
        public static final class Mod1 {
            public static final int driveMotorID = 6;
            public static final int angleMotorID = 4;
            public static final int canCoderID = 11;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(38.000);
            public static final SwerveModuleConstants constants = new SwerveModuleConstants(driveMotorID, angleMotorID,
                    canCoderID, angleOffset);
        }
    }
    public static final class AutoConstants {
        public static final boolean readAllianceColortoFlipPaths = true;

        public static final double kMaxSpeedMetersPerSecond = 1.0;
        public static final double kMaxAccelerationMetersPerSecondSquared = 1.0;
        public static final double kMaxAngularSpeedRadiansPerSecond = 0.1*Math.PI;
        public static final double kMaxAngularSpeedRadiansPerSecondSquared = 0.1*Math.PI;

        public static final PIDConstants translationPIDConstants = new PIDConstants(0.1, 0.0, 0.0);
        public static final PIDConstants rotationPIDConstants = new PIDConstants(0.7, 0.0, 0.0);

        /* Constraint for the motion profilied robot angle controller */
        public static final TrapezoidProfile.Constraints kThetaControllerConstraints = new TrapezoidProfile.Constraints(
                kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
    }
    // Elevator Extender Motor
    public static final class ElevatorSubsystem {
        public static final int deviceID = 13;
        public static final boolean isInverted = true;

        // FeedForward Control
        public static final double ks = 0.00;
        public static final double kv = 0.04; //0.2
        public static final double kg = 0.5; //0.75

        // public static final double ks = 0.00;
        // public static final double kv = 0.25;
        // public static final double kg = 0.85;

        public static final double kP = 0.05;
        public static final double kI = 0.00;
        public static final double kD = 0.0;
        public static final double kIz = 0;
        public static final double kFF = 0;
        public static final double kMaxOutput = 1;
        public static final double kMinOutput = -1;

        public static final double gearRatio = 25.0;
        public static final double sprocketDiameterInch = 1.92;

        //motor shaft details
        public static final int maxCurrentAmps = 30;
        public static final double maxAngularVelocityRPM = 100.0;
        public static final double maxAngularAccRPMPerSec = 80.0;
        public static final double minOutputVelocityRPM = 10.0; //requests below this no voltage output
        public static final double allowableSmartMotionPosErrorRotations = 3.0*gearRatio;
        public static final double autoPositionErrorInch = 2.0;

        //Elevator details
        public static final double maxLinearVelocityInchPerSec = 28.0;
        public static final double maxLinearAccelerationInchPerSec = 28.0;

        //Inches
        public static final double elevatorSoftLimitLowerInch = 0;
        public static final double elevatorPosBottom = 0.0;
        public static final double elevatorPosMid = 22.0;
        public static final double elevatorPosLoading = 23.0;
        public static final double elevatorPosTop = 24.2;
        public static final double elevatorSoftLimitUpperInch = 25.25;

        public static final double simCarriageWeightKg = 9.0; // ~20 lbs
        public static final double allowableTeleopErrorInch = 1.0;
    }

    // Slider Motor
    public static final class SliderSubsystem {
        public static final int deviceID = 14;
        public static final int sensorResolution = 2048;
        public static final boolean isInverted = false;
        // FeedForward Control
        public static final double ks = 0.0;
        public static final double kv = 0.00;
        public static final double kg = 0.00;

        public static final double gearRatio = 4.0*30.0/12.0;
        public static final double sprocketDiameterInch = 1.29;
        public static final double kP = 0.06;
        public static final double kI = 0.00;
        public static final double kD = 0.0;
        public static final double kIz = 0.0;
        public static final double kFF = 0.0;
        public static final double kMaxOutput = 1.0;
        public static final double kMinOutput = -1.0;
        public static final int kTimeoutMs = 30;

        public static final int maxCurrentAmps = 30;

        public static final double maxAngularVelocityRPM = 800.0;
        public static final double maxAngularAccRPMPerSec = 1000.0;
        public static final double minOutputVelocityRPM = 20.0; //requests below this no voltage output
        public static final double allowableSmartMotionPosErrorRotations = 1.4*gearRatio;
        public static final double autoPositionErrorInch = 2.0;

        public static final double maxLinearVelocityInchPerSec = 45.0;
        public static final double maxLinearAccelerationInchPerSec = 45.0;
        //Inches
        public static final double sliderSoftLimitLowerInch = 0.0;
        public static final double sliderIn = 0.0;
        public static final double sliderOut = 14.6;
        public static final double sliderSoftLimitUpperInch = 15.0;

        public static final double simCarriageWeightKg = 4.0; // ~20 lbs


        //public static final double allowableErrorInch = 1.0;

    }

    // Intake motor
    public static final class IntakeSubsystem {
        public static final int deviceID = 12;
        public static final boolean isInverted = false;

        // FeedForward Control
        public static final double ks = 0.0;
        public static final double kv = 0.000;
        // Closed Loop Control
        public static final double kP = 0.1;
        public static final double kI = 0.00;
        public static final double kD = 0.0;
        public static final double kIz = 0;
        public static final double kFF = 0;
        public static final double kMaxOutput = 1;
        public static final double kMinOutput = -1;



        public static final double gearRatio = 4.0*2.0;
        //public static final double maxAngularVelocityRPM = 100.0;

        public static final int maxCurrentAmps = 25;
        public static final int holdCubeCurrentAmps = 10;
        public static final int holdConeCurrentAmps = 10;
        //Velocity control mode
        public static final double intakeInCubeVelRPM = 50.0;

        public static final double intakeOutCubeVelRPM = -50.0;
        
        public static final double intakeInConeVelRPM = -100.0;

        public static final double intakeOutConeVelRPM = 100.0;
        //Voltage control mode
        public static final double holdCubeVoltage = 4.0;
        public static final double holdConeVoltage = -3.2;

        public static final double intakeInCubeVoltage = 8.0;
        public static final double intakeOutCubeVoltage = -5.0;
        
        public static final double intakeInConeVoltage = -7.0;
        public static final double intakeOutConeVoltage = 5.0;   
        

    }



    private static final Alert invalidRobotAlert = new Alert(
            "Invalid robot selected, using competition robot as default.",
            AlertType.ERROR);

    public static RobotType getRobot() {
        if (RobotBase.isReal()) {
            if (robot == RobotType.SIM  ) { // Invalid robot selected
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

            case CHASSIS:
                return Mode.CHASSIS;
            default:
                return Mode.REAL;
        }
    }

    public static enum RobotType {
        REAL, SIM, CHASSIS
    }

    public static enum Mode {
        REAL, REPLAY, SIM, CHASSIS
    }
}
