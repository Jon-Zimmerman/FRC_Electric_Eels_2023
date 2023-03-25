package frc.robot.subsystems.drive;

import java.util.HashMap;
import org.littletonrobotics.junction.Logger;

import frc.robot.Constants;
import frc.robot.Constants.Mode;
//import frc.robot.subsystems.drive.LimelightIO.LimelightIOInputs;
//import frc.robot.subsystems.drive.ModuleIO.ModuleIOInputs;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
//import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
//import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

//import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;

public class Swerve extends SubsystemBase {
    public SwerveDrivePoseEstimator swerveDrivePoseEstimatorLL;
    public SwerveDrivePoseEstimator swerveDrivePoseEstimator;
    // public ModuleIO[] mSwerveMods;
    private final GyroIO gyroIO;
    public final GyroIOInputsAutoLogged gyroInputs = new GyroIOInputsAutoLogged();

    public SwerveAutoBuilder swerveAutoBuilder;

    private final LimelightIO limelightIO;
    private final LimelightIOInputsAutoLogged limelightInputs = new LimelightIOInputsAutoLogged();

    private final ModuleIO[] ModuleIOs = new ModuleIO[4]; // FL, FR, BL, BR
    private final ModuleIOInputsAutoLogged[] moduleInputs = new ModuleIOInputsAutoLogged[] {
            new ModuleIOInputsAutoLogged(),
            new ModuleIOInputsAutoLogged(),
            new ModuleIOInputsAutoLogged(),
            new ModuleIOInputsAutoLogged() };

    public Swerve(LimelightIO limelightIO, GyroIO gyroIO, ModuleIO flModuleIO, ModuleIO frModuleIO,
            ModuleIO blModuleIO, ModuleIO brModuleIO) {
        this.gyroIO = gyroIO;
        ModuleIOs[0] = flModuleIO;
        ModuleIOs[1] = frModuleIO;
        ModuleIOs[2] = blModuleIO;
        ModuleIOs[3] = brModuleIO;
        this.limelightIO = limelightIO;
        /*
         * By pausing init for a second before setting module offsets, we avoid a bug
         * with inverting motors.
         * See https://github.com/Team364/BaseFalconSwerve/issues/8 for more info.
         */
        Timer.delay(1.0);
        resetModulesToAbsolute();
        swerveDrivePoseEstimatorLL = new SwerveDrivePoseEstimator(Constants.Swerve.swerveKinematics,
                Rotation2d.fromDegrees(gyroInputs.yawDegrees), getModulePositions(), new Pose2d());
        swerveDrivePoseEstimator = new SwerveDrivePoseEstimator(Constants.Swerve.swerveKinematics,
                Rotation2d.fromDegrees(gyroInputs.yawDegrees), getModulePositions(), new Pose2d());

        swerveAutoBuilder = new SwerveAutoBuilder(
                this::getPose,
                this::resetPose,
                Constants.Swerve.swerveKinematics,
                Constants.AutoConstants.translationPIDConstants,
                Constants.AutoConstants.rotationPIDConstants,
                this::setModuleStatesAuto,
                new HashMap<>(),
                Constants.AutoConstants.readAllianceColortoFlipPaths,
                this);
    }

    public void drive(Translation2d translation, double rotation, boolean isOpenLoop, boolean lockToHeading) {
        // boolean fieldRelative = Constants.Swerve.fieldRelative;
        // SmartDashboard.putNumber("maxspeed",Constants.Swerve.maxSpeed);

        SwerveModuleState[] swerveModuleStates;
        double lockToHeadingPScalar = 0.3;
        double translationScalar = 0.5;
        double degreeThreshold = 15.0;
        if (lockToHeading && Constants.enableLockToHeading) { // lock robot to a specific heading so that you can successfully place game objects
            double error = 0.0;
            if ((Math.abs(gyroInputs.yawDegrees % 360.0)) < degreeThreshold) {
                error = 0.0 - (gyroInputs.yawDegrees % 360.0);
            } else if ((gyroInputs.yawDegrees % 360.0) > (180.0 - degreeThreshold)) {
                error = 180.0 - (gyroInputs.yawDegrees % 360.0);
            } else if ((gyroInputs.yawDegrees % 360.0) < (-180.0 + degreeThreshold)) {
                error = -180.0 - (gyroInputs.yawDegrees % 360.0);
            }

            double rotationtest = error / degreeThreshold * lockToHeadingPScalar * Constants.Swerve.maxAngularVelocity;
           rotation = rotation*translationScalar;
            translation = translation.times(translationScalar);
            //SmartDashboard.putNumber("gyroyaw", gyroInputs.yawDegrees);
            //SmartDashboard.putNumber("LocktoHeadingError", error);
            //SmartDashboard.putNumber("LocktoHeadingRotation", rotationtest);
            
            if (Constants.getMode() == Mode.SIM) {
                gyroIO.additionalRotation(rotation);
            }
        }
        //SmartDashboard.putNumber("tX", translation.getX());
        //SmartDashboard.putNumber("tY", translation.getY());
        //SmartDashboard.putNumber("rot", rotation);
        if (translation.getX() == 0.0 && translation.getY() == 0.0 && rotation == 0.0
                && Constants.enableLockWheelsAt45) {
                    //SmartDashboard.putNumber("brake?", 1);
            swerveModuleStates = getLockWheels45();
        } else {
            //SmartDashboard.putNumber("brake?", 0);
            swerveModuleStates = Constants.Swerve.swerveKinematics.toSwerveModuleStates(
                    ChassisSpeeds.fromFieldRelativeSpeeds(
                            translation.getX(),
                            translation.getY(),
                            rotation,
                            Rotation2d.fromDegrees(gyroInputs.yawDegrees)));
        }

        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.Swerve.maxSpeed);
        // SmartDashboard.putNumber("Field Relative?", fieldRelative ? 1d : 0d);

        getModuleStates();

        setModuleStates(swerveModuleStates, isOpenLoop);

    }

    public void zeroGyro() {
        gyroIO.zeroGyro();
    }

    public void calibrateGyro() {
        gyroIO.calibrateGyro();
    }

    public void setModuleStates(SwerveModuleState[] desiredStates, boolean isOpenLoop) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.Swerve.maxSpeed);
        for (int i = 0; i < 4; i++) {
            ModuleIOs[i].setDesiredState(desiredStates[i], isOpenLoop);
        }
        Logger.getInstance().recordOutput("desiredSwerveStates", desiredStates);
    }

    /* Used by SwerveControllerCommand in Auto */
    public void setModuleStatesAuto(SwerveModuleState[] desiredStates) {
        setModuleStates(desiredStates, false);
    }

    public SwerveModuleState[] getLockWheels45() {

        SwerveModuleState[] WheelStates45 = Constants.Swerve.swerveKinematics.toSwerveModuleStates(
                ChassisSpeeds.fromFieldRelativeSpeeds(
                        0.0,
                        0.0,
                        0.0,
                        Rotation2d.fromDegrees(gyroInputs.yawDegrees)));
        // SwerveModuleState desiredState = new SwerveModuleState();
        for (int i = 0; i < 4; i++) {
            final double[] anglestobrake= {225,135,315,45};
            WheelStates45[i].angle = Rotation2d.fromDegrees( anglestobrake[i]); // set angles for 45, 135,225,315 in order to
            // brake really well
            WheelStates45[i].speedMetersPerSecond = 0; // set speed to 0
        }
        return WheelStates45;
    }

    public void resetPose(Pose2d pose) {
        swerveDrivePoseEstimatorLL.resetPosition(Rotation2d.fromDegrees(gyroInputs.yawDegrees), getModulePositions(),
                pose);
        swerveDrivePoseEstimator.resetPosition(Rotation2d.fromDegrees(gyroInputs.yawDegrees), getModulePositions(),
                pose);
    }

    public Pose2d getPose() {
        Pose2d pose = new Pose2d();
        if(Constants.LimelightAffectsOdometry){
            pose = swerveDrivePoseEstimatorLL.getEstimatedPosition();
            Logger.getInstance().recordOutput("SwervePoseLL", pose);
        }
        else{
            pose = swerveDrivePoseEstimator.getEstimatedPosition();
            Logger.getInstance().recordOutput("SwervePose", pose);
            Pose2d poseLL = swerveDrivePoseEstimatorLL.getEstimatedPosition();
            Logger.getInstance().recordOutput("SwervePoseLL", poseLL);
        }

        return pose;
    }

    public SwerveModuleState[] getModuleStates() {
        SwerveModuleState[] states = new SwerveModuleState[4];

        states[0] = new SwerveModuleState(moduleInputs[0].driveMotorStateMetersPerSecond,
                Rotation2d.fromDegrees(moduleInputs[0].angleMotorPositionDegrees));

        states[1] = new SwerveModuleState(moduleInputs[1].driveMotorStateMetersPerSecond,
                Rotation2d.fromDegrees(moduleInputs[1].angleMotorPositionDegrees));

        states[2] = new SwerveModuleState(moduleInputs[2].driveMotorStateMetersPerSecond,
                Rotation2d.fromDegrees(moduleInputs[2].angleMotorPositionDegrees));

        states[3] = new SwerveModuleState(moduleInputs[3].driveMotorStateMetersPerSecond,
                Rotation2d.fromDegrees(moduleInputs[3].angleMotorPositionDegrees));

        Logger.getInstance().recordOutput("actualSwerveStates", states);

        return states;
    }

    public SwerveModulePosition[] getModulePositions() {
        SwerveModulePosition[] positions = new SwerveModulePosition[4];

        for (int i = 0; i < 4; i++) {
            positions[i] = new SwerveModulePosition(moduleInputs[i].driveMotorPositionDistanceMeters,
                    Rotation2d.fromDegrees(moduleInputs[i].angleMotorPositionDegrees));
        }
        return positions;
    }

    public void resetModulesToAbsolute() {
        for (int i = 0; i < 4; i++) {
            ModuleIOs[i].resetToAbsolute();
        }
    }

    @Override
    public void periodic() {
        gyroIO.updateInputs(gyroInputs);
        limelightIO.updateInputs(limelightInputs);
        Logger.getInstance().processInputs("Drive/Gyro", gyroInputs);

        for (int i = 0; i < 4; i++) {
            ModuleIOs[i].updateInputs(moduleInputs[i]);
            Logger.getInstance().processInputs("Drive/Module" + Integer.toString(i),
                    moduleInputs[i]);
        }

        if (Constants.enableLimelight) {
            double botPose[] = limelightInputs.botPoseWPI;
            // Botpos transform in field-space (driver station WPILIB origin). Translation
            // (X,Y,Z) Rotation(Roll,Pitch,Yaw), total latency (cl+tl)
            if (botPose[0] < Constants.acceptableLimelightMergeDistMeters && botPose[0] != 0.0) {

                swerveDrivePoseEstimatorLL.addVisionMeasurement(
                        new Pose2d(new Translation2d(botPose[0], botPose[1]), new Rotation2d(botPose[5])),
                        Timer.getFPGATimestamp() - limelightInputs.latency);
            }
            swerveDrivePoseEstimatorLL.update(Rotation2d.fromDegrees(gyroInputs.yawDegrees), getModulePositions());

        }
        swerveDrivePoseEstimator.update(Rotation2d.fromDegrees(gyroInputs.yawDegrees), getModulePositions());

        getPose();

        for (int i = 0; i < 4; i++) {
            SmartDashboard.putNumber("Mod " + i + " Cancoder", moduleInputs[i].cancoderDegrees);
            SmartDashboard.putNumber("Mod " + i + " Integrated", moduleInputs[i].angleMotorPositionDegrees);
            SmartDashboard.putNumber("Mod " + i + " Velocity", moduleInputs[i].driveMotorStateMetersPerSecond);
        }

    }

}