package frc.robot.subsystems.drive;

import java.util.HashMap;
import org.littletonrobotics.junction.Logger;

import frc.robot.Constants;
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
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
//import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import edu.wpi.first.wpilibj.DriverStation.Alliance;

import com.kauailabs.navx.frc.AHRS;
//import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;

public class Swerve extends SubsystemBase {
    public SwerveDrivePoseEstimator swerveDrivePoseEstimator;
    // public ModuleIO[] mSwerveMods;
    private final GyroIO gyroIO;
    private final GyroIOInputsAutoLogged gyroInputs = new GyroIOInputsAutoLogged();

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
        this.limelightIO = limelightIO;
        this.gyroIO = gyroIO;
        ModuleIOs[0] = flModuleIO;
        ModuleIOs[1] = frModuleIO;
        ModuleIOs[2] = blModuleIO;
        ModuleIOs[3] = brModuleIO;

        /*
         * By pausing init for a second before setting module offsets, we avoid a bug
         * with inverting motors.
         * See https://github.com/Team364/BaseFalconSwerve/issues/8 for more info.
         */
        Timer.delay(1.0);
        resetModulesToAbsolute();
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

    public void drive(Translation2d translation, double rotation, boolean isOpenLoop) {
        boolean fieldRelative = Constants.Swerve.fieldRelative;
        // SmartDashboard.putNumber("maxspeed",Constants.Swerve.maxSpeed);
        SmartDashboard.putNumber("gyroyaw", gyroInputs.yawDegrees);
        SwerveModuleState[] swerveModuleStates = Constants.Swerve.swerveKinematics.toSwerveModuleStates(
                ChassisSpeeds.fromFieldRelativeSpeeds(
                        translation.getX(),
                        translation.getY(),
                        rotation,
                        Rotation2d.fromDegrees(gyroInputs.yawDegrees)));

        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.Swerve.maxSpeed);
        SmartDashboard.putNumber("Field Relative?", fieldRelative ? 1d : 0d);
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

    // #####################################################################################
    // #####################################################################################
    // #####################################################################################
    public void driveOntoChargeStation() {
        boolean onRamp = false;
        boolean balanced = false;
        double stopThresholdDegrees = 3;
        double mountingSpeed = 1.0; // meters per second;
        double balancingSpeed = 0.3; // meters per second;
        Timer timecheck = Timer();
        // if (DriverStation.getAlliance() == Alliance.Red) {
        // driveDirection = 0;

        // }

        // Lock wheels toward heading

        SwerveModuleState desiredState = new SwerveModuleState();
        // TODO change this to "for(SwerveModule mod : mSwerveMods){"
        // - RE: yes could write as for(ModuleIO mod : ModuleIOs){ but it has to be a
        // for loop elsewhere in this file
        // and I would rather keep it all the same
        for (int i = 0; i < 4; i++) {
            desiredState.speedMetersPerSecond = mountingSpeed;
            desiredState.angle = Rotation2d.fromDegrees(0);
            ModuleIOs[i].setDesiredState(desiredState, false); // maybe true would be better
        }
        double roll = gyroInputs.rollDegrees;
        timecheck.start();
        while (timecheck.hasElapsed(3) != true && !onRamp) { // if roll or pitch > trigger angle
            roll = gyroInputs.rollDegrees; // scan navx roll or pitch
            if (roll > 9) {
                onRamp = true;// set rampEngaged = true
                timecheck.reset();
            }
        }
        while (timecheck.hasElapsed(4) != true && !balanced) { // if roll or pitch > trigger angle
            roll = gyroInputs.rollDegrees; // scan navx roll or pitch

            if (roll > 4) {
                for (int i = 0; i < 4; i++) {
                    desiredState.angle = Rotation2d.fromDegrees(0); // set angles for 45, 135,225,315 in order to brake
                                                                    // really well
                    desiredState.speedMetersPerSecond = balancingSpeed; // set speed to 0
                    ModuleIOs[i].setDesiredState(desiredState, true);
                }
            } else if (roll < -4) {
                for (int i = 0; i < 4; i++) {
                    desiredState.angle = Rotation2d.fromDegrees(0); // set angles for 45, 135,225,315 in order to brake
                                                                    // really well
                    desiredState.speedMetersPerSecond = -balancingSpeed; // set speed to 0
                    ModuleIOs[i].setDesiredState(desiredState, true);
                }
            }
        }
        //Balanced
        for (int i = 0; i < 4; i++) {
            desiredState.angle = Rotation2d.fromDegrees(i * 90 + 45); // set angles for 45, 135,225,315 in order to
                                                                      // brake really well
            desiredState.speedMetersPerSecond = 0; // set speed to 0
            ModuleIOs[i].setDesiredState(desiredState, false);
        }
        // lets abstract this to its own function since that would be nice to do
        // whenever stationary

        // this should build, ill make it more complicated later if needed, or make it
        // simpler if necessary

        // if roll or pitch ~ 0
        // motorsstop
        // wait 1 second
        // if roll or pitch < -trigger angle
        // drive backward extra slow
        // else if roll or pitch < -trigger angle
        // drive forward extra slow
        // else
        // end condition = true
        //
    }

    // #####################################################################################
    // #####################################################################################
    // #####################################################################################

    private Timer Timer() {
        return null;
    }

    public Pose2d getPose() {
        Pose2d pose = swerveDrivePoseEstimator.getEstimatedPosition();
        Logger.getInstance().recordOutput("SwervePose", pose);

        return pose;
    }

    // public void resetOdometry(Pose2d pose) {
    // swerveOdometry.resetPosition(Rotation2d.fromDegrees(gyroInputs.yawDegrees),
    // getModulePositions(), pose);
    // }

    public void resetPose(Pose2d pose) {
        swerveDrivePoseEstimator.resetPosition(Rotation2d.fromDegrees(gyroInputs.yawDegrees), getModulePositions(),
                pose);
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
        Logger.getInstance().processInputs("Drive/Gyro", gyroInputs);

        for (int i = 0; i < 4; i++) {
            ModuleIOs[i].updateInputs(moduleInputs[i]);
            Logger.getInstance().processInputs("Drive/Module" + Integer.toString(i),
                    moduleInputs[i]);
        }

        if (Constants.enableLimelight) {
            double botPose[] = limelightInputs.botPoseWPI;
            swerveDrivePoseEstimator.addVisionMeasurement(
                    new Pose2d(new Translation2d(botPose[0], botPose[1]), new Rotation2d(botPose[5])),
                    Timer.getFPGATimestamp() - limelightInputs.latency);
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