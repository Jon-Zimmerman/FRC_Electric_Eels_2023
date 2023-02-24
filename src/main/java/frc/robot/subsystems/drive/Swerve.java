package frc.robot.subsystems.drive;

import java.util.HashMap;
import java.util.function.Consumer;

import org.littletonrobotics.junction.Logger;

import frc.robot.Constants;//
//import frc.robot.subsystems.drive.ModuleIO.ModuleIOInputs;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;

public class Swerve extends SubsystemBase {
    public SwerveDriveOdometry swerveOdometry;
    // public ModuleIO[] mSwerveMods;
    private final GyroIO gyroIO;
    private final GyroIOInputsAutoLogged gyroInputs = new GyroIOInputsAutoLogged();

    public SwerveAutoBuilder swerveAutoBuilder;


    private final ModuleIO[] ModuleIOs = new ModuleIO[4]; // FL, FR, BL, BR
    private final ModuleIOInputsAutoLogged[] moduleInputs = new ModuleIOInputsAutoLogged[] {
            new ModuleIOInputsAutoLogged(),
            new ModuleIOInputsAutoLogged(),
            new ModuleIOInputsAutoLogged(),
            new ModuleIOInputsAutoLogged() };

    public Swerve(GyroIO gyroIO, ModuleIO flModuleIO, ModuleIO frModuleIO,
            ModuleIO blModuleIO, ModuleIO brModuleIO) {
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
        swerveOdometry = new SwerveDriveOdometry(Constants.Swerve.swerveKinematics,
                Rotation2d.fromDegrees(gyroInputs.yawDegrees), getModulePositions());



        swerveAutoBuilder = new SwerveAutoBuilder(
                this::getPose,
                this::resetPose,
                Constants.Swerve.swerveKinematics,
                new PIDConstants(1.0, 0.0, 0.2),
                new PIDConstants(0.5, 0.0, 0.2),
                this::setModuleStatesAuto,
                new HashMap<>(),
                true,
                this);
    }

    public void drive(Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop) {
        // SmartDashboard.putNumber("maxspeed",Constants.Swerve.maxSpeed);
        SmartDashboard.putNumber("gyroyaw", gyroInputs.yawDegrees);
        SwerveModuleState[] swerveModuleStates = Constants.Swerve.swerveKinematics.toSwerveModuleStates(
                fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(
                        translation.getX(),
                        translation.getY(),
                        rotation,
                        Rotation2d.fromDegrees(gyroInputs.yawDegrees))
                        : new ChassisSpeeds(
                                translation.getX(),
                                translation.getY(),
                                rotation));
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.Swerve.maxSpeed);
        SmartDashboard.putNumber("Robocentric yes?", fieldRelative ? 1d : 0d);
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
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.Swerve.maxSpeed);
        for (int i = 0; i < 4; i++) {
            ModuleIOs[i].setDesiredState(desiredStates[i], false);

        }
        Logger.getInstance().recordOutput("desiredSwerveStates", desiredStates);

    }

    public Pose2d getPose() {
        Logger.getInstance().recordOutput("SwervePose", swerveOdometry.getPoseMeters());
        return swerveOdometry.getPoseMeters();
    }

    public void resetOdometry(Pose2d pose) {
        swerveOdometry.resetPosition(Rotation2d.fromDegrees(gyroInputs.yawDegrees), getModulePositions(), pose);
    }
    public void resetPose(Pose2d pose) {
        swerveOdometry.resetPosition(Rotation2d.fromDegrees(gyroInputs.yawDegrees), getModulePositions(), pose);
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
        swerveOdometry.update(Rotation2d.fromDegrees(gyroInputs.yawDegrees), getModulePositions());
        getPose();

        for (int i = 0; i < 4; i++) {
            SmartDashboard.putNumber("Mod " + i + " Cancoder", moduleInputs[i].cancoderDegrees);
            SmartDashboard.putNumber("Mod " + i + " Integrated", moduleInputs[i].angleMotorPositionDegrees);
            SmartDashboard.putNumber("Mod " + i + " Velocity", moduleInputs[i].driveMotorStateMetersPerSecond);
        }

    }

}