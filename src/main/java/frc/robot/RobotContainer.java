// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;
import org.littletonrobotics.junction.networktables.LoggedDashboardNumber;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Joystick.ButtonType;
import edu.wpi.first.wpilibj.interfaces.Gyro;
//import edu.wpi.first.wpilibj.XboxController;
//import edu.wpi.first.wpilibj.PS4Controller.Button;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;

import edu.wpi.first.wpilibj2.command.button.JoystickButton;

import frc.robot.autos.*;
import frc.robot.commands.*;
//import frc.robot.subsystems.*;

// import frc.robot.commands.DriveWithIntakeAuto;
// import frc.robot.commands.SpinAuto;
// import frc.robot.subsystems.drive.Drive;
// import frc.robot.subsystems.drive.DriveIO;
// import frc.robot.subsystems.drive.DriveIOSim;
// import frc.robot.subsystems.drive.DriveIOSparkMax;
import frc.robot.subsystems.drive.Swerve;
import frc.robot.subsystems.drive.ModuleIOFalcon;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIO;

import frc.robot.subsystems.drive.GyroIONavx;
import frc.robot.subsystems.drive.GyroIOSim;
import frc.robot.subsystems.drive.GyroIO;

import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeIO;
import frc.robot.subsystems.intake.IntakeIOSim;
import frc.robot.subsystems.intake.IntakeIOSparkMax;
import frc.robot.subsystems.intake.SparkMax;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls). Instead, the structure of the robot (including subsystems,
 * commands, and button mappings) should be declared here.
 */
public class RobotContainer {

  // Controllers
  //

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser = new LoggedDashboardChooser<>("Auto Choices");
  private final LoggedDashboardNumber intakeSpeedInput = new LoggedDashboardNumber("Intake Speed", 1500.0);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */  
   private final Joystick driver = new Joystick(4);
  private final int translationAxis = XboxController.Axis.kLeftY.value;
  private final int strafeAxis = XboxController.Axis.kLeftX.value;
  private final int rotationAxis = XboxController.Axis.kRightX.value;

  private final JoystickButton zeroGyro = new JoystickButton(driver, XboxController.Button.kY.value);
  private final JoystickButton calibrate = new JoystickButton(driver, XboxController.Button.kRightBumper.value);
  private final JoystickButton robotCentric = new JoystickButton(driver, XboxController.Button.kLeftBumper.value);
  private final JoystickButton elevatorPos1 = new JoystickButton(driver, XboxController.Button.kA.value);
  private final JoystickButton elevatorPos2 = new JoystickButton(driver, XboxController.Button.kB.value);
  /* Driver Buttons */
  // private final Joystick driver = new Joystick(0);
  // private final int translationAxis = Joystick.AxisType.kY.value;
  // private final int strafeAxis = Joystick.AxisType.kX.value;
  // private final int rotationAxis = Joystick.AxisType.kZ.value;
  // private final int throttleAxis = Joystick.AxisType.kTwist.value;
  // private final JoystickButton calibrate = new JoystickButton(driver, 12);

  // private final JoystickButton zeroGyro = new JoystickButton(driver, 8  );
  // private final JoystickButton robotCentric = new JoystickButton(driver, 7);
  // private final JoystickButton elevatorPos1 = new JoystickButton(driver, 3);
  // private final JoystickButton elevatorPos2 = new JoystickButton(driver, 4);


  // Subsystems
  // private final Drive drive;

  /* Subsystems */
  // private final Swerve s_Swerve = new Swerve();
  private final Swerve j_Swerve;

  private final SparkMax s_max = new SparkMax();
  private final Intake intake;

  public RobotContainer() {
    switch (Constants.currentMode) {
      // Real robot, instantiate hardware IO implementations
      case REAL:
        // drive = new Drive(new DriveIOSparkMax());
        j_Swerve = new Swerve(new GyroIONavx(),
            new ModuleIOFalcon(0, Constants.Swerve.Mod0.constants),
            new ModuleIOFalcon(1, Constants.Swerve.Mod1.constants),
            new ModuleIOFalcon(2, Constants.Swerve.Mod2.constants),
            new ModuleIOFalcon(3, Constants.Swerve.Mod3.constants));
        intake = new Intake(new IntakeIOSparkMax());

        // new TeleopSwerve(
        // s_Swerve,
        // () -> -driver.getRawAxis(translationAxis),
        // () -> -driver.getRawAxis(strafeAxis),
        // () -> -driver.getRawAxis(rotationAxis),
        // () -> robotCentric.getAsBoolean()));

        // drive = new Drive(new DriveIOFalcon500());
        // intake = new Intake(new IntakeIOSparkMax());
        break;

      // Sim robot, instantiate physics sim IO implementations
      case SIM:
        // drive = new Drive(new DriveIOSim());
        j_Swerve = new Swerve(new GyroIOSim(() -> -driver.getRawAxis(rotationAxis)),
        new ModuleIOSim(0, Constants.Swerve.Mod0.constants),
        new ModuleIOSim(1, Constants.Swerve.Mod1.constants),
        new ModuleIOSim(2, Constants.Swerve.Mod2.constants),
        new ModuleIOSim(3, Constants.Swerve.Mod3.constants));
        intake = new Intake(new IntakeIOSim());
        break;

      // Replayed robot, disable IO implementations
      default:
        // drive = new Drive(new DriveIO() {});
        j_Swerve = new Swerve(new GyroIO() {},
        new ModuleIO() {},
        new ModuleIO() {},
        new ModuleIO() {},
        new ModuleIO() {});
        intake = new Intake(new IntakeIO() {});
        
        break;
    }

    // Set up auto routines
    autoChooser.addDefaultOption("Do Nothing", new InstantCommand());
    // autoChooser.addOption("Spin", new SpinAuto(drive));
    // autoChooser.addOption("Drive With Intake", new DriveWithIntakeAuto(drive,
    // intake));

    // Configure the button bindings
    s_max.maxInit();
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by instantiating a {@link GenericHID} or one of its subclasses
   * ({@link edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then
   * passing it to a {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    // drive.setDefaultCommand(
    // new RunCommand(() -> drive.driveArcade(-controller.getLeftY(),
    // controller.getLeftX()), drive));
    j_Swerve.setDefaultCommand(new TeleopSwerve(
        j_Swerve,
        () -> -driver.getRawAxis(translationAxis),
        () -> -driver.getRawAxis(strafeAxis),
        () -> -driver.getRawAxis(rotationAxis),
        () -> robotCentric.getAsBoolean()));

        // driver.a()
        // .whileTrue(new StartEndCommand(() -> intake.runVelocity(intakeSpeedInput.get()), intake::stop, intake));

    zeroGyro.onTrue(new InstantCommand(() -> j_Swerve.zeroGyro()));

    elevatorPos1.onTrue(new InstantCommand(() -> s_max.extend(0d)));
    elevatorPos2.onTrue(new InstantCommand(() -> s_max.extend(5d)));
    calibrate.onTrue(new InstantCommand(() -> j_Swerve.calibrateGyro()));

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.get();
  }
  
}
