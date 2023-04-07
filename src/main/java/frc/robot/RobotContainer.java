// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;

import frc.robot.autos.Test_Odometry;
import frc.robot.autos.Top_Cube_Travel;
import frc.robot.autos.Top_Cube_Extended_Cube;
import frc.robot.autos.Top_Cone_Grab_Cone_Spicy_Meatball;

import frc.robot.autos.Mid_Cube_Balance;

import frc.robot.autos.Bottom_Cube_Extended_Cube;
import frc.robot.autos.Bottom_Cube_Travel;

import frc.robot.subsystems.drive.Swerve;
import frc.robot.commands.TeleopSwerve;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOFalcon;

import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeIO;
import frc.robot.subsystems.intake.IntakeIOSim;
import frc.robot.subsystems.intake.IntakeIOSparkMax;

import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.ElevatorIO;
import frc.robot.subsystems.elevator.ElevatorIOSim;
import frc.robot.subsystems.elevator.ElevatorIOSparkMax;

import frc.robot.subsystems.slider.Slider;
import frc.robot.subsystems.slider.SliderIO;
import frc.robot.subsystems.slider.SliderIOSim;
import frc.robot.subsystems.slider.SliderIOSparkMax;
//import frc.robot.subsystems.slider.SliderIOFalcon;

import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOSim;
import frc.robot.subsystems.drive.GyroIONavx;

import frc.robot.subsystems.drive.LimelightIO;
import frc.robot.subsystems.drive.LimelightIOSim;
import frc.robot.subsystems.drive.LimelightIONetwork;

//Commands:
import frc.robot.commands.ElevatorGoToPosition;
import frc.robot.commands.SliderGoToPosition;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls). Instead, the structure of the robot (including subsystems,
 * commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  private final double allowableElevatorTeleopInch = Constants.ElevatorSubsystem.allowableTeleopErrorInch;

  private final double allowableSliderTeleopInch = Constants.ElevatorSubsystem.allowableTeleopErrorInch;

  // Controllers

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser = new LoggedDashboardChooser<>("Auto Choices");
  // private final LoggedDashboardNumber intakeSpeedInput = new
  // LoggedDashboardNumber("Intake Speed", 1500.0);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */

  private final Joystick driver2 = new Joystick(1);
  private final JoystickButton flipIntakeMode = new JoystickButton(driver2, 7);
  private final JoystickButton intakeIn = new JoystickButton(driver2, XboxController.Button.kRightBumper.value);
  private final JoystickButton intakeOut = new JoystickButton(driver2, XboxController.Button.kLeftBumper.value);

  private final JoystickButton elevatorBottom = new JoystickButton(driver2, XboxController.Button.kA.value);
  private final JoystickButton elevatorMid = new JoystickButton(driver2, XboxController.Button.kX.value);
  //private final JoystickButton elevatorAltLoading = new JoystickButton(driver2, XboxController.Button.kX.value);
  private final JoystickButton elevatorLoading = new JoystickButton(driver2, XboxController.Button.kB.value);
  private final JoystickButton elevatorTop = new JoystickButton(driver2, XboxController.Button.kY.value);

  // DPad
  private final POVButton sliderIn = new POVButton(driver2, 180);
  private final POVButton sliderOut = new POVButton(driver2, 0);

  // private final Joystick driver = new Joystick(0);
  // private final int translationAxis = XboxController.Axis.kLeftY.value;
  // private final int strafeAxis = XboxController.Axis.kLeftX.value;
  // private final int rotationAxis = XboxController.Axis.kRightX.value;

  // private final JoystickButton lockToHeading = new JoystickButton(driver,
  // XboxController.Button.kA.value);

  // private final JoystickButton zeroGyro = new JoystickButton(driver,
  // XboxController.Button.kY.value);
  // private final JoystickButton calibrate = new JoystickButton(driver,
  // XboxController.Button.kRightBumper.value);
  // probably remove robotcentric

  // private final JoystickButton autoFocus = new JoystickButton(driver,
  // XboxController.Button.kA.value);

  // /* Driver Buttons */
  private final Joystick driver = new Joystick(0);
  private final int translationAxis = Joystick.AxisType.kY.value;
  private final int strafeAxis = Joystick.AxisType.kX.value;
  private final int rotationAxis = Joystick.AxisType.kZ.value;
  // private final int throttleAxis = Joystick.AxisType.kTwist.value;
  private final JoystickButton calibrate = new JoystickButton(driver, 12);

  private final JoystickButton zeroGyro = new JoystickButton(driver, 8);
  // private final JoystickButton robotCentric = new JoystickButton(driver, 7);
  private final JoystickButton lockToHeading = new JoystickButton(driver,
      1);

  // Subsystems

  private final Swerve j_Swerve;
  private final Intake intake;
  private final Elevator elevator;
  private final Slider slider;

  public RobotContainer() {

    switch (Constants.getMode()) {
      // Real robot, instantiate hardware IO implementations
      case REAL:
        Timer.delay(1.0);
        // drive = new Drive(new DriveIOSparkMax());
        if (Constants.enableLimelight) {

          j_Swerve = new Swerve(
              new LimelightIONetwork(),
              new GyroIONavx(),
              new ModuleIOFalcon(0, Constants.Swerve.Mod0.constants),
              new ModuleIOFalcon(1, Constants.Swerve.Mod1.constants),
              new ModuleIOFalcon(2, Constants.Swerve.Mod2.constants),
              new ModuleIOFalcon(3, Constants.Swerve.Mod3.constants));
          intake = new Intake(new IntakeIOSparkMax());
          elevator = new Elevator(new ElevatorIOSparkMax());
          slider = new Slider(new SliderIOSparkMax());
        } else {
          j_Swerve = new Swerve(
              new LimelightIOSim(),
              new GyroIONavx(),
              new ModuleIOFalcon(0, Constants.Swerve.Mod0.constants),
              new ModuleIOFalcon(1, Constants.Swerve.Mod1.constants),
              new ModuleIOFalcon(2, Constants.Swerve.Mod2.constants),
              new ModuleIOFalcon(3, Constants.Swerve.Mod3.constants));
          intake = new Intake(new IntakeIOSparkMax());
          elevator = new Elevator(new ElevatorIOSparkMax());
          slider = new Slider(new SliderIOSparkMax());

        }

        break;

      // Sim robot, instantiate physics sim IO implementations
      case SIM:
        // drive = new Drive(new DriveIOSim());
        j_Swerve = new Swerve(
            new LimelightIOSim(),
            new GyroIOSim(() -> -driver.getRawAxis(rotationAxis)),
            new ModuleIOSim(0, Constants.Swerve.Mod0.constants),
            new ModuleIOSim(1, Constants.Swerve.Mod1.constants),
            new ModuleIOSim(2, Constants.Swerve.Mod2.constants),
            new ModuleIOSim(3, Constants.Swerve.Mod3.constants));
        intake = new Intake(new IntakeIOSim());
        elevator = new Elevator(new ElevatorIOSim());
        slider = new Slider(new SliderIOSim());
        break;
      case CHASSIS:
        // drive = new Drive(new DriveIOSim());
        j_Swerve = new Swerve(
            new LimelightIONetwork(),
            new GyroIONavx(),
            new ModuleIOFalcon(0, Constants.Swerve.Mod0.constants),
            new ModuleIOFalcon(1, Constants.Swerve.Mod1.constants),
            new ModuleIOFalcon(2, Constants.Swerve.Mod2.constants),
            new ModuleIOFalcon(3, Constants.Swerve.Mod3.constants));

        intake = new Intake(new IntakeIO() {
        });
        elevator = new Elevator(new ElevatorIO() {
        });
        slider = new Slider(new SliderIO() {
        });
        break;
      // Replayed robot, disable IO implementations
      default:
        // drive = new Drive(new DriveIO() {});
        j_Swerve = new Swerve(
            new LimelightIO() {
            },
            new GyroIO() {
            },
            new ModuleIO() {
            },
            new ModuleIO() {
            },
            new ModuleIO() {
            },
            new ModuleIO() {
            });
        intake = new Intake(new IntakeIO() {
        });
        elevator = new Elevator(new ElevatorIO() {
        });
        slider = new Slider(new SliderIO() {
        });
        break;
    }

    autoChooser.addDefaultOption("Do Nothing", new InstantCommand());
    //autoChooser.addOption("Test odometry", new Test_Odometry(j_Swerve));

    autoChooser.addOption("Top: Cube & Travel", new Top_Cube_Travel(j_Swerve, intake, elevator, slider));
    //autoChooser.addOption("Top_Cube_Grab_Cube", new Top_Cube_Extended_Cube(j_Swerve, intake, elevator, slider));
    autoChooser.addOption("Top_Cone_Grab_Cube_Score_SpicyMB", new Top_Cone_Grab_Cone_Spicy_Meatball(j_Swerve, intake, elevator, slider));

    autoChooser.addOption("Middle: Cube & Balance", new Mid_Cube_Balance(j_Swerve, intake, elevator, slider));

    autoChooser.addOption("Bottom: Cube & Travel", new Bottom_Cube_Travel(j_Swerve, intake, elevator, slider));
    //autoChooser.addOption("Bottom_Cube_Ext_Cube", new Bottom_Cube_Extended_Cube(j_Swerve, intake, elevator, slider));

    // Configure the button bindings
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
        () -> driver.getRawAxis(translationAxis),
        () -> driver.getRawAxis(strafeAxis),
        () -> -driver.getRawAxis(rotationAxis),
        () -> lockToHeading.getAsBoolean()));

    zeroGyro.onTrue(new InstantCommand(() -> j_Swerve.zeroGyro()));
    calibrate.onTrue(new InstantCommand(() -> j_Swerve.calibrateGyro()));

    flipIntakeMode.onTrue(new InstantCommand(() -> intake.flipIntakeMode()));

    intakeIn.whileTrue(new StartEndCommand(() -> intake.intakeIn(), () -> intake.holdCurrent(), intake));
    intakeOut.whileTrue(new StartEndCommand(() -> intake.intakeOut(), intake::stop, intake));

    elevatorBottom.onTrue(
        new ElevatorGoToPosition(Constants.ElevatorSubsystem.elevatorPosBottom, allowableElevatorTeleopInch, elevator));
    //elevatorAltLoading.onTrue(
        //new ElevatorGoToPosition(Constants.ElevatorSubsystem.elevatorPosAltLoading, allowableElevatorTeleopInch, elevator));
    elevatorMid.onTrue(
        new ElevatorGoToPosition(Constants.ElevatorSubsystem.elevatorPosMid, allowableElevatorTeleopInch, elevator));
    elevatorLoading.onTrue(new ElevatorGoToPosition(Constants.ElevatorSubsystem.elevatorPosLoading,
        allowableElevatorTeleopInch, elevator));
    elevatorTop.onTrue(
        new ElevatorGoToPosition(Constants.ElevatorSubsystem.elevatorPosTop, allowableElevatorTeleopInch, elevator));

    sliderIn.onTrue(new SliderGoToPosition(Constants.SliderSubsystem.sliderIn, allowableSliderTeleopInch, slider));
    sliderOut.onTrue(new SliderGoToPosition(Constants.SliderSubsystem.sliderOut, allowableSliderTeleopInch, slider));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.get();
  }

  public void resetElevator() {
    elevator.setPositionSetPoint(elevator.getPosition());
  }
}
