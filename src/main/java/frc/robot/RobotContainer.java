// Copyright 2021-2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot;

import com.ctre.phoenix6.SignalLogger;
import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.FeederConstants;
import frc.robot.Constants.GroundIntakeConstants;
import frc.robot.commands.ClimberCommand;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.Elevator.MoveElevatorCommand;
import frc.robot.commands.Feeder.FeederWheelCommand;
import frc.robot.commands.Feeder.LaserCan;
import frc.robot.commands.Feeder.MoveFeederConmand;
import frc.robot.commands.Feeder.RunFeederWheel;
import frc.robot.commands.Feeder.StopFeeder;
import frc.robot.commands.GroundIntake.IntakeWheelCommand;
import frc.robot.commands.GroundIntake.MoveIntakeCommand;
import frc.robot.commands.IntakeCollectionCommand;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Feeder.FeederSubsystem;
import frc.robot.subsystems.Feeder.FeederWheelSubsystem;
import frc.robot.subsystems.Feeder.LaserCanSubsystem;
import frc.robot.subsystems.Feeder.i_LaserCanSubsystem;
import frc.robot.subsystems.GroundIntake.GroundIntakeSubsystem;
import frc.robot.subsystems.GroundIntake.GroundIntakeWheelSubsystem;
import frc.robot.subsystems.GroundIntake.IntakeCollectionSubsystem;
import frc.robot.subsystems.climber.ClimberSubsystem;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOTalonFX;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {

  public ElevatorSubsystem getElevatorSubsystem() {
    return elevatorSubsystem;
  }
  // Subsystems
  private final Drive drive;

  // Controller
  private final CommandXboxController m_Controller = new CommandXboxController(0);
  private final CommandXboxController s_Controller = new CommandXboxController(4);

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;

  /*子系统 */
  private ElevatorSubsystem elevatorSubsystem = new ElevatorSubsystem();
  private GroundIntakeSubsystem groundIntakeSubsystem = new GroundIntakeSubsystem();
  private FeederSubsystem feederSubsystem = new FeederSubsystem();
  private FeederWheelSubsystem feederWheelSubsystem = new FeederWheelSubsystem();
  private GroundIntakeWheelSubsystem groundIntakeWheelSubsystem = new GroundIntakeWheelSubsystem();
  private IntakeCollectionSubsystem intakeCollectionSubsystem = new IntakeCollectionSubsystem();
  private ClimberSubsystem climberSubsystem = new ClimberSubsystem();
  private final LaserCanSubsystem laserCanSubsystem = new LaserCanSubsystem();
  private final i_LaserCanSubsystem i_laserCanSubsystem = new i_LaserCanSubsystem();

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

    switch (Constants.currentMode) {
      case REAL:
        // Real robot, instantiate hardware IO implementations
        drive =
            new Drive(
                new GyroIOPigeon2(),
                new ModuleIOTalonFX(TunerConstants.FrontLeft),
                new ModuleIOTalonFX(TunerConstants.FrontRight),
                new ModuleIOTalonFX(TunerConstants.BackLeft),
                new ModuleIOTalonFX(TunerConstants.BackRight));
        break;

      case SIM:
        // Sim robot, instantiate physics sim IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIOSim(TunerConstants.FrontLeft),
                new ModuleIOSim(TunerConstants.FrontRight),
                new ModuleIOSim(TunerConstants.BackLeft),
                new ModuleIOSim(TunerConstants.BackRight));
        break;

      default:
        // Replayed robot, disable IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {});
        break;
    }

    // Set up auto routines
    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

    // Set up SysId routines
    autoChooser.addOption(
        "Drive Wheel Radius Characterization", DriveCommands.wheelRadiusCharacterization(drive));
    autoChooser.addOption(
        "Drive Simple FF Characterization", DriveCommands.feedforwardCharacterization(drive));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Forward)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Reverse)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    autoChooser.addOption(
        "Drive SysId (Dynamic Forward)", drive.sysIdDynamic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Dynamic Reverse)", drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));

    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    // Default command, normal field-relative drive
    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive,
            () -> -m_Controller.getLeftY(),
            () -> -m_Controller.getLeftX(),
            () -> -m_Controller.getRightX()));

    // Lock to 0° when A button is held
    s_Controller
        .a()
        .whileTrue(
            DriveCommands.joystickDriveAtAngle(
                drive,
                () -> -m_Controller.getLeftY(),
                () -> -m_Controller.getLeftX(),
                () -> new Rotation2d()));

    // Switch to X pattern when X button is pressed
    s_Controller.x().onTrue(Commands.runOnce(drive::stopWithX, drive));

    // Reset gyro to 0° when B button is pressed
    s_Controller
        .b()
        .onTrue(
            Commands.runOnce(
                    () ->
                        drive.setPose(
                            new Pose2d(drive.getPose().getTranslation(), new Rotation2d())),
                    drive)
                .ignoringDisable(true));

    /*命令组 */

    m_Controller
        .button(10)
        .whileTrue(
            new ParallelRaceGroup(
                new LaserCan(laserCanSubsystem, i_laserCanSubsystem), // LaserCan 命令
                new RunFeederWheel(feederWheelSubsystem, FeederConstants.Wheel_speed_in),
                new MoveElevatorCommand(
                    elevatorSubsystem, ElevatorConstants.ELEVATOR_POSITION_BOTTOM_ROTATIONS),
                new MoveFeederConmand(feederSubsystem, FeederConstants.INTAKE_ANGLE_PREPARE),
                new IntakeCollectionCommand(intakeCollectionSubsystem),
                new MoveIntakeCommand(
                    groundIntakeSubsystem, GroundIntakeConstants.INTAKE_ANGLE_GRAB)));

    m_Controller
        .y() // 假设按钮 Y 触发该命令组
        .onTrue(
            new ParallelCommandGroup(
                // 电梯移动到 L4 的命令
                new MoveElevatorCommand(
                    elevatorSubsystem, ElevatorConstants.ELEVATOR_POSITION_L4_ROTATIONS),
                // Feeder 移动到 shoot L4 的命令，带条件触发
                Commands.run(
                    () -> {
                      if (elevatorSubsystem.getElevatorPositionRotations() > 35) {
                        new MoveFeederConmand(
                                feederSubsystem, FeederConstants.INTAKE_ANGLE_SHOOT_L4)
                            .schedule();
                      }
                    })));
    m_Controller
        .x() // 假设按钮 Y 触发该命令组
        .onTrue(
            new ParallelCommandGroup(
                // 电梯移动到 L4 的命令
                new MoveElevatorCommand(
                    elevatorSubsystem, ElevatorConstants.ELEVATOR_POSITION_L3_ROTATIONS),
                // Feeder 移动到 shoot L4 的命令，带条件触发
                Commands.run(
                    () -> {
                      if (elevatorSubsystem.getElevatorPositionRotations() > 22) {
                        new MoveFeederConmand(feederSubsystem, FeederConstants.INTAKE_ANGLE_SHOOT)
                            .schedule();
                      }
                    })));
    m_Controller
        .a() // 假设按钮 Y 触发该命令组
        .onTrue(
            new ParallelCommandGroup(
                // 电梯移动到 L4 的命令
                new MoveElevatorCommand(
                    elevatorSubsystem, ElevatorConstants.ELEVATOR_POSITION_L2_ROTATIONS),
                // Feeder 移动到 shoot L4 的命令，带条件触发
                Commands.run(
                    () -> {
                      if (elevatorSubsystem.getElevatorPositionRotations() > 13) {
                        new MoveFeederConmand(feederSubsystem, FeederConstants.INTAKE_ANGLE_SHOOT)
                            .schedule();
                      }
                    })));

    /*Test键位 */

    /*电梯键位 */
    // m_Controller.button(5).whileTrue(new ElevatorUpCommand(elevatorSubsystem));
    // m_Controller.button(6).whileTrue(new ElevatorDownCommand(elevatorSubsystem));
    // elevatorSubsystem.setDefaultCommand(new Stop(elevatorSubsystem));
    /*地面抓取键位 */
    m_Controller
        .button(0)
        .whileTrue(
            new MoveIntakeCommand(groundIntakeSubsystem, GroundIntakeConstants.INTAKE_ANGLE_GRAB));
    m_Controller
        .button(0)
        .whileTrue(
            new MoveIntakeCommand(
                groundIntakeSubsystem, GroundIntakeConstants.INTAKE_ANGLE_PREPARE));
    m_Controller
        .button(0)
        .whileTrue(
            new MoveIntakeCommand(
                groundIntakeSubsystem, GroundIntakeConstants.INTAKE_ANGLE_GROUND));
    m_Controller
        .button(0)
        .whileTrue(
            new MoveIntakeCommand(
                groundIntakeSubsystem, GroundIntakeConstants.INTAKE_ANGLE_DRIVERSTATION));
    /*抓取轮子 */
    m_Controller
        .button(0)
        .whileTrue(
            new IntakeWheelCommand(
                groundIntakeWheelSubsystem, GroundIntakeConstants.WHEEL_SPEED_IN));
    m_Controller
        .button(0)
        .whileTrue(
            new IntakeWheelCommand(
                groundIntakeWheelSubsystem, GroundIntakeConstants.WHEEL_SPEED_IN));

    /*电梯键位 */
    m_Controller
        .button(7)
        .whileTrue(
            new MoveElevatorCommand(
                elevatorSubsystem, ElevatorConstants.ELEVATOR_POSITION_BOTTOM_ROTATIONS));
    m_Controller
        .button(0)
        .whileTrue(
            new MoveElevatorCommand(
                elevatorSubsystem, ElevatorConstants.ELEVATOR_POSITION_L1_ROTATIONS));
    m_Controller
        .button(0)
        .whileTrue(
            new MoveElevatorCommand(
                elevatorSubsystem, ElevatorConstants.ELEVATOR_POSITION_L2_ROTATIONS));
    m_Controller
        .button(0)
        .whileTrue(
            new MoveElevatorCommand(
                elevatorSubsystem, ElevatorConstants.ELEVATOR_POSITION_L3_ROTATIONS));
    m_Controller
        .button(8)
        .whileTrue(
            new MoveElevatorCommand(
                elevatorSubsystem, ElevatorConstants.ELEVATOR_POSITION_L4_ROTATIONS));
    m_Controller
        .button(5)
        .onTrue(new MoveFeederConmand(feederSubsystem, FeederConstants.INTAKE_ANGLE_ALGAE));
    m_Controller
        .button(6)
        .onTrue(new MoveFeederConmand(feederSubsystem, FeederConstants.INTAKE_ANGLE_PREPARE));
    m_Controller
        .b()
        .onTrue(new MoveFeederConmand(feederSubsystem, FeederConstants.INTAKE_ANGLE_SHOOT));

    m_Controller.button(0).whileTrue(new IntakeCollectionCommand(intakeCollectionSubsystem));

    m_Controller
        .button(0)
        .whileTrue(new FeederWheelCommand(feederSubsystem, FeederConstants.WHEEL_SPEED_IN));
    m_Controller
        .button(9)
        .whileTrue(new RunFeederWheel(feederWheelSubsystem, FeederConstants.Wheel_speed_out));

    feederSubsystem.setDefaultCommand(new StopFeeder(feederSubsystem));

    m_Controller.button(0).whileTrue(new ClimberCommand(climberSubsystem));

    m_Controller.button(0).onTrue(Commands.runOnce(SignalLogger::start));
    m_Controller.button(0).onTrue(Commands.runOnce(SignalLogger::stop));

    /*
     * Joystick 7 = quasistatic forward
     * Joystick 8 = quasistatic reverse
     * Joystick 9 = dynamic forward
     * Joystick 10 = dyanmic reverse
     */
    m_Controller
        .button(0)
        .whileTrue(elevatorSubsystem.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    m_Controller
        .button(0)
        .whileTrue(elevatorSubsystem.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    m_Controller
        .button(0)
        .whileTrue(elevatorSubsystem.sysIdDynamic(SysIdRoutine.Direction.kForward));
    m_Controller
        .button(0)
        .whileTrue(elevatorSubsystem.sysIdDynamic(SysIdRoutine.Direction.kReverse));
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
