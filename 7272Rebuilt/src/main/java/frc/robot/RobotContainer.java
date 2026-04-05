// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.File;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.LightstripEnvirobots;
import frc.robot.commands.Routine;
import frc.robot.subsystems.Indexing_Subsystem;
import frc.robot.subsystems.Intake_Subsystem;
import frc.robot.subsystems.Lightstrip;
import frc.robot.subsystems.Shooter_Subsystem;
import frc.robot.subsystems.SwerveSubsystem;
import swervelib.SwerveInputStream;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very
 * little robot logic should actually be handled in the {@link Robot} periodic
 * methods (other than the scheduler calls).
 * Instead, the structure of the robot (including subsystems, commands, and
 * trigger mappings) should be declared here.
 */
public class RobotContainer {

  private final SwerveSubsystem drivebase = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(),
      "swerve/neo"));

  private final Indexing_Subsystem m_indexer = new Indexing_Subsystem();
  private final Shooter_Subsystem m_shooter = new Shooter_Subsystem(() -> drivebase.getPose());
  private final Intake_Subsystem m_intake = new Intake_Subsystem();
  private final Lightstrip m_Lightstrip0 = new Lightstrip(1);
  private final Routine m_Routine = new Routine();

  // Replace with CommandPS4Controller or CommandJoystick if needed
  final CommandXboxController driverXbox = new CommandXboxController(0);
  final CommandJoystick buttonBoard = new CommandJoystick(1);
  // The robot's subsystems and commands are defined here...

  // Establish a Sendable Chooser that will be able to be sent to the
  // SmartDashboard, allowing selection of desired auto
  private final SendableChooser<Command> autoChooser;

  /**
   * Converts driver input into a field-relative ChassisSpeeds that is controlled
   * by angular velocity.
   */
  // SwerveInputStream driveAngularVelocity = SwerveInputStream.of(
  // drivebase.getSwerveDrive(),
  // () -> driverXbox.getLeftY(),
  // () -> driverXbox.getLeftX())
  // .withControllerRotationAxis(() -> driverXbox.getRightX())
  // .deadband(0.1)
  // .scaleTranslation(0.8)
  // .allianceRelativeControl(true);

  // // Create an aiming version
  // SwerveInputStream driveAiming = driveAngularVelocity.copy()
  // .aim(new Pose2d(4.6, 4, new Rotation2d())) // target pose
  // .aimWhile(() -> driverXbox.leftBumper().getAsBoolean());

  // Use it as your default command

  SwerveInputStream driveAngularVelocity = SwerveInputStream.of(drivebase.getSwerveDrive(),
      () -> driverXbox.getLeftY() * -1,
      () -> driverXbox.getLeftX() * -1)
      .withControllerRotationAxis(driverXbox::getRightX)
      .deadband(OperatorConstants.DEADBAND)
      .scaleTranslation(0.8)
      .allianceRelativeControl(true);
  // .aim(() -> {
  // var alliance = DriverStation.getAlliance();
  // if (alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red) {
  // return Constants.FieldConstants.REDHUB_POSE2D;
  // }
  // return Constants.FieldConstants.BLUEHUB_POSE2D;
  // })
  // .aimWhile(() -> driverXbox.rightBumper().getAsBoolean())
  // .aimLookahead(Milliseconds.of(150))

  /**
   * Clone's the angular velocity input stream and converts it to a fieldRelative
   * input stream.
   */
  SwerveInputStream driveDirectAngle = driveAngularVelocity.copy().withControllerHeadingAxis(driverXbox::getRightX,
      driverXbox::getRightY)
      .headingWhile(true);

  /**
   * Clone's the angular velocity input stream and converts it to a robotRelative
   * input stream.
   */
  SwerveInputStream driveRobotOriented = driveAngularVelocity.copy().robotRelative(true)
      .allianceRelativeControl(false);

  SwerveInputStream driveAngularVelocityKeyboard = SwerveInputStream.of(drivebase.getSwerveDrive(),
      () -> -driverXbox.getLeftY(),
      () -> -driverXbox.getLeftX())
      .withControllerRotationAxis(() -> driverXbox.getRawAxis(
          2))
      .deadband(OperatorConstants.DEADBAND)
      .scaleTranslation(0.8)
      .allianceRelativeControl(true);
  // Derive the heading axis with math!
  SwerveInputStream driveDirectAngleKeyboard = driveAngularVelocityKeyboard.copy()
      .withControllerHeadingAxis(() -> Math.sin(
          driverXbox.getRawAxis(
              2) *
              Math.PI)
          *
          (Math.PI *
              2),
          () -> Math.cos(
              driverXbox.getRawAxis(
                  2) *
                  Math.PI)
              *
              (Math.PI *
                  2))
      .headingWhile(true)
      .translationHeadingOffset(true)
      .translationHeadingOffset(Rotation2d.fromDegrees(
          0));

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */

  // private final SendableChooser<Command> autoChooser;
  public RobotContainer() {
    UsbCamera frontCamera = CameraServer.startAutomaticCapture("intake", 0);
    frontCamera.setResolution(640, 480);
    frontCamera.setFPS(30);

    // path planner commands
    // NamedCommands.registerCommand("lower_intake",m_Routine.ArmDown());
    // NamedCommands.registerCommand("shoot",m_Routine.ShootCommand(0.8));

    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Chooser", autoChooser);

    m_Lightstrip0.setDefaultCommand(new LightstripEnvirobots(m_Lightstrip0));

    SmartDashboard.putData("Auto Chooser", autoChooser);

    // Configure the trigger bindings
    configureBindings();
    DriverStation.silenceJoystickConnectionWarning(true);

    // Set the default auto (do nothing)
    // autoChooser.setDefaultOption("Do Nothing",
    // Commands.runOnce(drivebase::zeroGyroWithAlliance)
    // .andThen(Commands.none()));

    // // Add a simple auto option to have the robot drive forward for 1 second then
    // // stop
    // autoChooser.addOption("Drive Forward",
    // Commands.runOnce(drivebase::zeroGyroWithAlliance).withTimeout(.2)
    // .andThen(drivebase.driveForward().withTimeout(1)));
    // // Put the autoChooser on the SmartDashboard
    SmartDashboard.putData("Auto Chooser", autoChooser);

    // if (autoChooser.getSelected() == null) {
    // RobotModeTriggers.autonomous().onTrue(Commands.runOnce(drivebase::zeroGyroWithAlliance));
    // }
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be
   * created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with
   * an arbitrary predicate, or via the
   * named factories in
   * {@link edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses
   * for
   * {@link CommandXboxController
   * Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller PS4}
   * controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick
   * Flight joysticks}.
   */
  private void configureBindings() {

    driverXbox.y().whileTrue(new RunCommand(() -> m_indexer.feed(-1, 1), m_indexer));
    driverXbox.x().whileTrue(new RunCommand(() -> m_indexer.feed(0, 0), m_indexer));
    driverXbox.b().whileTrue(new RunCommand(() -> m_intake.intake_run(0.8)));
    driverXbox.a().whileTrue(new RunCommand(() -> m_intake.intake_run(0)));

    driverXbox.leftBumper().whileTrue(Commands.startEnd(
        () -> m_intake.armmove(0.2),
        () -> m_intake.armmove(0),
        m_intake));

    driverXbox.rightBumper().whileTrue(Commands.startEnd(
        () -> m_intake.armmove(-0.2),
        () -> m_intake.armmove(0),
        m_intake));

    driverXbox.pov(0).onTrue(Commands.runOnce(() -> m_shooter.setspeed(10000), m_shooter));
    driverXbox.pov(270).onTrue(Commands.runOnce(() -> m_shooter.set_speed_auto(), m_shooter));
    driverXbox.pov(90).onTrue(Commands.runOnce(() -> m_shooter.setspeed(6), m_shooter));
    driverXbox.pov(180).onTrue(Commands.runOnce(() -> m_shooter.setspeed(0), m_shooter));

    buttonBoard.button(10).whileTrue(Commands.startEnd(
        () -> m_shooter.set_speed_auto(),
        () -> m_shooter.setspeed(0),
        m_shooter));

    buttonBoard.button(9).whileTrue(Commands.startEnd(
        () -> m_indexer.feedw(-1, 0.8),
        () -> m_indexer.feed(0, 0),
        m_indexer));

    buttonBoard.button(5).whileTrue(drivebase.driveToPose(new Pose2d(2, 2, new Rotation2d())));

    buttonBoard.button(14).whileTrue(drivebase.driveTo(AutoDestination.center_field));
    // drive to RED
    buttonBoard.button(17).whileTrue(drivebase.driveTo(AutoDestination.red_hub_left));
    buttonBoard.button(16).whileTrue(drivebase.driveTo(AutoDestination.red_hub_front));
    buttonBoard.button(20).whileTrue(drivebase.driveTo(AutoDestination.red_hub_right));

    buttonBoard.button(15).whileTrue(drivebase.driveTo(AutoDestination.red_trench_left));
    buttonBoard.button(18).whileTrue(drivebase.driveTo(AutoDestination.red_trench_right));
    buttonBoard.button(22).whileTrue(drivebase.driveTo(AutoDestination.red_hump_left));
    buttonBoard.button(18).whileTrue(drivebase.driveTo(AutoDestination.red_hump_right));
    // drive to BLUE
    buttonBoard.button(2).whileTrue(drivebase.driveTo(AutoDestination.blue_hub_left));
    buttonBoard.button(3).whileTrue(drivebase.driveTo(AutoDestination.blue_hub_front));
    buttonBoard.button(5).whileTrue(drivebase.driveTo(AutoDestination.blue_hub_right));

    buttonBoard.button(1).whileTrue(drivebase.driveTo(AutoDestination.blue_trench_left));
    buttonBoard.button(7).whileTrue(drivebase.driveTo(AutoDestination.blue_trench_right));
    buttonBoard.button(4).whileTrue(drivebase.driveTo(AutoDestination.blue_hump_left));
    buttonBoard.button(6).whileTrue(drivebase.driveTo(AutoDestination.blue_hump_right));

    // // drive to commands
    // new JoystickButton(m_psoc, 10)
    // .onTrue(new PrintCommand("")
    // .andThen(drivebase.driveTo(AutoDestination.center_field)));
    // new JoystickButton(m_psoc, 11)
    // .onTrue(new PrintCommand("")
    // .andThen(drivebase.driveTo(AutoDestination.red_hub_front)));

    // // drive to commands
    // new JoystickButton(m_psoc, 10)
    // .whileTrue(drivebase.driveTo(AutoDestination.center_field));
    // new JoystickButton(m_psoc, 11)
    // .whileTrue(drivebase.driveTo(AutoDestination.red_hub_front));
    //
    // new JoystickButton(m_psoc, 13)
    // .whileTrue(drivebase.driveTo(AutoDestination.blue_hub_front));
    // new JoystickButton(m_psoc, 12)
    // .whileTrue(drivebase.driveTo(AutoDestination.blue_hub_right));

    Command driveFieldOrientedDirectAngle = drivebase.driveFieldOriented(driveDirectAngle);
    Command driveFieldOrientedAnglularVelocity = drivebase.driveFieldOriented(driveAngularVelocity);
    Command driveRobotOrientedAngularVelocity = drivebase.driveFieldOriented(driveRobotOriented);
    Command driveFieldOrientedDirectAngleKeyboard = drivebase.driveFieldOriented(driveDirectAngleKeyboard);
    Command driveFieldOrientedAnglularVelocityKeyboard = drivebase.driveFieldOriented(driveAngularVelocityKeyboard);

    if (RobotBase.isSimulation()) {
      drivebase.setDefaultCommand(driveFieldOrientedDirectAngleKeyboard);
    } else {
      drivebase.setDefaultCommand(driveFieldOrientedAnglularVelocity);
    }

    if (Robot.isSimulation()) {
      Pose2d target = new Pose2d(new Translation2d(1, 4),
          Rotation2d.fromDegrees(90));
      // drivebase.getSwerveDrive().field.getObject("targetPose").setPose(target);
      driveDirectAngleKeyboard.driveToPose(() -> target,
          new ProfiledPIDController(5,
              0,
              0,
              new Constraints(5, 2)),
          new ProfiledPIDController(5,
              0,
              0,
              new Constraints(Units.degreesToRadians(360),
                  Units.degreesToRadians(180))));
      driverXbox.start().onTrue(Commands.runOnce(() -> drivebase.resetOdometry(new Pose2d(3, 3, new Rotation2d()))));
      driverXbox.button(1).whileTrue(drivebase.sysIdDriveMotorCommand());
      driverXbox.button(2).whileTrue(Commands.runEnd(() -> driveDirectAngleKeyboard.driveToPoseEnabled(true),
          () -> driveDirectAngleKeyboard.driveToPoseEnabled(false)));

    }
    if (DriverStation.isTest()) {
      drivebase.setDefaultCommand(driveFieldOrientedAnglularVelocity); // Overrides drive command above!

      driverXbox.x().whileTrue(Commands.runOnce(drivebase::lock, drivebase).repeatedly());
      // driverXbox.start().onTrue((Commands.runOnce(drivebase::zeroGyro)));
      driverXbox.back().whileTrue(drivebase.centerModulesCommand());
      driverXbox.leftBumper().onTrue(Commands.none());
      driverXbox.rightBumper().onTrue(Commands.none());
    } else {
      // driverXbox.a().onTrue((Commands.runOnce(drivebase::zeroGyro)));
      // driverXbox.start().whileTrue(Commands.none());
      // driverXbox.back().whileTrue(Commands.none());
      // // driverXbox.leftBumper().whileTrue(Commands.runOnce(drivebase::lock,
      // // drivebase).repeatedly());
      // driverXbox.rightBumper().onTrue(Commands.none());
    }

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // // Pass in the selected auto from the SmartDashboard as our desired autnomous
    // // commmand
    return autoChooser.getSelected();
  }

  public void setMotorBrake(boolean brake) {
    drivebase.setMotorBrake(brake);
  }
}
