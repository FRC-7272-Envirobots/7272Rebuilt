package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.DegreesPerSecondPerSecond;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Pounds;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.RobotConstants;
import yams.mechanisms.config.ArmConfig;
import yams.mechanisms.positional.Arm;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;
import yams.motorcontrollers.remote.TalonFXWrapper;

public class Intake_Subsystem extends SubsystemBase {

  private final TalonFX m_intakespin = new TalonFX(RobotConstants.IntakeCan);

  private final TalonFX armTalon;
  private final SmartMotorController armYAMS;
  private final Arm arm;

  private final DigitalInput downLimitSwitchDIO = new DigitalInput(5);

  private static final Angle ANGLE_DOWN = Degrees.of(-5);
  private static final Angle ANGLE_UP = Degrees.of(129);

  public Intake_Subsystem() {
    this.armTalon = new TalonFX(RobotConstants.ArmCan);

    final SmartMotorControllerConfig armMotorConfig = new SmartMotorControllerConfig(this)
        .withControlMode(ControlMode.CLOSED_LOOP)
        .withClosedLoopController(100, 0, 0, DegreesPerSecond.of(360), DegreesPerSecondPerSecond.of(360))
        // .withClosedLoopRampRate(Seconds.of(.25))
        .withFeedforward(new ArmFeedforward(0, 0, 0, 0))
        // .withSoftLimit(ANGLE_DOWN, ANGLE_UP)
        .withStartingPosition(ANGLE_UP)
        .withGearing(42)
        .withIdleMode(MotorMode.BRAKE)
        .withStatorCurrentLimit(Amps.of(40))
        .withTelemetry("IntakeArmMotor",
            SmartMotorControllerConfig.TelemetryVerbosity.HIGH);

    this.armYAMS = new TalonFXWrapper(armTalon, DCMotor.getFalcon500(1),
        armMotorConfig);

    this.arm = new Arm(new ArmConfig()
        .withSmartMotorController(armYAMS)
        .withLength(Inches.of(9))
        .withMass(Pounds.of(5))
        .withStartingPosition(ANGLE_UP)
        .withTelemetry("IntakeArm",
            SmartMotorControllerConfig.TelemetryVerbosity.HIGH));

  }

  @Override
  public void periodic() {
    // if (downLimitSwitchDIO.get()) {

    // }
    SmartDashboard.putBoolean("Intake/At Limit", downLimitSwitchDIO.get());

    SmartDashboard.putNumber("Intake/Raw Motor Rotations", armTalon.getPosition().getValueAsDouble());

    arm.updateTelemetry();
    SmartDashboard.putNumber("Intake/Adjusted Angle degrees",
        armYAMS.getMechanismPosition().in(Degrees));
    SmartDashboard.putNumber("Intake/Velocity (deg / sec)",
        armYAMS.getMechanismVelocity().in(DegreesPerSecond));

  }

  // private static final double ARM_DRIVE_SPEED = 0.1;

  // public Command driveArmUp() {
  // return Commands.startEnd(
  // () -> armTalon.set(ARM_DRIVE_SPEED),
  // () -> armTalon.set(0),
  // this);
  // }

  // public Command driveArmDown() {
  // return Commands.startEnd(
  // () -> armTalon.set(ARM_DRIVE_SPEED * -1),
  // () -> armTalon.set(0),
  // this);
  // }

  public Command setArmPositionUp() {
    return arm.setAngle(ANGLE_UP);
  }

  public Command setArmPositionDown() {
    return arm.setAngle(ANGLE_DOWN);
  }

  public Command setArmPositionMid() {
    return arm.setAngle(Degrees.of(65));
  }

  // public Command stopArm() {
  // return arm.set(0);
  // }

  public Command intake_run(double speed) {
    m_intakespin.set(speed);
    return null;
  }
}
