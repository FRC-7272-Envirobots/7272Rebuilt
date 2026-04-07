package frc.robot.subsystems;

import java.util.function.Supplier;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.FieldConstants;

public class Shooter_Subsystem extends SubsystemBase {
  private static final String PREFERENCE_KEY_GRAVITY = "Shooter/Gravity";
  private static final String PREFERENCE_KEY_ANGLE = "Shooter/Angle";
  private static final String PREFERENCE_KEY_HEIGHT = "Shooter/Height";

  private static final double PREFERENCE_DEFAULT_GRAVITY = 10.0;
  private static final double PREFERENCE_DEFAULT_ANGLE = 75.0;
  private static final double PREFERNCE_DEFAULT_HEIGHT = 0.5;

  TalonFX m_LeftShooter = new TalonFX(frc.robot.Constants.RobotConstants.ShooterLcan);
  TalonFX m_RightShooter = new TalonFX(frc.robot.Constants.RobotConstants.ShooterRcan);
  Follower right_Follower;
  TalonFXConfiguration config;
  private final Supplier<Pose2d> positionSupplier;

  public Shooter_Subsystem(Supplier<Pose2d> positionSupplier) {
    this.positionSupplier = positionSupplier;

    Preferences.initDouble(PREFERENCE_KEY_GRAVITY, PREFERENCE_DEFAULT_GRAVITY);
    Preferences.initDouble(PREFERENCE_KEY_ANGLE, PREFERENCE_DEFAULT_ANGLE);
    Preferences.initDouble(PREFERENCE_KEY_HEIGHT, PREFERNCE_DEFAULT_HEIGHT);

    // m_RightShooter.setControl(new Follower(m_LeftShooter.getDeviceID(), null));
    // right_Follower = new Follower(RobotConstants.ShooterLcan, null);
    // m_RightShooter.setControl(right_Follower);

    config = new TalonFXConfiguration();
    config.Slot0.kP = 0.3;
    // "Nobody uses I" apparently, so dont set it.
    config.Slot0.kD = 0.0087045;
    config.Slot0.kS = 0.034662;
    config.Slot0.kV = 2.00919;
    config.Slot0.kA = 0.0015569;

    // Zero means no limit to cruise velocity
    config.MotionMagic.MotionMagicCruiseVelocity = 1000; // changed from 250 (12 inches / second) * (1 sprocket
                                                         // rotation / 1.8 inches
                                                         // ) * (90 motor rotations / sprocket rotation) = ~191
                                                         // motor
                                                         // roations / second.8 inches /
    config.MotionMagic.MotionMagicAcceleration = config.MotionMagic.MotionMagicCruiseVelocity * 2 * 100; //
    // .5
    // seconds
    // to
    // reach
    // full
    // speed
    config.MotionMagic.MotionMagicJerk = config.MotionMagic.MotionMagicAcceleration * 10 * 50; // spread jerk over
                                                                                               // .1

    m_LeftShooter.getConfigurator().apply(config);

  }

  public Command autoSpeedTilCanceled() {
    return this.run(() -> set_speed_auto()).finallyDo(() -> setspeed(0));
  }

  public Command autoSpeedForever() {
    return runOnce(() -> setDefaultCommand(autoSpeedTilCanceled()));
  }

  public Command stop() {
    return runOnce(() -> {
      this.setspeed(0);
      this.removeDefaultCommand();
    });
  }

  public Command runManualSpeed(double speed) {
    return this.startEnd(
        () -> this.setspeed(speed),
        () -> this.setspeed(0));
  }

  // public void setshooter(AngularVelocity speed) {
  // final MotionMagicVelocityDutyCycle m_request = new
  // MotionMagicVelocityDutyCycle0;``
  // m_LeftShooter.setControlPrivate(m_request.); }

  public final void setspeed(double velocityVoltage) {
    // System.out.printf("Setting shooter speed %s\n", velocityVoltage);
    MotionMagicVelocityVoltage m_request = new MotionMagicVelocityVoltage(velocityVoltage * -1);
    m_LeftShooter.setControl(m_request);
    m_RightShooter.setControl(m_request);

  }

  public double getshooterspeed(double distance) {
    // v\left(x\right)=\sqrt{\frac{x^{2}\cdot9.806}{x\sin\left(2\cdot
    // a_{1}\right)-2\cdot h_{1}\cos\left(a_{1}\right)^{2}}}
    // shootervel = Math.sqrt(
    // (Math.pow(distSupplier,2)*LauncherConstants.gravity)/
    // (distSupplier*Math.sin(2*LauncherConstants.luchagl)-(2*LauncherConstants.shooterht*(Math.pow(Math.cos(distSupplier),2))))

    // );

    double shooterGravity = Preferences.getDouble(PREFERENCE_KEY_GRAVITY, PREFERENCE_DEFAULT_GRAVITY);
    double shooterAngle = Preferences.getDouble(PREFERENCE_KEY_ANGLE, PREFERENCE_DEFAULT_ANGLE);
    double shooterHeight = Preferences.getDouble(PREFERENCE_KEY_HEIGHT, PREFERNCE_DEFAULT_HEIGHT);

    double numerator = (Math.pow(distance, 2) * shooterGravity);

    double denominator = ((distance * Math.sin(2 * Units.degreesToRadians(shooterAngle))
        - (2 * shooterHeight
            * (Math.pow(Math.cos(Units.degreesToRadians(shooterAngle)), 2)))));

    double shootervel = Math.sqrt(numerator / denominator);
    return shootervel;
  }

  private void set_speed_auto() {

    // get our position
    Pose2d ourPosition = positionSupplier.get();

    // get our alliance's hub's position
    Pose2d hubPosition;
    if (DriverStation.getAlliance().get() == Alliance.Blue) {
      hubPosition = FieldConstants.BLUEHUB_POSE2D;
    } else {
      hubPosition = FieldConstants.REDHUB_POSE2D;
    }

    // calcualate distance between current position and our alliance's hub
    Transform2d transform = new Transform2d(ourPosition, hubPosition);
    double distance = transform.getTranslation().getNorm();

    // set speed based on distance
    double shootervel = Math.min(getshooterspeed(Math.abs(distance)) * 1.3, 1.0);
    setspeed(shootervel);
    // System.out.printf("distance : %f shooter speed %f\n", distance, shootervel);

  }

}
