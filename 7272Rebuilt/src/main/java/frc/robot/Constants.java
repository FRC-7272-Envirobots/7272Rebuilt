// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;

import java.util.Map;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import swervelib.math.Matter;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean constants. This
 * class should not be used for any other purpose. All constants should be
 * declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

  public static final double ROBOT_MASS = (148 - 20.3) * 0.453592; // 32lbs * kg per pound
  public static final Matter CHASSIS = new Matter(new Translation3d(0, 0, Units.inchesToMeters(8)), ROBOT_MASS);
  public static final double LOOP_TIME = 0.13; // s, 20ms + 110ms sprk max velocity lag
  public static final double MAX_SPEED = Units.feetToMeters(14.5);
  // Maximum speed of the robot in meters per second, used to limit acceleration.

  // public static final class AutonConstants
  // {
  //
  // public static final PIDConstants TRANSLATION_PID = new PIDConstants(0.7, 0,
  // 0);
  // public static final PIDConstants ANGLE_PID = new PIDConstants(0.4, 0, 0.01);
  // }

  public static final class DrivebaseConstants {

    // Hold time on motor brakes when disabled
    public static final double WHEEL_LOCK_TIME = 10; // seconds
  }

  public static final class DriveConstants {
    public static final double MASS_KG = 31;
    public static final double MOMENT_OF_INERTIA = 6.883;

    public static final double DRIVE_LIMITER = 2;
    public static final double ELEVATOR_LIMITER = 60;

    // Driving Parameters - Note that these are not the maximum capable speeds of
    // the robot, rather the allowed maximum speeds
    public static final double kMaxSpeedMetersPerSecond = 4.8;
    public static final double kMaxAngularSpeed = 2 * Math.PI; // radians per second

    // Chassis configuration
    public static final double kTrackWidth = Units.inchesToMeters(27.0 - (2.0 * 1.75));
    // Distance between centers of right and left wheels on robot
    public static final double kWheelBase = Units.inchesToMeters(32.0 - (2.0 * 1.75));
    // Distance between front and back wheels on robot

    public static final Translation2d[] moduleTranslations = {
        new Translation2d(kWheelBase / 2.0, kTrackWidth / 2.0),
        new Translation2d(kWheelBase / 2.0, -kTrackWidth / 2.0),
        new Translation2d(-kWheelBase / 2.0, kTrackWidth / 2.0),
        new Translation2d(-kWheelBase / 2.0, -kTrackWidth / 2.0)
    };
  }

  public static final class LauncherConstants {
    public static final double luchagl = 75;
    public static final double shooterht = 0.5;
    public static final double gravity = 9.806;

  }

  public static final class ModuleConstants {
    // The MAXSwerve module can be configured with one of three pinion gears: 12T,
    // 13T, or 14T. This changes the drive speed of the module (a pinion gear with
    // more teeth will result in a robot that drives faster).
    public static final int kDrivingMotorPinionTeeth = 13;

    // Calculations required for driving motor conversion factors and feed forward
    public static final double kDrivingMotorFreeSpeedRps = 100;
    public static final double kWheelDiameterMeters = 0.0762;
    public static final double kWheelCircumferenceMeters = kWheelDiameterMeters * Math.PI;
    // 45 teeth on the wheel's bevel gear, 22 teeth on the first-stage spur gear, 15
    // teeth on the bevel pinion
    public static final double kDrivingMotorReduction = (45.0 * 22) / (kDrivingMotorPinionTeeth * 15);
    public static final double kDriveWheelFreeSpeedRps = (kDrivingMotorFreeSpeedRps * kWheelCircumferenceMeters)
        / kDrivingMotorReduction;

    public static final double WHEEL_COEFFICIENT_OF_FRICTION = 0.7;
    public static final double DRIVE_CURRENT_LIMIT = 50;
  }

  public static class OperatorConstants {

    // Joystick Deadband
    public static final double DEADBAND = 0.1;
    public static final double LEFT_Y_DEADBAND = 0.1;
    public static final double RIGHT_X_DEADBAND = 0.1;
    public static final double TURN_CONSTANT = 6;
  }

  public static class RobotConstants {
    public static final int spindexerCan = 15;
    public static final int IndexerCan = 16;
    public static final int ArmCan = 17;
    public static final int IntakeCan = 18;
    public static final int ShooterRcan = 19;
    public static final int ShooterLcan = 20;
  }

  public static class SwerveConstant {

  }

  public static final class VisionConstants {
    // public static final String OUTTAKE_LIMELIGHT_NAME = "limelight-outtake";
    // public static Distance OUTTAKE_LIMELIGHT_X_OFFSET = Distance.ofBaseUnits(14,
    // Inches); // front to back from center
    // public static Distance OUTTAKE_LIMELIGHT_Y_OFFSET = Distance.ofBaseUnits(3,
    // Inches); // left to right from center
    // public static Distance OUTTAKE_LIMELIGHT_Z_OFFSET = Distance.ofBaseUnits(9,
    // Inches);
    // public static Angle OUTTAKE_LIMELIGHT_ROLL_ANGLE = Angle.ofBaseUnits(0,
    // Degrees);
    // public static Angle OUTTAKE_LIMELIGHT_PITCH_ANGLE = Angle.ofBaseUnits(-45,
    // Degrees);
    // public static Angle OUTTAKE_LIMELIGHT_YAW_ANGLE = Angle.ofBaseUnits(0,
    // Degrees);
    public static final String OUTTAKE_LIMELIGHT_NAME = "limelight-outtake";
    public static Distance OUTTAKE_LIMELIGHT_X_OFFSET = Inches.of(11.700482); // front to back from center
    public static Distance OUTTAKE_LIMELIGHT_Y_OFFSET = Inches.of(-8.009224); // left to right from center
    public static Distance OUTTAKE_LIMELIGHT_Z_OFFSET = Inches.of(13.187444);
    public static Angle OUTTAKE_LIMELIGHT_ROLL_ANGLE = Degrees.of(0);
    public static Angle OUTTAKE_LIMELIGHT_PITCH_ANGLE = Degrees.of(15);
    public static Angle OUTTAKE_LIMELIGHT_YAW_ANGLE = Degrees.of(0);
    public static final boolean kGyroReversed = false;
  }

  public static final Map<AutoDestination, Pose2d> blueAuto_Map = Map.ofEntries(
  Map.entry(AutoDestination.center_field, new Pose2d(6.059, 4.305, Rotation2d.fromDegrees(-84.685)))

  );

  public static final class FieldConstants {
    public static final Pose2d REDHUB_POSE2D = new Pose2d(new Translation2d(11.914, 4.04), new Rotation2d());
    public static final Pose2d BLUEHUB_POSE2D = new Pose2d(new Translation2d(4.621, 4.04), new Rotation2d());
   
  }
}