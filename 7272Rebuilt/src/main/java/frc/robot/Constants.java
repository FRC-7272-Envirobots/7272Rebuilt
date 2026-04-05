// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Map;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
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
  public static final double MOMENT_OF_INERTIA = 6.883;
  public static final double LOOP_TIME = 0.13; // s, 20ms + 110ms sprk max velocity lag
  public static final double MAX_SPEED = Units.feetToMeters(14.5); // was 14.5
  // Maximum speed of the robot in meters per second, used to limit acceleration.

  public static final class DrivebaseConstants {
    // Hold time on motor brakes when disabled
    public static final double WHEEL_LOCK_TIME = 10; // seconds
  }

  public static class OperatorConstants {

    // Joystick Deadband
    public static final double DEADBAND = 0.1;
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

  public static final class VisionConstants {
    public static final String OUTTAKE_LIMELIGHT_NAME = "limelight-outtake";
  }

  public static final class AutoConstants {

    public static final Map<AutoDestination, Pose2d> Auto_Map = Map.ofEntries(
        Map.entry(AutoDestination.center_field, new Pose2d(8.288, 4.058, Rotation2d.fromDegrees(-90))),

        // blue map
        // hub
        Map.entry(AutoDestination.blue_hub_left, new Pose2d(2.362, 5.967, Rotation2d.fromDegrees(-40.678))),
        Map.entry(AutoDestination.blue_hub_front, new Pose2d(2.469, 3.937, Rotation2d.fromDegrees(-0))),
        Map.entry(AutoDestination.blue_hub_right, new Pose2d(2.362, 1.944, Rotation2d.fromDegrees(30.511))),
        // trench/hump
        Map.entry(AutoDestination.blue_hump_left, new Pose2d(4.680, 5.593, Rotation2d.fromDegrees(0))),
        Map.entry(AutoDestination.blue_hump_right, new Pose2d(4.680, 2.500, Rotation2d.fromDegrees(0))),
        Map.entry(AutoDestination.blue_trench_left, new Pose2d(4.603, 7.401, Rotation2d.fromDegrees(0))),
        Map.entry(AutoDestination.blue_trench_right, new Pose2d(4.603, 0.622, Rotation2d.fromDegrees(0))),

        // red map
        // hub
        Map.entry(AutoDestination.red_hub_left, new Pose2d(14.299, 2.180, Rotation2d.fromDegrees(141.302))),
        Map.entry(AutoDestination.red_hub_front, new Pose2d(14.080, 4.014, Rotation2d.fromDegrees(180))),
        Map.entry(AutoDestination.red_hub_right, new Pose2d(14.124, 5.565, Rotation2d.fromDegrees(-140.818))),
        // trench/hump
        Map.entry(AutoDestination.red_hump_left, new Pose2d(11.975, 2.448, Rotation2d.fromDegrees(0))),
        Map.entry(AutoDestination.red_hump_right, new Pose2d(11.975, 5.712, Rotation2d.fromDegrees(0))),
        Map.entry(AutoDestination.red_trench_left, new Pose2d(11.871, 0.661, Rotation2d.fromDegrees(0))),
        Map.entry(AutoDestination.red_trench_right, new Pose2d(11.975, 7.448, Rotation2d.fromDegrees(0))));
  }

  public static final class FieldConstants {
    public static final Pose2d REDHUB_POSE2D = new Pose2d(new Translation2d(11.914, 4.04), new Rotation2d(0));
    public static final Pose2d BLUEHUB_POSE2D = new Pose2d(new Translation2d(4.621, 4.04), new Rotation2d(0));

  }
}