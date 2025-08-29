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

import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.RobotConfig;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;

/**
 * This class defines the runtime mode used by AdvantageKit. The mode is always "real" when running
 * on a roboRIO. Change the value of "simMode" to switch between "sim" (physics sim) and "replay"
 * (log replay from a file).
 */
public final class Constants {
  public static final Mode simMode = Mode.SIM;
  public static final Mode currentMode = RobotBase.isReal() ? Mode.REAL : simMode;

  public static enum Mode {
    /** Running on a real robot. */
    REAL,

    /** Running a physics simulator. */
    SIM,

    /** Replaying from a log file. */
    REPLAY
  }

  public static final class VisionConstants {
    public static AprilTagFieldLayout aprilTagLayout =
        AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);

    public static double maxAmbiguity = 0.3; // placeholder
    public static double maxZError = 0.75; // placeholder

    public static int frontCameraCanID;

    public static double linearStdDevBaseline = 0.02;
    public static double angularStdDevBaseline = 0.06;
    public static double linearStdDevMegatag2Factor = 0.5;
    public static double angularStdDevMegatag2Factor = Double.POSITIVE_INFINITY;

    // Standard deviation multipliers for each camera
    // (Adjust to trust some cameras more than others)
    public static double[] cameraStdDevFactors =
        new double[] {
          1.0, // Camera 0
          1.0 // Camera 1
        };

    public static Transform3d cameraOneToRobot =
        new Transform3d(0.0, 0.0, 0.0, new Rotation3d()); // fill with actual camera offsets
    public static Transform3d cameraTwoToRobot =
        new Transform3d(0.0, 0.0, 0.0, new Rotation3d()); // fill with actual camera offsets
  }

  public final class DriveConstants {
    public static final double deadband = 0.0;
    public static final double driveToPointStaticFrictionConstant = 0.02;
    public static final double maxSpeedMetersPerSec = 4.8;
    public static final double odometryFrequency = 100.0; // Hz

    // TODO: URGENT, get physical bot constants from mechanical
    public static final double trackWidth = Units.inchesToMeters(26.5);
    public static final double wheelBase = Units.inchesToMeters(26.5);
    public static final double driveBaseRadius = Math.hypot(trackWidth / 2.0, wheelBase / 2.0);
    public static final Translation2d[] moduleTranslations =
        new Translation2d[] {
          new Translation2d(trackWidth / 2.0, wheelBase / 2.0),
          new Translation2d(trackWidth / 2.0, -wheelBase / 2.0),
          new Translation2d(-trackWidth / 2.0, wheelBase / 2.0),
          new Translation2d(-trackWidth / 2.0, -wheelBase / 2.0)
        };

    // Zeroed rotation values for each module, see setup instructions
    public static final Rotation2d frontLeftOffset = new Rotation2d(0.0);
    public static final Rotation2d frontRightOffset = new Rotation2d(0.0);
    public static final Rotation2d backLeftOffset = new Rotation2d(0.0);
    public static final Rotation2d backRightOffset = new Rotation2d(0.0);

    // Device CAN IDs
    public static final int pigeonCanId = 9; // TODO: URGENT, switch to a nav x

    public static final int frontRightDriveCanId = 3;
    public static final int frontLeftDriveCanId = 7;
    public static final int backRightDriveCanId = 2;
    public static final int backLeftDriveCanId = 9;

    public static final int frontRightTurnCanId = 4;
    public static final int frontLeftTurnCanId = 6;
    public static final int backRightTurnCanId = 1;
    public static final int backLeftTurnCanId = 8;

    public static final boolean frontLeftTurnInverted = true;
    public static final boolean frontRightTurnInverted = true;
    public static final boolean backLeftTurnInverted = true;
    public static final boolean backRightTurnInverted = true;

    public static final boolean frontLeftDriveInverted = true;
    public static final boolean frontRightDriveInverted = true;
    public static final boolean backLeftDriveInverted = true;
    public static final boolean backRightDriveInverted = true;

    // Drive motor configuration
    public static final int driveMotorCurrentLimit = 50;
    public static final double wheelRadiusMeters = Units.inchesToMeters(1.5);
    public static final double driveMotorReduction =
        (45.0 * 22.0) / (14.0 * 15.0); // TODO: Urgent, get the gear reduction from mechanical
    public static final DCMotor driveGearbox = DCMotor.getNeo550(1);

    // Drive encoder configuration
    public static final double driveEncoderPositionFactor =
        2 * Math.PI / driveMotorReduction; // Rotor Rotations -> Wheel Radians
    public static final double driveEncoderVelocityFactor =
        (2 * Math.PI) / 60.0 / driveMotorReduction; // Rotor RPM -> Wheel Rad/Sec

    // Drive PID configuration
    public static final double driveKp = 1.0;
    public static final double driveKd = 0.0;

    public static final double driveKs = 0.16681;
    public static final double driveKv = 2.609;
    public static final double driveKa = 0.51582;
    public static final double driveSimP = 0.05;
    public static final double driveSimD = 0.0;
    public static final double driveSimKs = 0.0;
    public static final double driveSimKv = 0.0789;

    // Turn motor configuration
    public static final int turnMotorCurrentLimit = 20;
    public static final double turnMotorReduction =
        9424.0 / 203.0; // TODO: URGENT, get the gear reduction from mechanical
    public static final DCMotor turnGearbox = DCMotor.getNeo550(1);

    // Turn encoder configuration
    // TODO: URGENT, apply the gear reduction
    public static final double turnEncoderPositionFactor = 2 * Math.PI; // Rotations -> Radians
    public static final double turnEncoderVelocityFactor = (2 * Math.PI) / 60.0; // RPM -> Rad/Sec

    // Turn PID configuration
    public static final double turnKp = 0.4;
    public static final double turnKi = 0.0;
    public static final double turnKd = 0.0;
    public static final double turnSimP = 8.0;
    public static final double turnSimD = 0.0;
    public static final double turnPIDMinInput = 0; // Radians
    public static final double turnPIDMaxInput = 2 * Math.PI; // Radians

    // angle lock constants
    public static final double ANGLE_KP = 0.0;
    public static final double ANGLE_KD = 0.0;
    public static final double ANGLE_MAX_VELOCITY = 8.0;
    public static final double ANGLE_MAX_ACCELERATION = 20.0;
    public static final double FF_START_DELAY = 2.0; // Secs
    public static final double FF_RAMP_RATE = 0.1; // Volts/Sec
    public static final double WHEEL_RADIUS_MAX_VELOCITY = 0.25; // Rad/Sec
    public static final double WHEEL_RADIUS_RAMP_RATE = 0.05;

    // PathPlanner configuration
    // TODO: set this to the correct values
    public static final double robotMassKg = 74.088;
    public static final double robotMOI = 6.883;
    public static final double wheelCOF = 1.2;
    public static final RobotConfig ppConfig =
        new RobotConfig(
            robotMassKg,
            robotMOI,
            new ModuleConfig(
                wheelRadiusMeters,
                maxSpeedMetersPerSec,
                wheelCOF,
                driveGearbox.withReduction(driveMotorReduction),
                driveMotorCurrentLimit,
                1),
            moduleTranslations);
  }

  public final class PneumaticConstants {
    public static final int compressorCanID = 14;
    public static final boolean compressorInverted = false;

    public static final int compressorFanPort = 5;

    public static final int pressureSealForward = 9;
    public static final int pressureSealBackward = 10;

    public static final int shootingSeal = 8;

    public static double maxPSI = 100.0;
  }

  // POTENTIAL BAD PRACTICE WARNING
  // using static imports for readability, meaning all nested classes must be static
  public static final class MechanismConstants {

    /** rotation constants */
    public static final class RotationConstants {
      // motor config
      public static final int rotationCanID = 10;
      public static final boolean rotationInverted = false;
      public static final int rotationCurrentLimit =
          60; // 80 is generally considered too high, though it is the default value
      public static final double rotationEncoderPositionFactor = (2 * Math.PI) / 50;
      public static final double rotationEncoderVeloFactor = (2 * Math.PI) / 50 / 60;

      // feedback loop
      public static final double rotationKp = 9;
      public static final double rotationKi = 1.5;
      public static final double rotationKd = 1.25;

      // feedforward loop
      public static final double rotationFFKs = 0;
      public static final double rotationFFKv = 1.2324;
      public static final double rotationFFKa = 0.63117;

      // motion profiling
      public static final double rotationTrapezoidMaxVelo = 1; // meters per second
      public static final double rotationTrapezoidMaxAccel = 0.5; // meters per second ^2
    }
    /** elevation constants */
    public static final class ElevationConstants {
      // motor config 1
      public static final int elevationCanID = 11;
      public static final boolean elevationInverted = false;
      public static final int elevationCurrentLimit =
          60; // 80 is generally considered too high, though it is the default value
      public static final double elevationEncoderPositionFactor = (2 * Math.PI) * 0.0125;
      public static final double elevationEncoderVeloFactor = (2 * Math.PI) * 0.0125 * 0.0166;

      // motor config 2
      public static final int elevationCanIDTwo = 12;
      public static final boolean elevationTwoInverted = false;
      public static final int elevationTwoCurrentLimit = 60;

      // limit switch
      public static final int elevationLimitSwitchID = 2;

      // feedback loop
      public static final double elevationKp = 10;
      public static final double elevationKi = 0;
      public static final double elevationKd = 0;

      // feedforward loop
      public static final double elevationFFKs = .16328;
      public static final double elevationFFKg = 0.15212;
      public static final double elevationFFKv = 0.0029191;
      public static final double elevationFFKa = 0.0016397;

      // motion profiling
      public static final double elevationTrapezoidMaxVelo = 1; // meters per second
      public static final double elevationTrapezoidMaxAccel = 0.5; // meters per second^2
    }

    /** barrel constants */
    public static final class BarrelConstants {
      // motor config
      public static final int barrelCanID = 13;
      public static final boolean barrelInverted = false;
      public static final int barrelCurrentLimit =
          40; // 80 is generally considered too high, though it is the default value
      public static final double barrelEncoderPositionFactor = (2 * Math.PI) * (1 / 35.166);
      public static final double barrelEncoderVeloFactor = ((2 * Math.PI) * (1 / 35.166) * 0.0166);

      // feedback loop
      public static final double barrelKp = 15;
      public static final double barrelKi = .25;
      public static final double barrelKd = .7;

      // feedforward loop
      public static final double barrelFFKs = 0;
      public static final double barrelFFKv = 2.0129;
      public static final double barrelFFKa = 4.5878;

      // motion profiling
      public static final double barrelTrapezoidMaxVelo = 1; // meters per second
      public static final double barrelTrapezoidMaxAccel = 0.5; // meters per second^2
    }
  }
}
