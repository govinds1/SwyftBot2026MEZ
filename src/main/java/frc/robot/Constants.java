// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.config.RobotConfig;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static final class DriveConstants {
    // Driving Parameters - Note that these are not the maximum capable speeds of
    // the robot, rather the allowed maximum speeds
    public static final double kMaxSpeedMetersPerSecond = 4.8;
    public static final double kMaxAngularSpeedRadiansPerSecond = 2 * Math.PI; // radians per second

    // Chassis configuration
    public static final double kTrackWidth = Units.inchesToMeters(23.5); // Distance between centers of right and left wheels on robot
    public static final double kWheelBase = Units.inchesToMeters(23.5); // Distance between front and back wheels on robot
    public static final double kBumperWidth = Units.inchesToMeters(3); // Width of a single bumper.
    public static final double kFullWidth = Units.inchesToMeters(27) + (kBumperWidth * 2.0); // Width from bumper to bumper
    public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
        new Translation2d(kWheelBase / 2, kTrackWidth / 2),
        new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
        new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
        new Translation2d(-kWheelBase / 2, -kTrackWidth / 2));

    // Angular offsets of the modules relative to the chassis in radians
    public static final double kFrontLeftChassisAngularOffset = -Math.PI / 2;
    public static final double kFrontRightChassisAngularOffset = 0;
    public static final double kBackLeftChassisAngularOffset = Math.PI;
    public static final double kBackRightChassisAngularOffset = Math.PI / 2;

    // Spark Max CAN IDs
    // TODO: Update as necessary.
    public static final int kFrontLeftDrivingCanId = 11;
    public static final int kRearLeftDrivingCanId = 13;
    public static final int kFrontRightDrivingCanId = 15;
    public static final int kRearRightDrivingCanId = 17;

    public static final int kFrontLeftTurningCanId = 10;
    public static final int kRearLeftTurningCanId = 12;
    public static final int kFrontRightTurningCanId = 14;
    public static final int kRearRightTurningCanId = 16;

    public static final boolean kGyroReversed = false;

    // RobotConfig for PathPlanner
    public static RobotConfig config;
  }

  public static final class ModuleConstants {
    // The MAXSwerve module can be configured with one of three pinion gears: 12T,
    // 13T, or 14T. This changes the drive speed of the module (a pinion gear with
    // more teeth will result in a robot that drives faster).
    public static final int kDrivingMotorPinionTeeth = 14;

    // Calculations required for driving motor conversion factors and feed forward
    public static final double kDrivingMotorFreeSpeedRps = NeoMotorConstants.kFreeSpeedRpm / 60;
    public static final double kWheelDiameterMeters = 0.0762;
    public static final double kWheelCircumferenceMeters = kWheelDiameterMeters * Math.PI;
    // 45 teeth on the wheel's bevel gear, 22 teeth on the first-stage spur gear, 15
    // teeth on the bevel pinion
    public static final double kDrivingMotorReduction = (45.0 * 22) / (kDrivingMotorPinionTeeth * 15);
    public static final double kDriveWheelFreeSpeedRps = (kDrivingMotorFreeSpeedRps * kWheelCircumferenceMeters)
        / kDrivingMotorReduction;

    // PID Values
    public static final double kDrivingPController = 0.04;
    public static final double kTurningPController = 1;
  }

  public static final class ShooterConstants {
    public static final int kLeftShooterMotorCanId = 21;
    public static final int kRightShooterMotorCanId = 22;
    public static final int kFeederMotorCanId = 23;
    public static final double kShooterPower = 0.5;
    public static final double kFeederPower = 0.6;
    public static final double kShooterSpinUpTime = 0.25;

    // RPM Control
    public static final double kShootingRPM = 6000; // TODO: Update as necessary. Make sure to properly calculate the kMotorReduction value below, and tune the PID and FF values.
    public static final double kPController = 0; // TODO: TUNE THIS AFTER VFF! Increment very slowly (starting at 0.00001), until shooter reachers 100% of the desired RPM and 'bounces back' quickly without any oscillation ("jerky" movement).
    public static final double kVFeedForward = 0.0002; // TODO: TUNE THIS FIRST! Increment very slowly until shooter reaches about 90-95% of the desired RPM.
    public static final double kRPMTolerance = 500; // TODO: Update as necessary.

    // TODO: Modify these buttons if necessary.
    public static final int kRunShooterButton = 1; // A button
    public static final int kStopShooterButton = 4; // Y button

    // TODO: Calculate shooting motor reduction. Conversion from motor shaft rotations to flywheel rotations. This necessary for velocity control.
    public static final double kMotorReduction = (45.0 * 22) / (10 * 15);
  }

  public static final class IntakeConstants {
    public static final int kIntakeRollerMotorCanId = 31;
    public static final int kIntakeExtenderMotorCanId = 32;
    public static final double kIntakeRollerSpeed = 0.7;
    public static final double kIntakeExtenderExtendSpeed = 0.1;
    public static final double kIntakeExtenderRetractSpeed = 0.2;
    public static final double kIntakeExtendTime = 1.0;
    public static final double kIntakeRetractTime = 1.8;
    public static final double kIntakeAgitateTime = 0.75;

    public static final double kIntakeVelocity = 2300; // Encoder units.

    public static final int kRollerRunButton = 2; // B button
    public static final int kRollerReverseButton = 3; // X button
    public static final int kExtenderOutAxis = 2; // Left trigger
    public static final int kExtenderInAxis = 3; // Right trigger
  }

  public static final class ClimberConstants {
    public static final int kClimberMotorCanId = 41;
    public static final double kClimbSpeed = 0.5;

    public static final double kClimberUpPosition = 100; // TODO: Update. Encoder units - check dashboard after raising to some height manually.

    public static final int kClimberUpButton = 5; // Left bumper
    public static final int kClimberDownButton = 6; // Right bumper 
  }

  public static final class OperatorConstants {
    public static final int kDriverControllerPort = 0;
    public static final int kOperatorControllerPort = 1;
    public static final double kDriveDeadband = 0.05;

    public static final int kResetGyroButton = 8; // Start button
    public static final int kHalfSpeedAxis = 2; // Left trigger
  }

  public static final class DriveAutoConstants {
    public static final double kMaxSpeedMetersPerSecond = 3;
    public static final double kMaxAccelerationMetersPerSecondSquared = 3;
    public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
    public static final double kMaxAngularAccelerationRadiansPerSecondSquared = Math.PI;

    public static final double kPXYController = 4;
    public static final double kPThetaController = 2;
    public static final Pose2d kRobotControllerTolerance = new Pose2d(0.1, 0.1, new Rotation2d(Units.degreesToRadians(1)));

    // Constraint for the motion profiled robot angle controller
    public static final TrapezoidProfile.Constraints kThetaControllerConstraints = new TrapezoidProfile.Constraints(
        kMaxAngularSpeedRadiansPerSecond, kMaxAngularAccelerationRadiansPerSecondSquared);
  }

  public static final class NeoMotorConstants {
    public static final double kFreeSpeedRpm = 5676;
  }
}