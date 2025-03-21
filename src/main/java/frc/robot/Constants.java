// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

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
    public static final double kMaxAngularSpeed = 2 * Math.PI; // radians per second

    // Chassis configuration
    public static final double kTrackWidth = Units.inchesToMeters(26.5);
    // Distance between centers of right and left wheels on robot
    public static final double kWheelBase = Units.inchesToMeters(26.5);
    // Distance between front and back wheels on robot
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

    // SPARK MAX CAN IDs
    public static final int kFrontLeftDrivingCanId = 7;
    public static final int kRearLeftDrivingCanId = 1;
    public static final int kFrontRightDrivingCanId = 5;
    public static final int kRearRightDrivingCanId = 3;

    public static final int kFrontLeftTurningCanId = 8;
    public static final int kRearLeftTurningCanId = 2;
    public static final int kFrontRightTurningCanId = 6;
    public static final int kRearRightTurningCanId = 4;

    public static final boolean kGyroReversed = false;
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
  }

  public static final int kDriverControllerPort = 0;
  public static final double kDriveDeadband = 0.05;

  public static final class AutoConstants {
    public static final double kMaxSpeedMetersPerSecond = 3;
    public static final double kMaxAccelerationMetersPerSecondSquared = 3;
    public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
    public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;

    public static final double kPXController = 1;
    public static final double kPYController = 1;
    public static final double kPThetaController = 1;

    // Constraint for the motion profiled robot angle controller
    public static final TrapezoidProfile.Constraints kThetaControllerConstraints = new TrapezoidProfile.Constraints(
        kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
  }

  public static final class NeoMotorConstants {
    public static final double kFreeSpeedRpm = 5676;
  }

  // Spark MAX CAN IDs //
  public static final int ALGAE_BAR_MOTOR_ID = 10; // Algae arm 

  public static final int ALGAE_ARM_MOTOR_ID = 12; // Algae arm
  
  public static final int ELEVATOR_MOTOR_ID = 11; // Tower
  
  // public static final int ELEVATOR_STAGE_2_MOTOR_ID = 20; 

  public static final int EFFECTOR_WHEEL_MOTOR_ID = 13; 

  public static final int EFFECTOR_ARM_MOTOR_ID = 9; 


  // PWM Ports //
  public static final int LED_PWM_ID = 4;

  // DIO (Digital Input/Output) Channels //
  // Example: public static final int RIGHT_ENCODER_CHANNEL_A = 0;
  // Example: public static final int RIGHT_ENCODER_CHANNEL_B = 1;
  // Example: public static final int LEFT_ENCODER_CHANNEL_A = 2;
  // Example: public static final int LEFT_ENCODER_CHANNEL_B = 3;
  
  public static final double ARM_SPEED = 0.2;
  public static final double WHEEL_SPEED = 0.6;
  public static final double ARM_GRAVITY_CONST = -0.03; 
  
  // Intake Constants
  public static final double INTAKE_LIFT_GEAR_RATIO = 3 * 7 * 7 * 48 / 29;

  public static final double INTAKE_ARM_MAX_POWER = 0.1;
  public static final double INTAKE_ARM_MIN_POWER = 0.05;
  public static final double INTAKE_ARM_kP = 0.025;

  public static final double INTAKE_DEPLOY_LIMIT = 51;
  public static final double INTAKE_RETURN_LIMIT = 0;

  public static final boolean INTAKE_ARM_INVERT = true;
  
  public static final double ALGAE_ARM_UP = 0.2; // Increase from 0.1
  public static final double ALGAE_ARM_DOWN = -0.2; // Increase from 0.1

  public static final boolean INTAKE_BAR_INVERT = false;
  public static final double INTAKE_BAR_SPEED = 0.8;

  public static final boolean EFFECTOR_WHEEL_INVERT = true;

  public static final double PICK_UP_ALGAE_POSITION = 33;
  public static final double HOLD_ALGAE_POSITION = 2.0;
  public static final double PICK_UP_CORAL_POSITION = 53;
  public static final double HOLD_CORAL_POSITION = 24;

  public static final double ALGAE_ARM_GRAVITY_RESISTANCE = 0.05;
   // Power settings for the wheel
   public final static double WHEEL_FORWARD = 0.6;  // Positive power for forward
   public final static double WHEEL_REVERSE = -0.6; // Negative power for reverse

   public final static double CORAL_ARM_FORWARD = 0.15; // Power for the coral arm
   public final static double CORAL_ARM_REVERSE = -0.15; // Power for the coral arm

  // REV PH Channels //
  // Example: public static final int EXTENSION_SOLENOID_ID = 0;

  // Rev PDH Constants //
/*
  public static final int INTAKE_BAR_MOTOR_PDH_CHANNEL = 1; 
  public static final int INTAKE_DEPLOY_MOTOR_PDH_CHANNEL = 2; 
  public static final int ELEVATOR_STAGE_1_MOTOR_PDH_CHANNEL = 3; 
  public static final int ELEVATOR_STAGE_2_MOTOR_PDH_CHANNEL = 4; 
  public static final int ELEVATOR_ARM_MOTOR_PDH_CHANNEL = 5; 
  public static final int ELEVATOR_WHEEL_MOTOR_PDH_CHANNEL = 6; 
  public static final int LEFT_FRONT_DRIVE_MOTOR_PDH_CHANNEL = 11; 
  public static final int RIGHT_FRONT_DRIVE_MOTOR_PDH_CHANNEL = 10;
  public static final int LEFT_BACK_DRIVE_MOTOR_PDH_CHANNEL = 12; 
  public static final int RIGHT_BACK_DRIVE_MOTOR_PDH_CHANNEL = 13; 
 */

}