// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.Constants;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.CoralArmSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.CoralEffectorSubsystem;

import java.util.List;

/**
 * Factory class for creating autonomous command sequences using 
 * the modern Commands factory approach.
 */
public final class AutonCommands {
    
    private AutonCommands() {
        // Utility class - prevent instantiation
    }
    
    /********************************************************************
     * Creates a command that drives the robot forward a specified distance.
     *
     * @param driveSubsystem The drive subsystem to use
     * @return A command that will drive the robot forward
     *********************************************************************/

    public static Command driveForward(DriveSubsystem driveSubsystem) {
        // Create trajectory configuration
        TrajectoryConfig config = new TrajectoryConfig(
            AutoConstants.kMaxSpeedMetersPerSecond,
            AutoConstants.kMaxAccelerationMetersPerSecondSquared
        ).setKinematics(DriveConstants.kDriveKinematics);

        // Create a simple forward trajectory
        Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
            new Pose2d(0, 0, new Rotation2d(0)),
            List.of(),
            new Pose2d(1.2, 0, new Rotation2d(0)),
            config
        );

        // Create PID controllers for trajectory following
        var xController = new PIDController(AutoConstants.kPXController, 0, 0);
        var yController = new PIDController(AutoConstants.kPYController, 0, 0);
        var thetaController = new ProfiledPIDController(
            AutoConstants.kPThetaController, 0, 0, 
            AutoConstants.kThetaControllerConstraints
        );
        thetaController.enableContinuousInput(-Math.PI, Math.PI);

        // Create the swerve controller command
        SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(
            trajectory,
            driveSubsystem::getPose,
            DriveConstants.kDriveKinematics,
            xController,
            yController,
            thetaController,
            driveSubsystem::setModuleStates,
            driveSubsystem
        );

        // Build and return the command
        return Commands.sequence(
            // Reset odometry to trajectory starting position
            Commands.runOnce(() -> driveSubsystem.resetOdometry(trajectory.getInitialPose()), driveSubsystem),
            // Execute the trajectory
            swerveControllerCommand,
            // Stop the robot
            Commands.runOnce(() -> driveSubsystem.drive(0, 0, 0, false), driveSubsystem)
        );
    }

    /********************************************************************
     * Creates a command that:
     * 1. Drives the robot forward
     * 2. Rotates the coral arm forward for 0.5 seconds
     * 3. Runs the end effector motor for 0.5 seconds
     *
     * @param driveSubsystem The drive subsystem to use
     * @param armSubsystem The arm subsystem to use
     * @param endEffectorSubsystem The end effector subsystem to use
     * @return A command sequence that performs all three actions
     ********************************************************************/
    public static Command scoreAndDrive(
            DriveSubsystem driveSubsystem,
            CoralArmSubsystem armSubsystem,
            CoralEffectorSubsystem endEffectorSubsystem) {
        
        return Commands.sequence(
            // First drive forward
            driveForward(driveSubsystem),
            
            // Then rotate the arm for 0.5 seconds
            Commands.run(
                () -> armSubsystem.setCoralArmPower(Constants.CORAL_ARM_FORWARD),
                armSubsystem
            ).withTimeout(0.35),
            
            // Then run the end effector for 0.5 seconds
            Commands.run(
                () -> endEffectorSubsystem.setWheelSpeed(Constants.WHEEL_FORWARD),
                endEffectorSubsystem
            ).withTimeout(0.5),
            
            // Finally, ensure everything is stopped
            Commands.parallel(
                Commands.runOnce(() -> driveSubsystem.drive(0, 0, 0, false), driveSubsystem),
                Commands.runOnce(() -> armSubsystem.setCoralArmPower(0), armSubsystem),
                Commands.runOnce(() -> endEffectorSubsystem.setWheelSpeed(0), endEffectorSubsystem)
            )
        );
    }
    

/**************************************************** */

    /**
     * Creates a custom autonomous routine that can be configured with parameters.
     * This demonstrates how to make more flexible autonomous routines.
     *
     * @param driveSubsystem The drive subsystem
     * @param armSubsystem The arm subsystem
     * @param endEffectorSubsystem The end effector subsystem
     * @param driveDistance The distance to drive in meters
     * @param armTime How long to move the arm in seconds
     * @param wheelTime How long to run the wheel in seconds
     * @return A customized autonomous command
     */
    public static Command customRoutine(
            DriveSubsystem driveSubsystem,
            CoralArmSubsystem armSubsystem,
            CoralEffectorSubsystem endEffectorSubsystem,
            double driveDistance,
            double armTime,
            double wheelTime) {
        
        // Create trajectory configuration
        TrajectoryConfig config = new TrajectoryConfig(
            AutoConstants.kMaxSpeedMetersPerSecond,
            AutoConstants.kMaxAccelerationMetersPerSecondSquared
        ).setKinematics(DriveConstants.kDriveKinematics);

        // Create a trajectory with the custom distance
        Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
            new Pose2d(0, 0, new Rotation2d(0)),
            List.of(),
            new Pose2d(driveDistance, 0, new Rotation2d(0)),
            config
        );

        // Create PID controllers for trajectory following
        var xController = new PIDController(AutoConstants.kPXController, 0, 0);
        var yController = new PIDController(AutoConstants.kPYController, 0, 0);
        var thetaController = new ProfiledPIDController(
            AutoConstants.kPThetaController, 0, 0, 
            AutoConstants.kThetaControllerConstraints
        );
        thetaController.enableContinuousInput(-Math.PI, Math.PI);

        // Create the swerve controller command
        SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(
            trajectory,
            driveSubsystem::getPose,
            DriveConstants.kDriveKinematics,
            xController,
            yController,
            thetaController,
            driveSubsystem::setModuleStates,
            driveSubsystem
        );
        
        return Commands.sequence(
            // Reset odometry to trajectory starting position
            Commands.runOnce(() -> driveSubsystem.resetOdometry(trajectory.getInitialPose()), driveSubsystem),
            // Execute the trajectory
            swerveControllerCommand,
            // Run the arm for the specified time
            Commands.run(
                () -> armSubsystem.setCoralArmPower(Constants.CORAL_ARM_FORWARD),
                armSubsystem
            ).withTimeout(armTime),
            // Run the wheel for the specified time
            Commands.run(
                () -> endEffectorSubsystem.setWheelSpeed(Constants.WHEEL_FORWARD),
                endEffectorSubsystem
            ).withTimeout(wheelTime),
            // Stop everything
            Commands.parallel(
                Commands.runOnce(() -> driveSubsystem.drive(0, 0, 0, false), driveSubsystem),
                Commands.runOnce(() -> armSubsystem.setCoralArmPower(0), armSubsystem),
                Commands.runOnce(() -> endEffectorSubsystem.setWheelSpeed(0), endEffectorSubsystem)
            )
        );
    }
}