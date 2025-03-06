// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;
// Open Source Software; you can modify and/or share it under the terms of
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
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.DriveSubsystem;

import java.util.List;

/**
 * A command that drives the robot forward a specified distance using a trajectory.
 */
public class AutonDriveForwardCommand extends Command {
    private final DriveSubsystem m_drive;
    private final Command m_command;
    
    /**
     * Creates a new AutonForwardCommand.
     *
     * @param driveSubsystem The drive subsystem to use.
     */
    public AutonDriveForwardCommand(DriveSubsystem driveSubsystem) {
        m_drive = driveSubsystem;
        
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
            m_drive::getPose,
            DriveConstants.kDriveKinematics,
            xController,
            yController,
            thetaController,
            m_drive::setModuleStates,
            m_drive
        );

        // Build the command
        m_command = Commands.sequence(
            // Reset odometry to trajectory starting position
            Commands.runOnce(() -> m_drive.resetOdometry(trajectory.getInitialPose())),
            // Execute the trajectory
            swerveControllerCommand,
            // Stop the robot
            Commands.runOnce(() -> m_drive.drive(0, 0, 0, false))
        );
    }

    @Override
    public void initialize() {
        m_command.initialize();
    }

    @Override
    public void execute() {
        m_command.execute();
    }

    @Override
    public void end(boolean interrupted) {
        m_command.end(interrupted);
    }

    @Override
    public boolean isFinished() {
        return m_command.isFinished();
    }
}