// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.commands.SetWheelPowerCommand;
// import frc.robot.subsystems.AlgaeSubsystem;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.EndEffectorSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import java.util.List;

/**
 * The RobotContainer class is the centralized location for robot configuration.
 * It contains all subsystems, controllers, and button mappings.
 * 
 * The structure follows a command-based paradigm where robot functionality is
 * divided into subsystems with corresponding commands.
 */
public class RobotContainer {
    // -------------------- Subsystems --------------------
    private final DriveSubsystem m_robotDrive = new DriveSubsystem();
    private final ArmSubsystem m_arm = new ArmSubsystem();
    private final EndEffectorSubsystem m_endEffector = new EndEffectorSubsystem();
    // private final AlgaeSubsystem m_algaeSubsystem = new AlgaeSubsystem();

    // -------------------- Controllers --------------------
    private final CommandXboxController driverController = new CommandXboxController(0);
    private final CommandXboxController operatorController = new CommandXboxController(1);

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        configureDefaultCommands();
        configureButtonBindings();
    }

    /**
     * Configure default commands for subsystems.
     * These commands run when no other command is using the subsystem.
     */
    private void configureDefaultCommands() {
        // Default drive command - Controlled by driver's left and right sticks
        m_robotDrive.setDefaultCommand(
            new RunCommand(
                () -> m_robotDrive.drive(
                    -MathUtil.applyDeadband(driverController.getLeftY(), Constants.kDriveDeadband),
                    -MathUtil.applyDeadband(driverController.getLeftX(), Constants.kDriveDeadband),
                    -MathUtil.applyDeadband(driverController.getRightX(), Constants.kDriveDeadband),
                    true),
                m_robotDrive)
        );

        // Default arm command - Incremental control using POV buttons
        m_arm.setDefaultCommand(
            m_arm.incrementalCommand(
                () -> operatorController.povLeft().getAsBoolean(),
                () -> operatorController.povRight().getAsBoolean(),
                0.01) // Small increment since this runs continuously
        );
    }

    /**
     * Configure all button bindings for controllers.
     * Grouped by subsystem for better organization.
     */
    private void configureButtonBindings() {
        configureDriveButtons();
        configureArmButtons();
        configureEndEffectorButtons();
        // configureAlgaeSubsystemButtons();
    }

    /**
     * Configure drive subsystem button bindings.
     */
    private void configureDriveButtons() {
        // Lock wheels in X pattern for stability
        driverController.a().whileTrue(
            new RunCommand(() -> m_robotDrive.setX(), m_robotDrive)
        );

        // Field-oriented mode
        // driverController.b().whileTrue(...);

        // Robot-oriented mode
        // driverController.x().whileTrue(...);

        // Move to specific pose
        // driverController.y().whileTrue(...);

        // Reset functions
        // driverController.start().whileTrue(...);
        // driverController.back().whileTrue(...);

        // Speed control presets
        // driverController.povUp().whileTrue(...);
        // driverController.povDown().whileTrue(...);
        // driverController.povRight().whileTrue(...);
        // driverController.povLeft().whileTrue(...);
    }

    /**
     * Configure arm subsystem button bindings.
     */
    private void configureArmButtons() {
        // Coral arm manual control
        operatorController.a().whileTrue(
            m_arm.setCoralArmPowerCommand(Constants.CORAL_ARM_FORWARD)
        );
        
        operatorController.b().whileTrue(
            m_arm.setCoralArmPowerCommand(Constants.CORAL_ARM_REVERSE)
        );
    }

    /**
     * Configure end effector subsystem button bindings.
     */
    private void configureEndEffectorButtons() {
        // End effector wheel control
        operatorController.rightBumper().whileTrue(
            new SetWheelPowerCommand(
                m_endEffector, () -> Constants.WHEEL_REVERSE)
        );

        operatorController.leftBumper().whileTrue(
            new SetWheelPowerCommand(
                m_endEffector, () -> Constants.WHEEL_FORWARD)
        );
    }

    /**
     * Configure algae subsystem button bindings.
     */
    /*
    private void configureAlgaeSubsystemButtons() {
        // Algae intake control - proportional to trigger pressure
        operatorController.leftTrigger(0.1).whileTrue(
            m_algaeSubsystem.runAlgaeIntakeCommand(
                () -> operatorController.getLeftTriggerAxis() * Constants.INTAKE_BAR_SPEED)
        );

        operatorController.rightTrigger(0.1).whileTrue(
            m_algaeSubsystem.runAlgaeIntakeCommand(
                () -> -operatorController.getRightTriggerAxis() * Constants.INTAKE_BAR_SPEED)
        );

        // Algae arm position presets
        operatorController.x().onTrue(
            m_algaeSubsystem.positionForAlgaePickupCommand()
        );
        
        operatorController.y().onTrue(
            m_algaeSubsystem.positionToHoldAlgaeCommand()
        );

        // Manual algae arm adjustment
        operatorController.povUp().whileTrue(
            m_algaeSubsystem.runAlgaeArmCommand(Constants.DEPLOY_SPEED)
        );
        
        operatorController.povDown().whileTrue(
            m_algaeSubsystem.runAlgaeArmCommand(-Constants.DEPLOY_SPEED)
        );
    }
 */

    /**
     * Creates and returns the autonomous command.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        // Create trajectory configuration
        TrajectoryConfig config = new TrajectoryConfig(
            AutoConstants.kMaxSpeedMetersPerSecond,
            AutoConstants.kMaxAccelerationMetersPerSecondSquared
        ).setKinematics(DriveConstants.kDriveKinematics);

        // Create a simple forward trajectory
        Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
            new Pose2d(0, 0, new Rotation2d(0)),
            List.of(),
            new Pose2d(3, 0, new Rotation2d(0)),
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
            m_robotDrive::getPose,
            DriveConstants.kDriveKinematics,
            xController,
            yController,
            thetaController,
            m_robotDrive::setModuleStates,
            m_robotDrive
        );

        // Reset odometry to trajectory starting position
        m_robotDrive.resetOdometry(trajectory.getInitialPose());

        // Return the complete autonomous command
        return swerveControllerCommand.andThen(() -> m_robotDrive.drive(0, 0, 0, false));
    }
}