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
import frc.robot.commands.EndEffectorWheelCommand;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.EndEffectorSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import java.util.List;

/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems
  private final DriveSubsystem m_robotDrive = new DriveSubsystem();

  // The driver's controller

  private final CommandXboxController driverController = new CommandXboxController(0);
  private final CommandXboxController operatorController = new CommandXboxController(1);

  // declaration to the RobotContainer class
  private final ArmSubsystem m_arm = new ArmSubsystem();

  private final EndEffectorSubsystem m_endEffector = new EndEffectorSubsystem();

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();

    // Configure default commands
    m_robotDrive.setDefaultCommand(
        // The left stick controls translation of the robot.
        // Turning is controlled by the X axis of the right stick.
        new RunCommand(
            () -> m_robotDrive.drive(
                -MathUtil.applyDeadband(driverController.getLeftY(), Constants.kDriveDeadband),
                -MathUtil.applyDeadband(driverController.getLeftX(), Constants.kDriveDeadband),
                -MathUtil.applyDeadband(driverController.getRightX(), Constants.kDriveDeadband),
                true),
            m_robotDrive));
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by
   * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its
   * subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then calling
   * passing it to a
   * {@link JoystickButton}.
   */
  private void configureButtonBindings() {

    driverController.a().whileTrue(new RunCommand(() -> m_robotDrive.setX(), m_robotDrive)); // Puts drive into brake
                                                                                             // mode
    // driverController.b().whileTrue()); // Puts drive into field oriented mode
    // driverController.x().whileTrue()); // Puts drive into robot oriented mode
    // driverController.y().whileTrue(); // This move to pose 1

    // driverController.start().whileTrue(); // Resets the
    // driverController.back().whileTrue(); // Resets the gyro

    // driverController.povUp().whileTrue(); // Sets drive speed to 0.5
    // driverController.povDown().whileTrue()); // Sets drive speed to 1
    // driverController.povRight().whileTrue()); // Sets drive speed to 0.75
    // driverController.povLeft().whileTrue(); // Sets drive speed to 0.25

    operatorController.rightBumper().whileTrue(
        new EndEffectorWheelCommand(
            // Change the sign if you want to change the direction of the wheel
            m_endEffector, () -> -0.6));

    operatorController.leftBumper().whileTrue(
        new EndEffectorWheelCommand(
            // Change the sign if you want to change the direction of the wheel
            m_endEffector, () -> 0.6));

    // Arm position preset buttons using command factories
    /*
    TODO Test and adjust the position variables in the ArmSubsystem (lines 31-33 ish)
    Current values just for reference here.
    POSITION_ZERO = 0.0;
    POSITION_ONE = -20.0;
    POSITION_TWO = -26.0;
    */
    
    // Go to preset POSITION_ZERO variable in ArmSubsystem
    operatorController.x().onTrue(m_arm.positionZeroCommand());
    // Go to preset POSITION_ONE variable in ArmSubsystem 
    operatorController.b().onTrue(m_arm.positionOneCommand());
    // Go to preset POSITION_TWO variable in ArmSubsystem
    operatorController.y().onTrue(m_arm.positionTwoCommand());

    // Option 1: Using the command factory
    // This approach doesn't track button state transitions, it applies increments
    // continuously while the button is held, so be careful with the increment
    // amount
    m_arm.setDefaultCommand(
        m_arm.incrementalCommand(
            // Runs an increment command when the up button is held
            () -> operatorController.povLeft().getAsBoolean(),
            // Runs a increment command when the down button is held
            () -> operatorController.povRight().getAsBoolean(),
            // TODO Small increment since this runs continuously
            0.01 
        ));

    // Option 2 Incremental: Create a custom command that handles button transitions
    // m_arm.setDefaultCommand(
    // new IncrementalArmControlCommand(
    // m_arm,
    // () -> operatorController.povUp().getAsBoolean(),
    // () -> operatorController.povDown().getAsBoolean(),
    // 0.1
    // )
    // );
    // }

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // Create config for trajectory
    TrajectoryConfig config = new TrajectoryConfig(
        AutoConstants.kMaxSpeedMetersPerSecond,
        AutoConstants.kMaxAccelerationMetersPerSecondSquared)
        // Add kinematics to ensure max speed is actually obeyed
        .setKinematics(DriveConstants.kDriveKinematics);

    // An example trajectory to follow. All units in meters.
    Trajectory driveForwardTrajectory = TrajectoryGenerator.generateTrajectory(
        // Start at the origin facing the +X direction
        new Pose2d(0, 0, new Rotation2d(0)),
        // Pass through these two interior waypoints, making an 's' curve path
        List.of(),
        // End 3 meters straight ahead of where we started, facing forward
        new Pose2d(3, 0, new Rotation2d(0)),
        config);

    var thetaController = new ProfiledPIDController(
        AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(
        driveForwardTrajectory,
        m_robotDrive::getPose, // Functional interface to feed supplier
        DriveConstants.kDriveKinematics,

        // Position controllers
        new PIDController(AutoConstants.kPXController, 0, 0),
        new PIDController(AutoConstants.kPYController, 0, 0),
        thetaController,
        m_robotDrive::setModuleStates,
        m_robotDrive);

    // Reset odometry to the starting pose of the trajectory.
    m_robotDrive.resetOdometry(driveForwardTrajectory.getInitialPose());

    // Run path following command, then stop at the end.
    return swerveControllerCommand.andThen(() -> m_robotDrive.drive(0, 0, 0, false));
  }
}
