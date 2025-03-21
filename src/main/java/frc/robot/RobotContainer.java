// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.commands.AutonCommands;
import frc.robot.commands.SetWheelSpeedCommand;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.AlgaeArmSubsystem;
import frc.robot.subsystems.AlgaeEffectorSubsystem;
import frc.robot.subsystems.CoralArmSubsystem;

import frc.robot.subsystems.ElevatorConstants;
import frc.robot.subsystems.ElevatorSubsystem;

import frc.robot.subsystems.CoralEffectorSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

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
    private final CoralArmSubsystem m_coralArm = new CoralArmSubsystem();
    private final CoralEffectorSubsystem m_coralEffector = new CoralEffectorSubsystem();
    private final AlgaeArmSubsystem m_algaeArm = new AlgaeArmSubsystem();
    private final AlgaeEffectorSubsystem m_algaeEffector = new AlgaeEffectorSubsystem();
    private final ElevatorSubsystem m_elevator = new ElevatorSubsystem();
    
    // -------------------- Controllers --------------------
    private final CommandXboxController driverController = new CommandXboxController(0);
    private final CommandXboxController operatorController = new CommandXboxController(1);

    // Needed for an auton chooser
    private final SendableChooser<Command> m_autoChooser = new SendableChooser<>();

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */    
    public RobotContainer() {
        configureDefaultCommands();
        configureButtonBindings();
        configureAutonOptions();
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
        m_coralArm.setDefaultCommand(
            m_coralArm.incrementalCommand(
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
        configureOperatorButtons();
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
    private void configureOperatorButtons() {
        // Coral arm manual control
        operatorController.a().whileTrue(
            m_coralArm.setCoralArmPowerCommand(Constants.CORAL_ARM_FORWARD)
        );
        
        operatorController.b().whileTrue(
            m_coralArm.setCoralArmPowerCommand(Constants.CORAL_ARM_REVERSE)
        );

        // Algae arm position presets
        operatorController.x().whileTrue(
            m_elevator.setElevatorPowerCommand(ElevatorConstants.SPEED_UP)
        );
        
        operatorController.y().whileTrue(
            m_elevator.setElevatorPowerCommand(ElevatorConstants.SPEED_DOWN)
        );
    
        // End effector wheel control
        operatorController.rightBumper().whileTrue(
            new SetWheelSpeedCommand(
                m_coralEffector, () -> Constants.WHEEL_REVERSE)
        );

        operatorController.leftBumper().whileTrue(
            new SetWheelSpeedCommand(
                m_coralEffector, () -> Constants.WHEEL_FORWARD)
        );

        // Algae intake control - proportional to trigger pressure
        operatorController.leftTrigger(0.1).whileTrue(
            m_algaeEffector.runAlgaeIntakeCommand(
                () -> operatorController.getLeftTriggerAxis() * Constants.INTAKE_BAR_SPEED)
        );

        operatorController.rightTrigger(0.1).whileTrue(
            m_algaeEffector.runAlgaeIntakeCommand(
                () -> -operatorController.getRightTriggerAxis() * Constants.INTAKE_BAR_SPEED)
        );


        
        // Manual algae arm adjustment
        operatorController.povUp().whileTrue(
            m_algaeArm.runAlgaeArmCommand(Constants.ALGAE_ARM_UP)
        );
        
        operatorController.povDown().whileTrue(
            m_algaeArm.runAlgaeArmCommand(-Constants.ALGAE_ARM_DOWN)
        );
    }

private void configureAutonOptions() {
        // Add options to the chooser
        m_autoChooser.setDefaultOption("Drive Forward",
             AutonCommands.driveForward(m_robotDrive));
        m_autoChooser.addOption("Score and Drive", 
            AutonCommands.scoreAndDrive(m_robotDrive, m_coralArm, m_coralEffector));
        m_autoChooser.addOption("Do Nothing", 
            Commands.none());
        
        // Put the chooser on the dashboard
        SmartDashboard.putData("Auto Selector", m_autoChooser);
    }

    /**
     * Creates and returns the autonomous command.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        // Return the selected autonomous command from the chooser
        return m_autoChooser.getSelected();
    }

}