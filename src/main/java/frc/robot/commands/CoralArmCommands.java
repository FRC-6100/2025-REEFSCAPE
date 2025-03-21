package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import frc.robot.subsystems.CoralArmSubsystem;
import frc.robot.subsystems.CoralArmSubsystem.ArmPosition;

/**
 * Command factory class for the Coral Arm subsystem.
 */
public class CoralArmCommands {
    private final CoralArmSubsystem arm;
    
    public CoralArmCommands(CoralArmSubsystem arm) {
        this.arm = arm;
    }

    /**
     * Creates a command to continuously increment arm up while button is held
     */
    public Command incrementUp() {
        return arm.incrementUpCommand();
    }

    /**
     * Creates a command to continuously increment arm down while button is held
     */
    public Command incrementDown() {
        return arm.incrementDownCommand();
    }

    /**
     * Creates a command that moves the arm to a target position
     */
    public Command setPosition(double targetPosition) {
        return new FunctionalCommand(
            () -> {
                // Log that we're starting the arm movement
                System.out.println("Starting arm movement to " + targetPosition);
            },
            // Move the arm to the target position
            () -> arm.setPosition(targetPosition),
            interrupted -> {
                if (interrupted) {
                    System.out.println("Arm movement interrupted");
                    arm.stopAndHold();
                } else {
                    System.out.println("Arm reached position " + targetPosition);
                }
            },
            () -> arm.isAtPosition(targetPosition),
            arm
        ).withName("MoveArmTo(" + targetPosition + ")");
    }

    // Preset position commands
    public Command setStowed() {
        return arm.setPositionCommand(ArmPosition.STOWED);
    }

    public Command setIntake() {
        return arm.setPositionCommand(ArmPosition.POSE_ONE);
    }

    public Command setLowScore() {
        return arm.setPositionCommand(ArmPosition.POSE_TWO);
    }

    public Command setHighScore() {
        return arm.setPositionCommand(ArmPosition.POSE_THREE);
    }
}