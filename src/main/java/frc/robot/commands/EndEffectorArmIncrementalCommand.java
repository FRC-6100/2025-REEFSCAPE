package frc.robot.commands;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.EndEffectorSubsystem;

/**
 * Command to incrementally move the end effector arm and hold position
 */
public class EndEffectorArmIncrementalCommand extends Command {
    private final EndEffectorSubsystem m_endEffector;
    private final BooleanSupplier m_incrementPositive; // Button for clockwise/positive movement
    private final BooleanSupplier m_incrementNegative; // Button for counter-clockwise/negative movement
    private final double m_incrementAmount; // How much to move per button press
    
    // Variables to track button state for single press detection
    private boolean m_wasPositivePressed = false;
    private boolean m_wasNegativePressed = false;
    
    /**
     * Creates a new EndEffectorArmIncrementalCommand for small, precise arm movements
     * 
     * @param endEffector The end effector subsystem
     * @param incrementPositive Button supplier for positive/clockwise increments
     * @param incrementNegative Button supplier for negative/counter-clockwise increments
     * @param incrementAmount Amount to move per button press (in encoder units)
     */
    public EndEffectorArmIncrementalCommand(
            EndEffectorSubsystem endEffector,
            BooleanSupplier incrementPositive,
            BooleanSupplier incrementNegative,
            double incrementAmount) {
        
        m_endEffector = endEffector;
        m_incrementPositive = incrementPositive;
        m_incrementNegative = incrementNegative;
        m_incrementAmount = incrementAmount;
        
        addRequirements(endEffector);
    }

    @Override
    public void initialize() {
        // Initialize button state trackers
        m_wasPositivePressed = false;
        m_wasNegativePressed = false;
    }

    @Override
    public void execute() {
        boolean isPositivePressed = m_incrementPositive.getAsBoolean();
        boolean isNegativePressed = m_incrementNegative.getAsBoolean();
        
        // Detect rising edge of positive button (just pressed)
        if (isPositivePressed && !m_wasPositivePressed) {
            // Increment position in positive direction
            m_endEffector.incrementArmPosition(m_incrementAmount);
        }
        
        // Detect rising edge of negative button (just pressed)
        if (isNegativePressed && !m_wasNegativePressed) {
            // Increment position in negative direction
            m_endEffector.incrementArmPosition(-m_incrementAmount);
        }
        
        // Update button state trackers
        m_wasPositivePressed = isPositivePressed;
        m_wasNegativePressed = isNegativePressed;
        
        // Always try to hold position (PID control)
        m_endEffector.holdArmPosition();
    }

    @Override
    public void end(boolean interrupted) {
        // Stop arm motor when command ends
        m_endEffector.stopArm();
    }

    @Override
    public boolean isFinished() {
        // This command should run until interrupted
        return false;
    }
}