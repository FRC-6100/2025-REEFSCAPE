package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CoralEffectorSubsystem;

/**
 * Command to control the end effector wheel with variable power
 */
public class SetWheelSpeedCommand extends Command {
    private final CoralEffectorSubsystem m_endEffectorWheel;
    private final DoubleSupplier m_powerSupplier;
    
    /**
     * Creates a new EndEffectorWheelCommand for controlling the wheel
     * 
     * @param endEffectorWheel The end effector wheel subsystem
     * @param powerSupplier Supplier that provides the power level (-1.0 to 1.0)
     */
    public SetWheelSpeedCommand(
        CoralEffectorSubsystem endEffectorWheel,
            DoubleSupplier powerSupplier) {
        
        m_endEffectorWheel = endEffectorWheel;
        m_powerSupplier = powerSupplier;
        
        addRequirements(endEffectorWheel);
    }

    @Override
    public void initialize() {
        // Nothing to initialize
    }

    @Override
    public void execute() {
        // Get the current power setting from the supplier
        double speed = m_powerSupplier.getAsDouble();
        
        // Apply the power to the wheel motor
        m_endEffectorWheel.setWheelSpeed(speed);
    }

    @Override
    public void end(boolean interrupted) {
        // Stop wheel motor when command ends
        m_endEffectorWheel.stop();
    }

    @Override
    public boolean isFinished() {
        // This command should run until interrupted
        return false;
    }
}