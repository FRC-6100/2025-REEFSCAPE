package frc.robot.commands;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CoralArmSubsystem;

/**
 * Command to incrementally adjust the arm position
 */
public class ArmIncrementalCommand extends Command {
  private final CoralArmSubsystem m_subsystem;
  private final BooleanSupplier m_incrementUp;
  private final BooleanSupplier m_incrementDown;
  private final double m_incrementAmount;
  
  private boolean m_lastIncrementUp = false;
  private boolean m_lastIncrementDown = false;

  /**
   * Creates a new ArmIncrementalCommand.
   *
   * @param subsystem The subsystem used by this command.
   * @param incrementUp Supplier that returns true when the arm should be moved up
   * @param incrementDown Supplier that returns true when the arm should be moved down
   * @param incrementAmount The amount to increment by each time (rotations)
   */
  public ArmIncrementalCommand(
      CoralArmSubsystem subsystem, 
      BooleanSupplier incrementUp,
      BooleanSupplier incrementDown,
      double incrementAmount) {
    m_subsystem = subsystem;
    m_incrementUp = incrementUp;
    m_incrementDown = incrementDown;
    m_incrementAmount = incrementAmount;
    
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // Initialize tracking variables
    m_lastIncrementUp = false;
    m_lastIncrementDown = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Check for button presses (on rising edge only to avoid continuous movement)
    boolean currentIncrementUp = m_incrementUp.getAsBoolean();
    boolean currentIncrementDown = m_incrementDown.getAsBoolean();
    
    // Detect rising edge of increment up button
    if (currentIncrementUp && !m_lastIncrementUp) {
      m_subsystem.incrementPosition(m_incrementAmount);
    }
    
    // Detect rising edge of increment down button
    if (currentIncrementDown && !m_lastIncrementDown) {
      m_subsystem.incrementPosition(-m_incrementAmount);
    }
    
    // Update last button states
    m_lastIncrementUp = currentIncrementUp;
    m_lastIncrementDown = currentIncrementDown;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // No need to explicitly stop since we want to maintain position
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // This is a default command, so it should never finish on its own
    return false;
  }
}