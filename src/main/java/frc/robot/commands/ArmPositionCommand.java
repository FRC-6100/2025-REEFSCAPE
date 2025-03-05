package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ArmSubsystem;

/**
 * Command to move the arm to a preset position
 */
public class ArmPositionCommand extends Command {
  private final ArmSubsystem m_subsystem;
  private final boolean m_isPositionOne;

  /**
   * Creates a new ArmPositionCommand.
   *
   * @param subsystem The subsystem used by this command.
   * @param isPositionOne Whether to move to position one (true) or position two (false)
   */
  public ArmPositionCommand(ArmSubsystem subsystem, boolean isPositionOne) {
    m_subsystem = subsystem;
    m_isPositionOne = isPositionOne;
    
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (m_isPositionOne) {
      m_subsystem.setPositionOne();
    } else {
      m_subsystem.setPositionTwo();
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Nothing to do here, the PID controller in the subsystem handles it
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // No need to explicitly stop since we want to maintain position
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // Command completes when the arm is at the target position
    return m_subsystem.isAtTarget();
  }
}