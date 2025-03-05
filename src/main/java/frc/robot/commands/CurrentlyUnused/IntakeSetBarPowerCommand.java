package frc.robot.commands.CurrentlyUnused;

import edu.wpi.first.wpilibj2.command.Command;

public class IntakeSetBarPowerCommand extends Command {
    private final double power;

    IntakeSubsystem m_intakeSubsystem = new IntakeSubsystem();

    public IntakeSetBarPowerCommand(double power) {
        this.power = power;
        addRequirements(m_intakeSubsystem);
    }

    @Override
    public void execute() {
        m_intakeSubsystem.setIntakeBarPower(power);
    }

    @Override
    public void end(boolean interrupted) {
        m_intakeSubsystem.stopIntakeBar();
    }
}
