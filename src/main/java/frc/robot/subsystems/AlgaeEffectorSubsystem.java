package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import java.util.function.DoubleSupplier;

public class AlgaeEffectorSubsystem extends SubsystemBase {
  // Create the motor controller
  private final SparkMax algaeIntakeMotor;

  /**
   * Creates a new AlgaeIntakeSubsystem.
   */
  public AlgaeEffectorSubsystem() {
    // Initialize the intake bar motor (NEO 550)
    algaeIntakeMotor = new SparkMax(Constants.ALGAE_BAR_MOTOR_ID, MotorType.kBrushless);
    algaeIntakeMotor.setInverted(Constants.INTAKE_BAR_INVERT);
  }

  /**
   * Sets the percent output to the algae intake motor.
   * 
   * @param percentOutput The percent output to set (-1.0 to 1.0)
   */
  public void setAlgaeIntakeSpeed(double percentOutput) {
    algaeIntakeMotor.set(percentOutput);
  }

  /**
   * Stops the algae intake motor.
   */
  public void stopAlgaeIntake() {
    algaeIntakeMotor.set(0);
  }

  /**
   * Creates a command that runs the algae intake at a fixed speed.
   * 
   * @param speed The speed to run the intake at (-1.0 to 1.0)
   * @return A command that will run the algae intake
   */
  public Command runAlgaeIntakeCommand(double speed) {
    return this.runOnce(() -> setAlgaeIntakeSpeed(speed))
              .andThen(this.run(() -> setAlgaeIntakeSpeed(speed)))
              .finallyDo(interrupted -> stopAlgaeIntake());
  }

  /**
   * Creates a command that runs the algae intake with a speed supplier.
   * 
   * @param speedSupplier A supplier for the speed (-1.0 to 1.0)
   * @return A command that will run the algae intake
   */
  public Command runAlgaeIntakeCommand(DoubleSupplier speedSupplier) {
    return this.run(() -> setAlgaeIntakeSpeed(speedSupplier.getAsDouble()))
              .finallyDo(interrupted -> stopAlgaeIntake());
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}