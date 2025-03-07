package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import java.util.function.DoubleSupplier;

public class AlgaeArmSubsystem extends SubsystemBase {
  // Create the motor controller
  private final SparkMax algaeArmMotor;
  
  // Create encoder for position tracking
  private final RelativeEncoder algaeArmEncoder;

  /**
   * Creates a new AlgaeArmSubsystem.
   */
  public AlgaeArmSubsystem() {
    // Initialize the arm motor (NEO 550)
    algaeArmMotor = new SparkMax(Constants.ALGAE_ARM_MOTOR_ID, MotorType.kBrushless);
    algaeArmMotor.setInverted(Constants.INTAKE_ARM_INVERT);
    
    // Get the encoder for the arm motor for position control
    algaeArmEncoder = algaeArmMotor.getEncoder();
    
    // Reset the arm encoder position to 0
    algaeArmEncoder.setPosition(0);
  }

  /**
   * Sets the percent output to the algae arm motor.
   * Applies gravity compensation if configured.
   * 
   * @param percentOutput The percent output to set (-1.0 to 1.0)
   */
  public void setAlgaeArmSpeed(double percentOutput) {
    // Apply gravity compensation constant to help maintain position
    algaeArmMotor.set(percentOutput + Constants.GRAVITY_RESISTANCE);
  }

  /**
   * Gets the current position of the algae arm.
   * 
   * @return The current position in rotations
   */
  public double getAlgaeArmPosition() {
    return algaeArmEncoder.getPosition();
  }

  /**
   * Stops the algae arm motor.
   */
  public void stopAlgaeArm() {
    algaeArmMotor.set(0);
  }

  /**
   * Creates a command that runs the algae arm at a fixed speed.
   * 
   * @param speed The speed to run the arm at (-1.0 to 1.0)
   * @return A command that will run the algae arm
   */
  public Command runAlgaeArmCommand(double speed) {
    return this.runOnce(() -> setAlgaeArmSpeed(speed))
              .andThen(this.run(() -> setAlgaeArmSpeed(speed)))
              .finallyDo(interrupted -> stopAlgaeArm());
  }

  /**
   * Creates a command that runs the algae arm with a speed supplier.
   * 
   * @param speedSupplier A supplier for the speed (-1.0 to 1.0)
   * @return A command that will run the algae arm
   */
  public Command runAlgaeArmCommand(DoubleSupplier speedSupplier) {
    return this.run(() -> setAlgaeArmSpeed(speedSupplier.getAsDouble()))
              .finallyDo(interrupted -> stopAlgaeArm());
  }

  /**
   * Creates a command that moves the algae arm to a specific position.
   * 
   * @param targetPosition The target position in rotations
   * @return A command that will move the arm to the target position
   */
  public Command moveArmToPositionCommand(double targetPosition) {
    return this.run(() -> {
      double currentPosition = getAlgaeArmPosition();
      double error = targetPosition - currentPosition;
      double output = Math.copySign(
          Math.min(Math.abs(error * Constants.INTAKE_ARM_kP), Constants.INTAKE_ARM_MAX_POWER),
          error);
          
      // Apply minimum power if we're still moving but ensure it's in the right direction
      if (Math.abs(error) > 0.5 && Math.abs(output) < Constants.INTAKE_ARM_MIN_POWER) {
        output = Math.copySign(Constants.INTAKE_ARM_MIN_POWER, error);
      }
      
      setAlgaeArmSpeed(output);
    }).until(() -> Math.abs(targetPosition - getAlgaeArmPosition()) < 0.5)
      .finallyDo(interrupted -> stopAlgaeArm());
  }

  /**
   * Creates a command that deploys the algae intake mechanism.
   * 
   * @return A command that will deploy the algae intake
   */
  public Command deployAlgaeArmCommand() {
    return moveArmToPositionCommand(Constants.INTAKE_DEPLOY_LIMIT);
  }

  /**
   * Creates a command that stows the algae intake mechanism.
   * 
   * @return A command that will stow the algae intake
   */
  public Command stowAlgaeArmCommand() {
    return moveArmToPositionCommand(Constants.INTAKE_RETURN_LIMIT);
  }

  /**
   * Creates a command that positions the arm for algae pickup.
   * 
   * @return A command that positions the arm for algae pickup
   */
  public Command positionForAlgaePickupCommand() {
    return moveArmToPositionCommand(Constants.PICK_UP_ALGAE_POSITION);
  }

  /**
   * Creates a command that positions the arm to hold algae.
   * 
   * @return A command that positions the arm to hold algae
   */
  public Command positionToHoldAlgaeCommand() {
    return moveArmToPositionCommand(Constants.HOLD_ALGAE_POSITION);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}