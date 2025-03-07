package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

/*
 * This was copy and pasted from the Coral ArmSubsystem because some of the principles should be the same
 */

public class ElevatorSubsystem extends SubsystemBase {
  private final SparkMax m_motor;
  private final RelativeEncoder m_encoder;
  private final SparkClosedLoopController m_controller;

  // The preset positions (in rotations)
  /*
   * Values derived from testing on Monday
   * May need adjustments
   * Theoritical max:
   */

  // TODO Adjust these pose values based on your robot & testing
  private final double POSITION_ZERO = 0.0;
  private final double POSITION_ONE = -20.0;
  private final double POSITION_TWO = -26.0;

  // Current target position
  private double m_targetPosition = 0;

  // Constants for PID and feed forward
  private static final double kP = 0.1;
  private static final double kI = 0;
  private static final double kD = 0;
  // private static final double kFF = 0.05; // Feed forward to counteract gravity

  // Constants for position control
  private static final double OUTPUT_UP = 0.2; // Limits max speed of the Elevator
  private static final double OUTPUT_DOWN = -0.2; // Limits max speed in reverse
  private static final double POSITION_TOLERANCE = 0.1; // Rotations tolerance

  public ElevatorSubsystem() {
    // Initialize the motor
    m_motor = new SparkMax(Constants.ELEVATOR_MOTOR_ID, MotorType.kBrushless);

    // Configure motor
    SparkMaxConfig config = new SparkMaxConfig();
    config.idleMode(IdleMode.kBrake);
    config.smartCurrentLimit(20); // Appropriate for Neo550

    // Configure PID
    config.closedLoop.pid(kP, kI, kD);
    // config.closedLoop.ff(kFF);
    config.closedLoop.outputRange(OUTPUT_DOWN, OUTPUT_UP);

    // Apply configuration
    // m_motor.applyConfig(config); // FIXME applyConfig is not quite working

    // Get the encoder
    m_encoder = m_motor.getEncoder();

    // Reset encoder position to zero
    m_encoder.setPosition(0);

    // Get the closed loop controller
    m_controller = m_motor.getClosedLoopController();

    // Set initial position target
    m_targetPosition = m_encoder.getPosition();
  }

  @Override
  public void periodic() {
    // Publish current position and target to SmartDashboard
    SmartDashboard.putNumber("Elevator Position", m_encoder.getPosition());
    SmartDashboard.putNumber("Elevator Target", m_targetPosition);
    SmartDashboard.putBoolean("Elevator At Target", isAtTarget());

    // Report motor status to SmartDashboard
    SmartDashboard.putNumber("Elevator Motor Position", m_encoder.getPosition());
    SmartDashboard.putNumber("Elevator Motor Velocity", m_encoder.getVelocity());
    SmartDashboard.putNumber("Elevator Motor Applied Output", m_motor.getAppliedOutput());
    SmartDashboard.putNumber("Elevator Motor Current", m_motor.getOutputCurrent());
    // SmartDashboard.putString("Elevator Motor Idle Mode",
    // m_motor.getIdleMode().toString());

  }

  /**
   * Set the Elevator to position one
   */
  public void setPositionZero() {
    setTargetPosition(POSITION_ZERO);
  }

  /**
   * Set the Elevator to position one
   */
  public void setPositionOne() {
    setTargetPosition(POSITION_ONE);
  }

  /**
   * Set the Elevator to position two
   */
  public void setPositionTwo() {
    setTargetPosition(POSITION_TWO);
  }

  /**
   * Increment the target position
   * 
   * @param increment Amount to increment (positive or negative)
   */
  public void incrementPosition(double increment) {
    setTargetPosition(m_targetPosition + increment);
  }

  /**
   * Set a specific target position
   * 
   * @param position Target position in rotations
   */
  public void setTargetPosition(double position) {
    // Set the target position
    m_targetPosition = position;

    // Apply the PID controller to reach the target
    m_controller.setReference(m_targetPosition, ControlType.kPosition);
  }

  /**
   * @return The current position of the Elevator in rotations
   */
  public double getCurrentPosition() {
    return m_encoder.getPosition();
  }

  /**
   * Check if the Elevator is at the target position
   * 
   * @return true if the Elevator is within tolerance of the target
   */
  public boolean isAtTarget() {
    return Math.abs(m_encoder.getPosition() - m_targetPosition) < POSITION_TOLERANCE;
  }
  
  public void setElevatorPower(double percentOutput) {
    m_motor.set(percentOutput);
  }

  /**
   * Stop the Elevator motor
   */
  public void stop() {
    // When stopped, maintain the current position
    m_targetPosition = m_encoder.getPosition();
    m_controller.setReference(m_targetPosition, ControlType.kPosition);
  }

  // ==== COMMAND FACTORIES ====

  /**
   * Creates a command that moves the Elevator to position one
   * 
   * @return A command that moves the Elevator to position one
   */
  public Command positionZeroCommand() {
    return Commands.runOnce(() -> setPositionZero(), this)
        .andThen(Commands.waitUntil(this::isAtTarget));
  }

  /**
   * Creates a command that moves the Elevator to position one
   * 
   * @return A command that moves the Elevator to position one
   */
  public Command positionOneCommand() {
    return Commands.runOnce(() -> setPositionOne(), this)
        .andThen(Commands.waitUntil(this::isAtTarget));
  }

  /**
   * Creates a command that moves the Elevator to position two
   * 
   * @return A command that moves the Elevator to position two
   */
  public Command positionTwoCommand() {
    return Commands.runOnce(() -> setPositionTwo(), this)
        .andThen(Commands.waitUntil(this::isAtTarget));
  }

  /**
   * Creates a command that moves the Elevator to a specific position
   * 
   * @param position The target position in rotations
   * @return A command that moves the Elevator to the specified position
   */
  public Command positionCommand(double position) {
    return Commands.runOnce(() -> setTargetPosition(position), this)
        .andThen(Commands.waitUntil(this::isAtTarget));
  }

  /**
   * Creates a command that handles incremental Elevator movement based on button
   * inputs
   * 
   * @param incrementUp     Supplier that returns true when the Elevator should be
   *                        moved up
   * @param incrementDown   Supplier that returns true when the Elevator should be
   *                        moved down
   * @param incrementAmount The amount to increment by each time (rotations)
   * @return A command that handles incremental Elevator movement
   */
  public Command incrementalCommand(
      BooleanSupplier incrementUp,
      BooleanSupplier incrementDown,
      double incrementAmount) {

    return Commands.run(() -> {
      // Each execution, check if buttons are pressed and apply increments
      if (incrementUp.getAsBoolean()) {
        incrementPosition(incrementAmount);
      }
      if (incrementDown.getAsBoolean()) {
        incrementPosition(-incrementAmount);
      }
    }, this);
  }
 

  /**
   * Creates a command for direct motor control that properly stops when button is
   * released
   */
  public Command setElevatorPowerCommand(double percentOutput) {
    // Use startEnd to explicitly define what happens when command starts and ends
    return Commands.startEnd(
        // Start action (when button is pressed)
        () -> setElevatorPower(percentOutput),
        // End action (when button is released)
        () -> setElevatorPower(0),
        this);
  }
}