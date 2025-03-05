package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ArmSubsystem extends SubsystemBase {
  private final SparkMax m_motor;
  private final RelativeEncoder m_encoder;
  private final SparkClosedLoopController m_controller;
  
  // The preset positions (in rotations)
  /* Values derived from testing on Monday
   * May need adjustments
   * Theoritical max:
   */
  private final double POSITION_ZERO = 0.0;  // Adjust based on your desired position
  private final double POSITION_ONE = -20.0;  // TODO Adjust based on your desired position
  private final double POSITION_TWO = -26.0;  // Adjust based on your desired position
  
  // Current target position
  private double m_targetPosition = 0;
  
  // Constants for PID and feed forward
  private static final double kP = 0.1;
  private static final double kI = 0;
  private static final double kD = 0;
//   private static final double kFF = 0.05; // Feed forward to counteract gravity
  
  // Constants for position control
  private static final double MAX_OUTPUT = 0.4;  // Limits max speed of the arm
  private static final double MIN_OUTPUT = -0.4; // Limits max speed in reverse
  private static final double POSITION_TOLERANCE = 0.1; // Rotations tolerance
  
  public ArmSubsystem() {
    // Initialize the motor as a Neo550
    m_motor = new SparkMax(Constants.END_EFFECTOR_ARM_MOTOR_ID, MotorType.kBrushless);
    
    // Configure motor
    SparkMaxConfig config = new SparkMaxConfig();
    config.idleMode(IdleMode.kBrake);
    config.smartCurrentLimit(20); // Appropriate for Neo550
    
    // Configure PID 
    config.closedLoop.pid(kP, kI, kD);
    // config.closedLoop.ff(kFF);
    config.closedLoop.outputRange(MIN_OUTPUT, MAX_OUTPUT);
    
    // Apply configuration
    // m_motor.applyConfig(config); // FIXME: Not yet supported in the API?!
    
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
    SmartDashboard.putNumber("Arm Position", m_encoder.getPosition());
    SmartDashboard.putNumber("Arm Target", m_targetPosition);
    SmartDashboard.putBoolean("Arm At Target", isAtTarget());
 
    // Report motor status to SmartDashboard
    SmartDashboard.putNumber("Arm Motor Position", m_encoder.getPosition());
    SmartDashboard.putNumber("Arm Motor Velocity", m_encoder.getVelocity());
    SmartDashboard.putNumber("Arm Motor Applied Output", m_motor.getAppliedOutput());
    SmartDashboard.putNumber("Arm Motor Current", m_motor.getOutputCurrent());
    // SmartDashboard.putString("Arm Motor Idle Mode", m_motor.getIdleMode().toString());
    
  }

  /**
   * Set the arm to position one
   */
  public void setPositionZero() {
    setTargetPosition(POSITION_ZERO);
  }
  
  /**
   * Set the arm to position one
   */
  public void setPositionOne() {
    setTargetPosition(POSITION_ONE);
  }
  
  /**
   * Set the arm to position two
   */
  public void setPositionTwo() {
    setTargetPosition(POSITION_TWO);
  }
  
  /**
   * Increment the target position
   * @param increment Amount to increment (positive or negative)
   */
  public void incrementPosition(double increment) {
    setTargetPosition(m_targetPosition + increment);
  }
  
  /**
   * Set a specific target position
   * @param position Target position in rotations
   */
  public void setTargetPosition(double position) {
    // Set the target position
    m_targetPosition = position;
    
    // Apply the PID controller to reach the target
    m_controller.setReference(m_targetPosition, ControlType.kPosition);
  }
  
  /**
   * @return The current position of the arm in rotations
   */
  public double getCurrentPosition() {
    return m_encoder.getPosition();
  }
  
  /**
   * Check if the arm is at the target position
   * @return true if the arm is within tolerance of the target
   */
  public boolean isAtTarget() {
    return Math.abs(m_encoder.getPosition() - m_targetPosition) < POSITION_TOLERANCE;
  }
  
  /**
   * Stop the arm motor
   */
  public void stop() {
    // When stopped, maintain the current position
    m_targetPosition = m_encoder.getPosition();
    m_controller.setReference(m_targetPosition, ControlType.kPosition);
  }
  
  // ==== COMMAND FACTORIES ====
  
  /**
   * Creates a command that moves the arm to position one
   * @return A command that moves the arm to position one
   */
  public Command positionZeroCommand() {
    return Commands.runOnce(() -> setPositionZero(), this)
        .andThen(Commands.waitUntil(this::isAtTarget));
  }
  
  /**
   * Creates a command that moves the arm to position one
   * @return A command that moves the arm to position one
   */
  public Command positionOneCommand() {
    return Commands.runOnce(() -> setPositionOne(), this)
        .andThen(Commands.waitUntil(this::isAtTarget));
  }
  
    /**
   * Creates a command that moves the arm to position two
   * @return A command that moves the arm to position two
   */
  public Command positionTwoCommand() {
    return Commands.runOnce(() -> setPositionTwo(), this)
        .andThen(Commands.waitUntil(this::isAtTarget));
  }
  /**
   * Creates a command that moves the arm to a specific position
   * @param position The target position in rotations
   * @return A command that moves the arm to the specified position
   */
  public Command positionCommand(double position) {
    return Commands.runOnce(() -> setTargetPosition(position), this)
        .andThen(Commands.waitUntil(this::isAtTarget));
  }
  
  /**
   * Creates a command that handles incremental arm movement based on button inputs
   * @param incrementUp Supplier that returns true when the arm should be moved up
   * @param incrementDown Supplier that returns true when the arm should be moved down
   * @param incrementAmount The amount to increment by each time (rotations)
   * @return A command that handles incremental arm movement
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

    public void setCoralArmPower(double percentOutput) {
      m_motor.set(percentOutput);
      SmartDashboard.putString("Motor Control Mode", "Direct");
  }

  // Command factory for testing
  public Command setCoralArmPowerCommand(double percentOutput) {
      return Commands.runOnce(() -> setCoralArmPower(percentOutput), this);
  }  

}