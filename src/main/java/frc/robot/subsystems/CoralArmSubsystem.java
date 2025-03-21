package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class CoralArmSubsystem extends SubsystemBase {
    private final SparkMax m_motor;
    private final RelativeEncoder m_encoder;
    private final SparkClosedLoopController m_controller;

    // Define preset positions (in rotations)
    public enum ArmPosition {
        STOWED(0.0),
        POSE_ONE(-5.0),
        POSE_TWO(-15.0),
        POSE_THREE(-25.0);
        
        private final double position;
        
        ArmPosition(double position) {
            this.position = position;
        }
        
        public double getPosition() {
            return position;
        }
    }

    // Current target position
    private double m_targetPosition = 0;

    // Increment value for small adjustments
    private static final double INCREMENT_VALUE = 0.5; // Adjust based on testing
    
    // Soft limits to prevent mechanism damage
    private static final double FORWARD_LIMIT = 2.0;   // Maximum forward position
    private static final double REVERSE_LIMIT = -28.0; // Maximum reverse position

    // PID Constants
    private static final double kP = 0.10;
    private static final double kI = 0.00;
    private static final double kD = 0.00; // 1.0?
    private static final double kFF = 0.05; // Feed forward component

    // Position control constants
    private static final double MAX_OUTPUT = 0.4; // Limits max speed
    private static final double MIN_OUTPUT = -0.4;
    private static final double POSITION_TOLERANCE = 0.5; // Acceptable error

    public CoralArmSubsystem() {
      // Initialize the motor
      m_motor = new SparkMax(Constants.EFFECTOR_ARM_MOTOR_ID, MotorType.kBrushless);

      // Configure motor
      SparkMaxConfig config = new SparkMaxConfig();
      config.idleMode(IdleMode.kBrake);
      config.smartCurrentLimit(20); // Appropriate for Neo550
      config.inverted(false); // Set appropriate direction

      // Configure PID and feed forward
      config.closedLoop.pidf(kP, kI, kD, kFF);
      config.closedLoop.outputRange(MIN_OUTPUT, MAX_OUTPUT);

      // Apply configuration
      m_motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

      // Get the encoder and controller
      m_encoder = m_motor.getEncoder();
      m_controller = m_motor.getClosedLoopController();

      // Reset encoder position to zero on startup
      m_encoder.setPosition(0);
      
      // Initialize target position to current position
      m_targetPosition = m_encoder.getPosition();
  }

  @Override
  public void periodic() {
      // Apply gravity compensation based on arm angle
      // This is a simple cosine-based model - adjust as needed
      double gravityCompensation = Math.cos(getCurrentPosition() * 0.1) * 0.05;
      // Add compensation to the target position control
      if (Math.abs(getCurrentPosition() - m_targetPosition) < 0.5) {
          // Only apply when near the target to avoid interfering with main movement
          m_controller.setReference(m_targetPosition, ControlType.kPosition, ClosedLoopSlot.kSlot0, gravityCompensation);
      }

      
      
      // Publish to SmartDashboard for monitoring
      SmartDashboard.putNumber("Arm/Current Position", getCurrentPosition());
      SmartDashboard.putNumber("Arm/Target Position", m_targetPosition);
      SmartDashboard.putNumber("Arm/Position Error", m_targetPosition - getCurrentPosition());
      SmartDashboard.putBoolean("Arm/At Target", isAtPosition(m_targetPosition));
      SmartDashboard.putNumber("Arm/Gravity Compensation", gravityCompensation);
      SmartDashboard.putNumber("Arm/Motor Output", m_motor.getAppliedOutput());
      SmartDashboard.putNumber("Arm/Motor Current", m_motor.getOutputCurrent());
  }

  /**
   * Gets the current arm position
   * @return Position in rotations
   */
  public double getPosition() {
      return m_encoder.getPosition();
  }
  
  /**
   * Gets the current arm position (alias for getPosition)
   */
  public double getCurrentPosition() {
      return getPosition();
  }

  /**
   * Set a specific target position
   * @param position Target position in rotations
   */
  public void setPosition(double position) {
      // Enforce soft limits
      if (position > FORWARD_LIMIT) {
          position = FORWARD_LIMIT;
      } else if (position < REVERSE_LIMIT) {
          position = REVERSE_LIMIT;
      }
      
      // Store the target and apply to controller
      m_controller.setReference(m_targetPosition, ControlType.kPosition, ClosedLoopSlot.kSlot0, 0);
      m_controller.setReference(m_targetPosition, ControlType.kPosition);
  }

  /**
   * Increment the arm position
   * @param increment Amount to increment (positive or negative)
   */
  public void incrementPosition(double increment) {
      setPosition(m_targetPosition + increment);
  }

  /**
   * Check if arm is at the specified position
   * @param position Position to check against
   * @return true if within tolerance
   */
  public boolean isAtPosition(double position) {
      return Math.abs(getPosition() - position) < POSITION_TOLERANCE;
  }

  /**
   * Stop the arm motor and maintain position
   */
  public void stopAndHold() {
      // Update target to current position and hold there
      m_targetPosition = getPosition();
      m_controller.setReference(m_targetPosition, ControlType.kPosition);
  }

  // ==== COMMAND FACTORIES ====

  /**
   * Creates a command to increment the arm position up
   */
  public Command incrementUpCommand() {
      return this.run(() -> {
          double newTarget = getPosition() + INCREMENT_VALUE;
          if (newTarget <= FORWARD_LIMIT) {
              setPosition(newTarget);
          }
      }).withName("ArmIncrementUp");
  }

  /**
   * Creates a command to increment the arm position down
   */
  public Command incrementDownCommand() {
      return this.run(() -> {
          double newTarget = getPosition() - INCREMENT_VALUE;
          if (newTarget >= REVERSE_LIMIT) {
              setPosition(newTarget);
          }
      }).withName("ArmIncrementDown");
  }

  /**
   * Creates a command to move to a preset position
   * @param position Preset position to move to
   */
  public Command setPositionCommand(ArmPosition position) {
      return setPositionCommand(position.getPosition());
  }

  /**
   * Creates a command to move to a specific position
   * @param targetPosition Position to move to
   */
  public Command setPositionCommand(double targetPosition) {
      return Commands.sequence(
          // Set the position
          Commands.runOnce(() -> setPosition(targetPosition)),
          // Wait until we reach the target
          Commands.waitUntil(() -> isAtPosition(targetPosition))
      ).withName("MoveArmTo(" + targetPosition + ")");
  }

} // end of class