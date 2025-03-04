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
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.sim.SparkFlexSim;
import com.revrobotics.sim.SparkLimitSwitchSim;
import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configs;
import frc.robot.Constants;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ArmSubsystem extends SubsystemBase {
  private final SparkMax m_motor;
//   private final PIDController m_pidController;
  private final RelativeEncoder m_encoder;
  
  // The two preset positions (in rotations)
  private final double POSITION_ONE = 0.0;  // Adjust based on your desired position
  private final double POSITION_TWO = 5.0;  // Adjust based on your desired position
  
  // Current target position
  private double m_targetPosition = 0;
  
  // Constants for PID and feed forward
  private static final double kP = 0.1;
  private static final double kI = 0;
  private static final double kD = 0;
  private static final double kFF = 0.05; // Feed forward to counteract gravity
  
  // Constants for position control
  private static final double MAX_OUTPUT = 0.4;  // Limits max speed of the arm
  private static final double MIN_OUTPUT = -0.4; // Limits max speed in reverse
  private static final double POSITION_TOLERANCE = 0.1; // Rotations tolerance
  
  public ArmSubsystem() {
    // Initialize the motor as a Neo550
    m_motor = new SparkMax(Constants.END_EFFECTOR_ARM_MOTOR_ID, MotorType.kBrushless);
    
    // Reset motor to factory defaults
    // m_motor.restoreFactoryDefaults();
    
    // Configure motor
    // m_motor.setIdleMode(IdleMode.kBrake);
    // m_motor.setIdleMode(IdleMode.kBrake); 
    // m_motor.setSmartCurrentLimit(20); // Appropriate for Neo550
    
    // Get the encoder
    m_encoder = m_motor.getEncoder();
    
    // Reset encoder position to zero
    m_encoder.setPosition(0);


    
    // Get the PID controller
    // m_pidController = m_motor. getPIDController();
    
    // // Configure PID controller
    // m_pidController.setP(kP);
    // m_pidController.setI(kI);
    // m_pidController.setD(kD);
    // m_pidController.setFF(kFF);
    // m_pidController.setOutputRange(MIN_OUTPUT, MAX_OUTPUT);
    
    // Set initial position target
    m_targetPosition = m_encoder.getPosition();
    
    // Save configuration (burnt to flash)
    // m_motor.burnFlash();
  }
  
  @Override
  public void periodic() {
    // Publish current position and target to SmartDashboard
    SmartDashboard.putNumber("Arm Position", m_encoder.getPosition());
    SmartDashboard.putNumber("Arm Target", m_targetPosition);
    SmartDashboard.putBoolean("Arm At Target", isAtTarget());
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
    // m_pidController.setReference(m_targetPosition, CANSparkMax.ControlType.kPosition);
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
    // m_pidController.setReference(m_targetPosition, CANSparkMax.ControlType.kPosition);
  }
}