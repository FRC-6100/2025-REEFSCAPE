// Author: Created March 2025
package frc.robot.subsystems;

import frc.robot.Constants;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class EndEffectorSubsystem extends SubsystemBase {

    // End Effector Motor Controllers
    private SparkMax m_end_effector_wheel; // NEO motor
    private SparkMax m_end_effector_arm; // NEO550 motor
    
    // Arm motor limits and parameters
    private double arm_max = 39.4;
    private double arm_min = 0;
    private double targetArmPosition = 0;
    private final double kP = 0.05; // Proportional control constant for arm position holding
    
    /** Subsystem for controlling the end effector (wheel and arm) */
    public EndEffectorSubsystem() {
        // Configure the Spark MAX motor controllers
        m_end_effector_wheel = new SparkMax(Constants.END_EFFECTOR_WHEEL_MOTOR_ID, MotorType.kBrushless);
        configureSparkMAX(m_end_effector_wheel, Constants.ELEVATOR_WHEEL_INVERT);
        
        m_end_effector_arm = new SparkMax(Constants.END_EFFECTOR_ARM_MOTOR_ID, MotorType.kBrushless);
        configureSparkMAX(m_end_effector_arm, Constants.ELEVATOR_ARM_INVERT);
        
        // Initialize target position to current position
        targetArmPosition = getArmPosition();
    }
    
    private void configureSparkMAX(SparkMax max, boolean reverse) {
        SparkMaxConfig config = new SparkMaxConfig();
        config.inverted(reverse).idleMode(IdleMode.kBrake);
        max.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }
    
    // Wheel Motor Methods ------------------------------------------------------------------------------
    
    /**
     * Sets speed of the end effector wheel motor
     * @param speed The speed to set (-1.0 to 1.0)
     */
    public void setWheelSpeed(double speed) {
        m_end_effector_wheel.set(speed);
    }
    
    /**
     * Gets position of the end effector wheel motor
     * @return Current encoder position
     */
    public double getWheelPosition() {
        return m_end_effector_wheel.getEncoder().getPosition();
    }
    
    /**
     * Sets speed of wheel motor to 0
     */
    public void stopWheel() {
        setWheelSpeed(0);
    }
    
    // Arm Motor Methods ------------------------------------------------------------------------------
    
    /**
     * Sets speed of the end effector arm motor with limits
     * @param speed The speed to set (-1.0 to 1.0)
     */
    public void setArmSpeed(double speed) {
        // Update the target position when manually controlling
        if (speed != 0) {
            targetArmPosition = getArmPosition();
        }
        
        // Apply limits to prevent exceeding boundaries
        if ((speed > 0) && (getArmPosition() > arm_max)) {
            m_end_effector_arm.set(0);
        } else if ((speed < 0) && (getArmPosition() < arm_min)) {
            m_end_effector_arm.set(0);
        } else {
            m_end_effector_arm.set(speed);
        }
    }
    
    /**
     * Gets position of the end effector arm motor
     * @return Current encoder position
     */
    public double getArmPosition() {
        return m_end_effector_arm.getEncoder().getPosition();
    }
    
    /**
     * Sets speed of arm motor to 0
     */
    public void stopArm() {
        m_end_effector_arm.set(0);
    }
    
    /**
     * Increments the arm position by a small amount
     * @param increment Amount to change position (positive or negative)
     */
    public void incrementArmPosition(double increment) {
        double newPosition = targetArmPosition + increment;
        
        // Ensure we stay within limits
        targetArmPosition = Math.min(Math.max(newPosition, arm_min), arm_max);
    }
    
    /**
     * Gets the current target position for the arm
     * @return Target position value
     */
    public double getArmTargetPosition() {
        return targetArmPosition;
    }
    
    /**
     * Holds the arm at the target position using proportional control
     */
    public void holdArmPosition() {
        double currentPosition = getArmPosition();
        double error = targetArmPosition - currentPosition;
        
        // Adjust power based on error
        double power = error * kP;
        
        // Limit power to a small value for careful movement
        power = Math.max(-0.15, Math.min(0.15, power));
        
        // Apply power
        m_end_effector_arm.set(power);
    }
    
    @Override
    public void periodic() {
        // Publish values to SmartDashboard
        SmartDashboard.putNumber("End Effector Wheel Position", getWheelPosition());
        SmartDashboard.putNumber("End Effector Arm Position", getArmPosition());
        SmartDashboard.putNumber("End Effector Arm Target", targetArmPosition);
    }
}