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

public class CoralEffectorSubsystem extends SubsystemBase {

   

    // End Effector Wheel Motor Controller
    private SparkMax m_effectorWheel; // NEO motor
    
    /** Subsystem for controlling the end effector wheel */
    public CoralEffectorSubsystem() {
        // Configure the Spark MAX motor controller
        m_effectorWheel = new SparkMax(Constants.EFFECTOR_WHEEL_MOTOR_ID, MotorType.kBrushless);
        configureSparkMAX(m_effectorWheel, Constants.EFFECTOR_WHEEL_INVERT);
    }
    
    private void configureSparkMAX(SparkMax max, boolean reverse) {
        SparkMaxConfig config = new SparkMaxConfig();
        config.inverted(reverse).idleMode(IdleMode.kBrake);
        max.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }
    
    /**
     * Sets speed of the end effector wheel motor
     * @param speed The speed to set (-1.0 to 1.0)
     */
    public void setWheelSpeed(double speed) {
        m_effectorWheel.set(speed);
    }
    
    /**
     * Gets position of the end effector wheel motor
     * @return Current encoder position
     */
    public double getPosition() {
        return m_effectorWheel.getEncoder().getPosition();
    }
    
    /**
     * Sets speed of wheel motor to 0
     */
    public void stop() {
        setWheelSpeed(0);
    }
    
    @Override
    public void periodic() {
        // Publish encoder values to SmartDashboard
        SmartDashboard.putNumber("Effector Wheel/Speed", m_effectorWheel.getEncoder().getVelocity());
        SmartDashboard.putNumber("Effector Wheel/Current", m_effectorWheel.getOutputCurrent());
    }
}