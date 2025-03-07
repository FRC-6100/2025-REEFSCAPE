package frc.robot.subsystems;

public class AlgaeConstants {

  public static final double ARM_SPEED = 0.2;
  public static final double WHEEL_SPEED = 0.6;
  public static final double ARM_GRAVITY_CONST = -0.03; 
  // Number of rotations elevator climb motor must complete to raise/lower elevator by one inch
  public static final int ELEVATOR_ROTATIONS_PER_INCH = 13; 


  public static final double ALGAE_ARM_MAX_POWER = 0.1;
  public static final double ALGAE_ARM_MIN_POWER = 0.05;
  public static final double ALGAE_ARM_kP = 0.025;

  public static final boolean ALGAE_ARM_INVERT = true;
  public static final boolean AGLAE_BAR_INVERT = false;
  public static final double ALGAE_BAR_SPEED = 0.8;

    public static final double ALGAE_ARM_UP = 0.2; // Increase from 0.1
    public static final double ALGAE_ARM_DOWN = -0.2; // Increase from 0.1
  
    public static final double PICK_UP_ALGAE_POSITION = 33;
    public static final double HOLD_ALGAE_POSITION = 2.0;

  public static final double PICK_UP_CORAL_POSITION = 53;
  public static final double HOLD_CORAL_POSITION = 24;

  public static final double ALGAE_GRAVITY_RESISTANCE = 0.05; // TODO Adjust for gravity if needed

    // CORAL Constants
    public static final double INTAKE_LIFT_GEAR_RATIO = 3 * 7 * 7 * 48 / 29;

   public final static double CORAL_WHEEL_FORWARD = 0.6;  //
   public final static double CORAL_WHEEL_REVERSE = -0.6; //
   public final static double CORAL_ARM_FORWARD = 0.15; //
   public final static double CORAL_ARM_REVERSE = -0.15; //

    // Motor CAN IDs
    public static final int ALGAE_BAR_MOTOR_ID = 10; // Intake bar motor
    public static final int ALGAE_ARM_MOTOR_ID = 12; // Algae arm motor
    
    // Motor configuration
    public static final boolean INTAKE_BAR_INVERT = false;
    public static final boolean INTAKE_ARM_INVERT = true;
    
    // Position limits and setpoints
    public static final double INTAKE_DEPLOY_LIMIT = 51;
    public static final double INTAKE_RETURN_LIMIT = 0;
    public static final double PICK_UP_ALGAE_POSITION = 33;
    public static final double HOLD_ALGAE_POSITION = 2.0;
    
    // Motor control constants
    public static final double ALGAE_ARM_UP = 0.2;
    public static final double ALGAE_ARM_DOWN = -0.2;
    public static final double INTAKE_BAR_SPEED = 0.8;
    public static final double GRAVITY_RESISTANCE = 0.05; // Compensation for gravity
    
    // Control system constants
    public static final double INTAKE_ARM_MAX_POWER = 0.1;
    public static final double INTAKE_ARM_MIN_POWER = 0.05;
    public static final double INTAKE_ARM_kP = 0.025;
    
    // Gear ratios and mechanical constants
    public static final double INTAKE_LIFT_GEAR_RATIO = 3 * 7 * 7 * 48 / 29;
}

}
