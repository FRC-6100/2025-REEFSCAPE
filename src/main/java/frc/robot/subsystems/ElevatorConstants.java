package frc.robot.subsystems;

public class ElevatorConstants {

    // Preset positions (in rotations)
    public static final double POSITION_ZERO = 0.0;
    public static final double POSITION_ONE = -20.0; // FIXME This elevator value is random
    public static final double POSITION_TWO = -26.0; // FIXE This elevator value is random

    // PID and feed forward constants
    public static final double kP = 0.1;
    public static final double kI = 0;
    public static final double kD = 0;
    // private static final double kFF = 0.05; // Feed forward to counteract gravity

    // Position control constants
    public static final double SPEED_UP = 0.2; // Limits max speed of the Elevator
    public static final double SPEED_DOWN = -0.2; // Limits max speed in reverse
    public static final double POSITION_TOLERANCE = 0.1; // Rotations tolerance

    public static final boolean ELEVATOR_STAGE_1_INVERT = true;
    public static final boolean ELEVATOR_STAGE_2_INVERT = true;
    public static final boolean ELEVATOR_ARM_INVERT = true;

}
