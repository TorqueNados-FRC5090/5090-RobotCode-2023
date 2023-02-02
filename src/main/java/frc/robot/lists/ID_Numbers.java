package frc.robot.lists;

/* 
 * This class is only for holding ID numbers for motors and input port numbers 
 * DO NOT USE THIS FOR ANY OTHER PURPOSE
 * To access numbers in this file, statically import it like so:
 * import static frc.robot.lists.ID_Numbers.*;
 */
public final class ID_Numbers {
    // Controller ports
    public static final int JOYSTICK_PORT = 0;
    public static final int CONTROLLER_PORT = 1;

    // Digital input ports

    // Motor IDs
    // Array of motor IDs for swerve 3x for turning and 4x for driving
    // Other numbers are for CANCoders
    public static final int[][] SWERVE_IDS = {
        {30,40, 2},
        {31,41, 4},
        {32,42, 1},
        {33,43, 3}
    };
}
