package frc.robot.lists;

/* 
 * This class is only for holding ID numbers
 * for things like CAN channels and DIO ports.
 * 
 * DO NOT USE THIS FOR ANY OTHER PURPOSE
 * To access numbers in this file, statically import it like so:
 * import static frc.robot.lists.ID_Numbers.*;
 */

/**
 * Holds port and ID numbers.
 */
public final class ID_Numbers {
    /** Ports used by controllers. */
    public static final class ControllerPorts {
        public static final int DRIVER_PORT = 0;
        public static final int OPERATOR_PORT = 1;
    }

    // -------- DIO ports ---------

    // -------- Motor IDs ---------

    /** IDs used by the drivetrain.
        3X for turning, 4X for driving, 5X for abs encoders. */
    public static final class SwerveIDs {
        // Front left module
        public static final int FL_DRIVE_ID = 40;
        public static final int FL_TURN_ID = 30;
        public static final int FL_ENCODER_ID = 2;
        // Front right module
        public static final int FR_DRIVE_ID = 41;
        public static final int FR_TURN_ID = 31;
        public static final int FR_ENCODER_ID = 4;
        // Rear left module
        public static final int RL_DRIVE_ID = 43;
        public static final int RL_TURN_ID = 33;
        public static final int RL_ENCODER_ID = 3;
        // Rear right module
        public static final int RR_DRIVE_ID = 42;
        public static final int RR_TURN_ID = 32;
        public static final int RR_ENCODER_ID = 1;
    }
}