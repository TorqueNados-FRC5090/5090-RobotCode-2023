package frc.robot.utils;

// Imports
import java.util.Map;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import frc.robot.Constants.DriveConstants.ModulePosition;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.SwerveModule;

public class ShuffleboardContent {
    static ShuffleboardLayout boolsLayout;

    // Constructor
    public ShuffleboardContent() {}

    public static void initBooleanShuffleboard(SwerveModule sm) {
        ModulePosition modulePosition = sm.getModulePosition();
        int moduleNumber = modulePosition.ordinal();
        String abrev = sm.modAbrev[moduleNumber];

        ShuffleboardTab x = Shuffleboard.getTab("Drivetrain");

        x.addBoolean("DriveCAN" + abrev, () -> sm.driveMotorConnected)
            .withPosition(8, moduleNumber);
        x.addBoolean("TurnCAN" + abrev, () -> sm.turnMotorConnected)
            .withPosition(9, moduleNumber);
    }

    public static void initDriveShuffleboard(SwerveModule sm) {
        ModulePosition modulePosition = sm.getModulePosition();
        int moduleNumber = modulePosition.ordinal();
        String abrev = sm.modAbrev[moduleNumber];
        String driveLayout = modulePosition.toString() + " Drive";
        ShuffleboardLayout drLayout = Shuffleboard.getTab("Drivetrain")
            .getLayout(driveLayout, BuiltInLayouts.kList).withPosition(moduleNumber * 2, 0)
            .withSize(2, 2).withProperties(Map.of("Label position", "LEFT"));

        drLayout.addNumber("Drive Speed MPS " + abrev, () -> sm.getDriveVelocity());

        drLayout.addNumber("Drive Position " + abrev, () -> sm.getDrivePosition());

        drLayout.addNumber("App Output " + abrev,
            () -> sm.driveMotor.getAppliedOutput());

        drLayout.addNumber("Current Amps " + abrev,
            () -> sm.getDriveCurrent());

        drLayout.addNumber("Firmware" + abrev,
            () -> sm.driveMotor.getFirmwareVersion());
    }

    public static void initTurnShuffleboard(SwerveModule sm) {
        ModulePosition modulePosition = sm.getModulePosition();
        int moduleNumber = modulePosition.ordinal();
        String abrev = sm.modAbrev[moduleNumber];
        String turnLayout = modulePosition.toString() + " Turn";

        ShuffleboardLayout tuLayout = Shuffleboard.getTab("Drivetrain")
            .getLayout(turnLayout, BuiltInLayouts.kList).withPosition(moduleNumber * 2, 2)
            .withSize(2, 3).withProperties(Map.of("Label position", "LEFT"));

        tuLayout.addNumber("Turn Setpoint Deg " + abrev, () -> sm.angle);

        tuLayout.addNumber("Turn Enc Pos " + abrev,
            () -> sm.getTurnPosition() % 360);

        tuLayout.addNumber("Act Ang Deg " + abrev,
            () -> sm.actualAngleDegrees);

        tuLayout.addNumber("TurnAngleOut" + abrev,
            () -> sm.turningMotor.getAppliedOutput());

        tuLayout.addNumber("Position" + abrev, () -> sm.turnCANcoder.getMyPosition());

        tuLayout.addNumber("Current Amps" + abrev, () -> sm.getTurnCurrent());

        tuLayout.addNumber("Abs Offset" + abrev, () -> sm.turningEncoderOffset);

        tuLayout.addNumber("Firmware" + abrev,
            () -> sm.turningMotor.getFirmwareVersion());
    }

    public static void initCoderBooleanShuffleboard(SwerveModule sm) {
        ModulePosition modulePosition = sm.getModulePosition();
        int moduleNumber = modulePosition.ordinal();
        String abrev = sm.modAbrev[moduleNumber];
        ShuffleboardTab x = Shuffleboard.getTab("CanCoders");

        x.addBoolean("CANOK" + abrev, () -> sm.turnCoderConnected)
            .withPosition(8, moduleNumber);
        x.addBoolean("Fault" + abrev, () -> sm.turnCANcoder.getFaulted())
            .withPosition(9, moduleNumber);
    }

    public static void initCANCoderShuffleboard(SwerveModule sm) {
        ModulePosition modulePosition = sm.getModulePosition();
        int moduleNumber = modulePosition.ordinal();
        String abrev = sm.modAbrev[moduleNumber];
        String canCoderLayout = modulePosition.toString() + " CanCoder";

        ShuffleboardLayout coderLayout = Shuffleboard.getTab("CanCoders")
            .getLayout(canCoderLayout, BuiltInLayouts.kList).withPosition(moduleNumber * 2, 0)
            .withSize(2, 3).withProperties(Map.of("Label position", "LEFT"));

        coderLayout.addNumber("Position" + abrev,
            () -> sm.turnCANcoder.getMyPosition());
        coderLayout.addNumber("Abs Position" + abrev,
            () -> sm.turnCANcoder.getAbsolutePosition());
        coderLayout.addNumber("Velocity" + abrev,
            () -> sm.turnCANcoder.getVelValue());
        coderLayout.addString(" MagField " + abrev,
            () -> sm.turnCANcoder.getMagnetFieldStrength().toString());
        coderLayout.addNumber("Battery Volts" + abrev,
            () -> sm.turnCANcoder.getBatValue());
        coderLayout.addNumber("Bus Volts" + abrev,
            () -> sm.turnCANcoder.getBusVoltage());

        coderLayout.addNumber("Abs Offset" + abrev, () -> sm.turningEncoderOffset);

        coderLayout.addNumber("Firmware#" + abrev,
            () -> sm.turnCANcoder.getFirmwareVersion());
    }

    public static void initMisc(Drivetrain drive) {
        ShuffleboardTab drLayout1 = Shuffleboard.getTab("Drivetrain");

        drLayout1.addBoolean("FieldOr", () -> drive.fieldOriented).withPosition(8,4)
            .withSize(1, 1);
                        
        drLayout1.addNumber("GyroYaw", () -> drive.getHeadingDegrees()).withPosition(9,4)
            .withSize(1, 1);
    }
}
