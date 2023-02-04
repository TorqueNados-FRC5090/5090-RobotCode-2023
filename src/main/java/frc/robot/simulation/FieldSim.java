package frc.robot.simulation;

// Imports
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import java.util.Map;
import frc.robot.utils.ModuleMap;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.DriveConstants.ModulePosition;
import frc.robot.subsystems.Drivetrain;

// 
public class FieldSim {

    private final Drivetrain m_swerveDrive;
    private final Field2d m_field2d = new Field2d();

    private final Map<ModulePosition, Pose2d> m_swerveModulePoses =
        ModuleMap.of(new Pose2d(), new Pose2d(), new Pose2d(), new Pose2d());

    /**
     * @param drivetrain A drivetrain object to be used in simulation.
     */
    public FieldSim(Drivetrain swerveDrive) { 
        m_swerveDrive = swerveDrive; 
    }

    public void initSim() {}

    public Field2d getField2d() { return m_field2d; }

    private void updateRobotPoses() {
        m_field2d.setRobotPose(m_swerveDrive.getPoseMeters());

        for (ModulePosition i : ModulePosition.values()) {

        Translation2d updatedPositions = DriveConstants.kModuleTranslations
            .get(i)
            .rotateBy(m_swerveDrive.getPoseMeters().getRotation())
            .plus(m_swerveDrive.getPoseMeters().getTranslation());

        m_swerveModulePoses.put(
            i, 
            new Pose2d(
                updatedPositions,
                m_swerveDrive
                .getSwerveModule(i)
                .getHeadingRotation2d()
                .plus(m_swerveDrive.getHeadingRotation2d())));
        }

        m_field2d.getObject("Swerve Modules")
            .setPoses(ModuleMap.orderedValues(m_swerveModulePoses, new Pose2d[0]));
    }

    public void periodic() {
        updateRobotPoses();

        if (RobotBase.isSimulation())
            simulationPeriodic();

        SmartDashboard.putData("Field2d", m_field2d);
    }

    public void simulationPeriodic() {}
}
