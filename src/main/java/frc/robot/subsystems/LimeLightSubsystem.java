package frc.robot.subsystems;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
//import frc.robot.LimelightHelpers;

public class LimeLightSubsystem extends SubsystemBase {
    public NetworkTableInstance limeLight;
    public NetworkTable limeLightDataTable;

    public LimeLightSubsystem() {
        super();
        //double tx = LimelightHelpers.getTX("");
        limeLight = NetworkTableInstance.getDefault();
        limeLightDataTable = limeLight.getTable("datatable");
    }

    public Boolean TargetPresent() {
        if (limeLightDataTable.getEntry("tv").getDouble(0) == 1)
            return true;
        else
            return false;
    }

    public double[] GetRobotPosition() {
        return limeLightDataTable.getEntry("botpose").getDoubleArray(new double[6]);
    }

    public double[] GetAprilTagDistanceFromRobot() {
        return limeLightDataTable.getEntry("targetpose_robotspace").getDoubleArray(new double[6]);
    }

    public double[] GetApriltagId() {
        return limeLightDataTable.getEntry("tid").getDoubleArray(new double[6]);
    }
}
