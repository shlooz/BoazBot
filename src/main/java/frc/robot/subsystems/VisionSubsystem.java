package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.LimelightHelpers;

import static frc.robot.LimelightHelpers.*;

public class VisionSubsystem extends SubsystemBase {
    private final NetworkTable limelight;
    private final LimelightHelpers limelighthHelpers;

    private NetworkTableEntry tx;
    private NetworkTableEntry ty;
    private NetworkTableEntry ta;
    private NetworkTableEntry td;

    public VisionSubsystem() {
        limelight = NetworkTableInstance.getDefault().getTable("limelight");
        limelighthHelpers = new LimelightHelpers();

        tx = limelight.getEntry("tx");
        ty = limelight.getEntry("ty");
        ta = limelight.getEntry("ta");
        td = limelight.getEntry("td");
    }

    public double getTx(){
        return tx.getDouble(0);
    }

    public double getTy(){
        return ty.getDouble(0);
    }

    public double getTa(){
        return ta.getDouble(0);
    }

    public double getTd(){
        return td.getDouble(0);
    }
    public double[] getDistance(){
        return getBotPose("limelight");
    }

    @Override
    public void periodic() {
        System.out.println(getDistance()[0]);
        System.out.println(getDistance()[1]);

      }


}
