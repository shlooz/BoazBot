package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import org.photonvision.PhotonCamera;

import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;

public class VisionSubsystem extends SubsystemBase {
    private final PhotonCamera limelight;

    public VisionSubsystem() {
        // Initialize the Limelight camera
        limelight = new PhotonCamera("limelight");
    }

    /**
     * Returns the distance from the robot in 3 dimensions (x, y, z).
     *
     * @return a double array representing the distance in meters [x, y, z]
     */
    public double[] get3DDistance() {
        PhotonPipelineResult result = limelight.getLatestResult();

        if (result.hasTargets()) {
            PhotonTrackedTarget target = result.getBestTarget();

            // Get the best target's position in 3D space
            Transform3d targetTransform = target.getBestCameraToTarget();
            Translation3d translation = targetTransform.getTranslation();

            // Convert from meters to desired units (if necessary)
            double x = translation.getX();
            double y = translation.getY();
            double z = translation.getZ();

            return new double[]{x, y, z};
        } else {
            // No targets found, return zeros or an appropriate default value
            return new double[]{0.0, 0.0, 0.0};
        }
    }

    @Override
    public void periodic() {
        System.out.println(get3DDistance()[0]);
        System.out.println(get3DDistance()[1]);
        System.out.println(get3DDistance()[2]);
        // This method will be called once per scheduler run
    }
}
