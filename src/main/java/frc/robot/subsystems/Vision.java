package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.List;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.subsystems.VisionConstants.*;

public class Vision extends SubsystemBase{

    private List<CamAndEstimator> cameras = new ArrayList<>();


    public record CamAndEstimator (PhotonPoseEstimator estimator, PhotonCamera camera) {}

    public Vision() {
        cameras.add(
            new CamAndEstimator(
                new PhotonPoseEstimator(kTagLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, cameraOffsets.get(0)),
                new PhotonCamera("aprilOne")
            )
        );

        cameras.add(
            new CamAndEstimator(
                new PhotonPoseEstimator(kTagLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, cameraOffsets.get(1)),
                new PhotonCamera("aprilTwo")
            )
        );
    }

}
