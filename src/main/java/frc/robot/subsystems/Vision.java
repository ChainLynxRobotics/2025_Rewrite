package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import java.util.function.Consumer;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.subsystems.VisionConstants.*;

public class Vision extends SubsystemBase{

    private List<CamAndEstimator> cameras = new ArrayList<>();

    public Consumer<EstimatedRobotPose> updateDrivetrain;
    public record CamAndEstimator (PhotonPoseEstimator estimator, PhotonCamera camera) {}

    public Vision(Consumer<EstimatedRobotPose> updateDrivetrain) {
        this.updateDrivetrain = updateDrivetrain;

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

    public boolean checkTolerance(EstimatedRobotPose pose) {
        double poseAmbiguity = 1;
        double distanceAmbiguity = 1;

        for (PhotonTrackedTarget t : pose.targetsUsed) {
            if (t.poseAmbiguity != -1 && t.poseAmbiguity < poseAmbiguity) {
                poseAmbiguity = t.poseAmbiguity;
            }

            double dist =
            Math.sqrt(
                Math.pow(t.bestCameraToTarget.getX(), 2)
                    + Math.pow(t.bestCameraToTarget.getY(), 2));

            if (dist < distanceAmbiguity) {
                distanceAmbiguity = dist;
            }
        }

        if (poseAmbiguity >= AMBIGUITY_TOLERANCE) {
            return false;
        }
        if (distanceAmbiguity >= DISTANCE_TOLERANCE) {
            return false;
        }
        
        return true;
    }

    public void update() {
        for (var cameraRecord: cameras) {

            List<PhotonPipelineResult> data = cameraRecord.camera.getAllUnreadResults();

            for (PhotonPipelineResult result: data) {
                Optional<EstimatedRobotPose> optionalPoseResult = cameraRecord.estimator.update(result);
                if (optionalPoseResult.isEmpty()) {
                    continue;
                }
                EstimatedRobotPose poseResult = optionalPoseResult.get();
                if (checkTolerance(poseResult)) {
                    updateDrivetrain.accept(poseResult);
                }
            }
            
        }
    }

}
