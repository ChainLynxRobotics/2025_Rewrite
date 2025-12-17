package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Meters;
import static frc.robot.subsystems.VisionConstants.*;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.*;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
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

public class Vision extends SubsystemBase {

  private List<CamAndEstimator> cameras = new ArrayList<>();

  public Consumer<VisionPose> updateDrivetrain;

  public record CamAndEstimator(PhotonPoseEstimator estimator, PhotonCamera camera) {}

  public record VisionPose(Pose3d pose, double timestamp, Matrix<N3, N1> deviation) {}

  public Vision(Consumer<VisionPose> updateDrivetrain) {
    this.updateDrivetrain = updateDrivetrain;

    cameras.add(
        new CamAndEstimator(
            new PhotonPoseEstimator(
                kTagLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, kCameraOffsets.get(0)),
            new PhotonCamera("aprilOne")));

    cameras.add(
        new CamAndEstimator(
            new PhotonPoseEstimator(
                kTagLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, kCameraOffsets.get(1)),
            new PhotonCamera("aprilTwo")));
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
              Math.pow(t.bestCameraToTarget.getX(), 2) + Math.pow(t.bestCameraToTarget.getY(), 2));

      if (dist < distanceAmbiguity) {
        distanceAmbiguity = dist;
      }
    }

    if (poseAmbiguity >= kAmbiguityTolerance) {
      return false;
    }
    if (distanceAmbiguity >= kDistanceTolerance) {
      return false;
    }

    return true;
  }

  public Distance getAverageDistance(EstimatedRobotPose pose) {
    Distance averageDistance = Meters.of(0);

    for (var target : pose.targetsUsed) {
      averageDistance.plus(
          Meters.of(target.bestCameraToTarget.getTranslation().getDistance(new Translation3d())));
    }

    return averageDistance.div(pose.targetsUsed.size());
  }

  public double getAverageAmbiguity(EstimatedRobotPose pose) {
    double averageAmbiguity = 0;

    for (var target : pose.targetsUsed) {
      averageAmbiguity += target.getPoseAmbiguity();
    }

    return averageAmbiguity / pose.targetsUsed.size();
  }

  public void update() {
    for (var cameraRecord : cameras) {

      List<PhotonPipelineResult> data = cameraRecord.camera.getAllUnreadResults();

      for (PhotonPipelineResult result : data) {
        Optional<EstimatedRobotPose> optionalPoseResult = cameraRecord.estimator.update(result);
        if (optionalPoseResult.isEmpty()) {
          continue;
        }
        EstimatedRobotPose poseResult = optionalPoseResult.get();
        if (checkTolerance(poseResult)) {
          Matrix<N3, N1> deviation =
              kBaseDeviation
                  .times(Math.pow(getAverageDistance(poseResult).in(Meters), 1.5))
                  .times(1 / Math.pow(poseResult.targetsUsed.size(), 2))
                  .times(Math.pow(getAverageAmbiguity(poseResult) * 10, 4));
          VisionPose swervePose =
              new VisionPose(poseResult.estimatedPose, result.getTimestampSeconds(), deviation);
          updateDrivetrain.accept(swervePose);
        }
      }
    }
  }
}
