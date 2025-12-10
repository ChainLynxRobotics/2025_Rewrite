package frc.robot.subsystems;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Transform3d;
import java.util.ArrayList;
import java.util.List;

public class VisionConstants {
  public static final AprilTagFieldLayout kTagLayout =
  AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);

  public static final double AMBIGUITY_TOLERANCE = 0.4; // percentage
  public static final double DISTANCE_TOLERANCE = 2.5; // meters

  public static List<Transform3d> cameraOffsets = new ArrayList<>();
}
