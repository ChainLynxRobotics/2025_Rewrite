package frc.robot;

import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.units.measure.Distance;

public class RobotConfig {
  public static final class ElevatorConfig {
    // Elevator measurements
    public static final Distance kL1Height = Meters.of(0.0);
    public static final Distance kL2Height = Meters.of(0.25);
    public static final Distance kL3Height = Meters.of(0.5);
    public static final Distance kL4Height = Meters.of(0.75);
    public static final Distance kHumanPlayerHeight = Meters.of(0.0);
    public static final Distance kL2AlgaeHeight = Meters.of(0.0);
    public static final Distance kL3AlgaeHeight = Meters.of(0.0);
    public static final int kMotorNumber = 2;
    public static final double kGearing = 3;
    public static final double kCarriageMass = 26.914545;
    public static final double kDrumRadius = 0.0285;
    public static final double kMinHeight = 0.0; // Meters
    public static final double kMaxHeight = 1.76; // Meters
    public static final boolean kSimulateGravity = true;
    public static final double kStartingHeight = 0.0;
    public static final double kStandardDeviation = 0.0;

    // PID constants
    public static final double kP = 0.2;
    public static final double kI = 0.025;
    public static final double kD = 0.05;

    // Feedforward constants
    public static final double kS = 0.0;
    public static final double kG = 0.0;
    public static final double kV = 0.0;
    public static final double kA = 0.0;

    // Trapezoid profile constraints constants
    public static final double kMaxVelocity = 1.0;
    public static final double kMaxAcceleration = 0.5;

    // Conversion from motor rotations to elevator height
    public static final double kMetersPerRotation = 0.17907078125;

    // Elevator heights by state
    public enum ElevatorState {
      L1(kL1Height),
      L2(kL2Height),
      L3(kL3Height),
      L4(kL4Height),
      HUMANPLAYER(kHumanPlayerHeight),
      L2ALGAE(kL2AlgaeHeight),
      L3ALGAE(kL3AlgaeHeight);

      private Distance height;

      ElevatorState(Distance height) {
        this.height = height;
      }

      public Distance getHeight() {
        return height;
      }
    }
  }
}