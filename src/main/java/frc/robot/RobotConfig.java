package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;

public class RobotConfig {
  public static final class ElevatorConfig {
    // Elevator measurements
    public static final Distance kL1Height = Millimeters.of(185.42);
    public static final Distance kL2Height = Millimeters.of(337.82);
    public static final Distance kL3Height = Millimeters.of(711.2);
    public static final Distance kL4Height = Millimeters.of(1546.86);
    public static final Distance kHumanPlayerHeight = Millimeters.of(431.8);
    public static final Distance kL2AlgaeHeight = Millimeters.of(355.6);
    public static final Distance kL3AlgaeHeight = Millimeters.of(889);
    public static final int kMotorNumber = 2;
    public static final double kGearing = 3;
    public static final double kCarriageMass = 26.914545;
    public static final double kDrumRadius = 0.0285;
    public static final Distance kMinHeight = Millimeters.of(0.0);
    public static final Distance kMaxHeight = Millimeters.of(660.4 + 657.225 + 400.05);
    public static final boolean kSimulateGravity = true;
    public static final double kStartingHeight = 0.0;
    public static final double kStandardDeviation = 0.0;

    public static final Distance kPullyRadius = Millimeters.of(28.5);
    public static final Angle kFullExtensionAngle =
        Radians.of(kMaxHeight.div(kPullyRadius).magnitude());

    // PID constants
    public static final double kP = 6;
    public static final double kI = 0.0;
    public static final double kD = 0.0;

    // Feedforward constants
    public static final double kS = 0.0;
    public static final double kG = 2.13037;
    public static final double kV = 0.3375;
    public static final double kA = 0.035;

    // Trapezoid profile constraints constants (rotations per second)
    public static final AngularVelocity kMaxVelocity = RotationsPerSecond.of(0.5);
    public static final AngularAcceleration kMaxAcceleration = RotationsPerSecondPerSecond.of(0.5);

    // Conversion from motor rotations to elevator height
    public static final double kMetersPerRotation =
        (kMaxHeight).in(Meters) / kFullExtensionAngle.in(Rotations);

    // Elevator heights by state
    public enum ElevatorState {
      BOTTOM(kMinHeight),
      L1(kL1Height),
      L2(kL2Height),
      L3(kL3Height),
      L4(kL4Height),
      HUMANPLAYER(kHumanPlayerHeight),
      L2ALGAE(kL2AlgaeHeight),
      L3ALGAE(kL3AlgaeHeight);

      public final Distance height;

      ElevatorState(Distance height) {
        this.height = height;
      }

      public Distance getHeight() {
        return height;
      }
    }

    public static final ElevatorSim elevatorSim =
        new ElevatorSim(
            DCMotor.getKrakenX60(kMotorNumber),
            kGearing,
            kCarriageMass,
            kDrumRadius,
            kMinHeight.in(Millimeters),
            kMaxHeight.in(Millimeters),
            kSimulateGravity,
            kStartingHeight);

    public static final TalonFXConfiguration talonFXConfiguration = new TalonFXConfiguration();

    public static final Slot0Configs slot0Configs =
        new Slot0Configs()
            .withKP(kP)
            .withKI(kI)
            .withKD(kD)
            .withKV(kV)
            .withKA(kA)
            .withKG(kG)
            .withKS(kS);

    public static final MotionMagicConfigs motionMagicConfigs =
        new MotionMagicConfigs()
            .withMotionMagicCruiseVelocity(kMaxVelocity)
            .withMotionMagicAcceleration(kMaxAcceleration);
  }
}
