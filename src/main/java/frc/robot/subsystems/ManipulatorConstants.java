package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import java.util.Map;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
@Logged
public class ManipulatorConstants {
    public static final Integer kRotationalMotor = 5;

    public static final Integer kSideMotor = 6;

    public static final double kP = 0.020; // proportion
    public static final double kI = 0.002; // integral
    public static final double kD = 0.0; // derivative
    public static final double kT = 0.02; // time to next step

    public static final double kTolerance = 1;

    public static final double kMaxAcc = 250;
    public static final double kMaxVel = 350;

    public static final double kEncoderScale = 1;
    public static final double kCadPositionOffset = 50; // Adjustment for odd alignment in the CAD
    public static final double kEncoderOffset = 0.935;

    public static final double kManipulatorCir = 12.41;
    public static final double kManipulatorRatio = 4;


    public static final AngularVelocity kVelocity = RotationsPerSecond.of(13/(kManipulatorCir*kManipulatorRatio)); //13 is 13 meter per second launch speed.

    public enum ManipulatorPosition {
        STOWED,   
        L1_SCORE,
        L2_SCORE,
        L3_SCORE,
        L4_SCORE,
        L2_ALGAE,
        L3_ALGAE,
        HUMAN_PLAYER_STATION,
        NONE, 
    }
    public static Map<ManipulatorPosition, Angle> stateToAngle = Map.of(
        ManipulatorPosition.STOWED, Rotations.of(0),
        ManipulatorPosition.L1_SCORE, Rotations.of(0.45),
        ManipulatorPosition.L2_SCORE, Rotations.of(0.05),
        ManipulatorPosition.L3_SCORE, Rotations.of(0.05),
        ManipulatorPosition.L4_SCORE, Rotations.of(0.1),
        ManipulatorPosition.L2_ALGAE, Degrees.of(58),
        ManipulatorPosition.L3_ALGAE, Degrees.of(58),
        ManipulatorPosition.HUMAN_PLAYER_STATION, Rotations.of(0.34),
        ManipulatorPosition.NONE, Rotations.of(0.4)
    );
}
