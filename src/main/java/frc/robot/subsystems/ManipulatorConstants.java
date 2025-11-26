package frc.robot.subsystems;

public class ManipulatorConstants {
    public static final Integer kRotationalMotor = 12;

    public static final Integer kSideMotor = 13;

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
}

