package frc.robot;

public class RobotConfig {
    public static final class ElevatorConfig{
        //Elevator measurements (in meters)
        public static final double kBottomHeight = 0;
        public static final double kMiddleHeight = 0;        
        public static final double kMaxHeight = 0;

        //PID constants
        public static final double kP = 0.1;
        public static final double kI = 0.0;
        public static final double kD = 0.0;

        //Feedforward constants
        public static final double kS = 0.0;
        public static final double kG = 0.0;
        public static final double kV = 0.0;
        public static final double kA = 0.0;

        //Trapezoid profile constraints constants
        public static final double kMaxVelocity = 1.0;
        public static final double kMaxAcceleration = 0.5;

        //Conversion from motor rotations to elevator height
        public static final double kMetersPerRotation = 0;
    }
}
