package frc.robot;

import edu.wpi.first.units.measure.Distance;

public class RobotConfig {
    public static final class ElevatorConfig{
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

        public static final double kMetersPerRotation = 0;
        //public static final Distance kMaxHeight = 3; //meters
    }
}
