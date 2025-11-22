package frc.robot.subsystems;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.DynamicMotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import static edu.wpi.first.units.Units.Meters;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.RobotConfig.ElevatorConfig.kA;
import static frc.robot.RobotConfig.ElevatorConfig.kD;
import static frc.robot.RobotConfig.ElevatorConfig.kG;
import static frc.robot.RobotConfig.ElevatorConfig.kI;
import static frc.robot.RobotConfig.ElevatorConfig.kMaxAcceleration;
import static frc.robot.RobotConfig.ElevatorConfig.kMaxVelocity;
import static frc.robot.RobotConfig.ElevatorConfig.kMetersPerRotation;
import static frc.robot.RobotConfig.ElevatorConfig.kP;
import static frc.robot.RobotConfig.ElevatorConfig.kS;
import static frc.robot.RobotConfig.ElevatorConfig.kV;


public class ElevatorSubsystem extends SubsystemBase{

    Constraints constraints = new Constraints(kMaxVelocity, kMaxAcceleration);

    private TrapezoidProfile trapezoidProfile = new TrapezoidProfile(constraints);

    private PIDController pid = new PIDController(kP, kI, kD);

    private TalonFX leader = new TalonFX(1);
    private TalonFX follower= new TalonFX(2);

    private DutyCycleOut leaderDutyCycleOut = new DutyCycleOut(0.0);
    private DutyCycleOut followerDutyCycleOut = new DutyCycleOut(0.0);

    private ElevatorFeedforward feedforward = new ElevatorFeedforward(kS, kG, kV, kA);

    private DynamicMotionMagicVoltage motionMagicVoltage = new DynamicMotionMagicVoltage(0, 0,0, 0);

    private Slot0Configs slot0Configs = new Slot0Configs().withKP(kP)
    .withKI(kI).withKD(kD).withKV(kV).withKA(kA).withKG(kG).withKS(kS);

    private TrapezoidProfile.State currentSetpoint = new TrapezoidProfile.State();
    private TrapezoidProfile.State goalSetpoint = new TrapezoidProfile.State();

    public ElevatorSubsystem(){
        leader.setControl(leaderDutyCycleOut);
        leader.getConfigurator().apply(slot0Configs);

        follower.setControl(followerDutyCycleOut);
        follower.getConfigurator().apply(slot0Configs);
    }

    public Distance getHeight() {
        return Meters.of(leader.getPosition().getValueAsDouble()*kMetersPerRotation);
    }

    @Override
    public void periodic(){
        currentSetpoint = trapezoidProfile.calculate(0.02, currentSetpoint, goalSetpoint);
        leader.set(pid.calculate(getHeight().in(Meters), currentSetpoint.position));
    }

    public void setHeight(double height){
        goalSetpoint = new TrapezoidProfile.State(height, 0);
    }

}