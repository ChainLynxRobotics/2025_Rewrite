package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Volts;
import static edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction.kForward;
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

import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.DynamicMotionMagicVoltage;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

public class ElevatorSubsystem extends SubsystemBase {

  private Constraints constraints = new Constraints(kMaxVelocity, kMaxAcceleration);

  private TrapezoidProfile trapezoidProfile = new TrapezoidProfile(constraints);

  private PIDController pid = new PIDController(kP, kI, kD);

  private TalonFX leader = new TalonFX(0);

  private Follower follower = new Follower(0, false);

  private DutyCycleOut leaderDutyCycleOut = new DutyCycleOut(0.0);

  private ElevatorFeedforward feedforward = new ElevatorFeedforward(kS, kG, kV, kA);

  private DynamicMotionMagicVoltage motionMagicVoltage = new DynamicMotionMagicVoltage(0, 0, 0, 0);

  private VoltageOut voltageOut = new VoltageOut(0.0);

  private Slot0Configs slot0Configs =
      new Slot0Configs()
          .withKP(kP)
          .withKI(kI)
          .withKD(kD)
          .withKV(kV)
          .withKA(kA)
          .withKG(kG)
          .withKS(kS);

  FeedbackConfigs feedbackConfigs =
      new FeedbackConfigs().withSensorToMechanismRatio(kMetersPerRotation);

  private TrapezoidProfile.State currentSetpoint = new TrapezoidProfile.State();
  private TrapezoidProfile.State goalSetpoint = new TrapezoidProfile.State();

  SysIdRoutine sysIdRoutine =
      new SysIdRoutine(
          new SysIdRoutine.Config(),
          new SysIdRoutine.Mechanism(
              volts -> leader.setControl(voltageOut.withOutput(volts.in(Volts))),
              log ->
                  log.motor("Leader")
                      .angularAcceleration(null)
                      .angularPosition(null)
                      .angularVelocity(null)
                      .current(null)
                      .linearAcceleration(null)
                      .linearPosition(null)
                      .linearVelocity(null)
                      .voltage(null),
              this));

  public ElevatorSubsystem() {
    leader.setControl(leaderDutyCycleOut);
    leader.getConfigurator().apply(slot0Configs);
    leader.getConfigurator().apply(feedbackConfigs);
  }

  @Override
  public void periodic() {
    currentSetpoint = trapezoidProfile.calculate(0.02, currentSetpoint, goalSetpoint);
    leader.set(pid.calculate(getHeight(), currentSetpoint.position));
    sysIdRoutine.dynamic(kForward);
}

  public double getHeight() {
    return leader.getPosition().getValueAsDouble();
  }

  public void setHeight(double height) {
    goalSetpoint = new TrapezoidProfile.State(height, 0);
  }

  public void reset() {
    this.setHeight(0.0);
  }
}
