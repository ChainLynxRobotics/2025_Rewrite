package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;
import static frc.robot.RobotConfig.ElevatorConfig.kA;
import static frc.robot.RobotConfig.ElevatorConfig.kCarriageMass;
import static frc.robot.RobotConfig.ElevatorConfig.kD;
import static frc.robot.RobotConfig.ElevatorConfig.kDrumRadius;
import static frc.robot.RobotConfig.ElevatorConfig.kG;
import static frc.robot.RobotConfig.ElevatorConfig.kGearing;
import static frc.robot.RobotConfig.ElevatorConfig.kI;
import static frc.robot.RobotConfig.ElevatorConfig.kMaxAcceleration;
import static frc.robot.RobotConfig.ElevatorConfig.kMaxHeight;
import static frc.robot.RobotConfig.ElevatorConfig.kMaxVelocity;
import static frc.robot.RobotConfig.ElevatorConfig.kMetersPerRotation;
import static frc.robot.RobotConfig.ElevatorConfig.kMinHeight;
import static frc.robot.RobotConfig.ElevatorConfig.kMotorNumber;
import static frc.robot.RobotConfig.ElevatorConfig.kP;
import static frc.robot.RobotConfig.ElevatorConfig.kS;
import static frc.robot.RobotConfig.ElevatorConfig.kSimulateGravity;
import static frc.robot.RobotConfig.ElevatorConfig.kStartingHeight;
import static frc.robot.RobotConfig.ElevatorConfig.kV;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.Logged.Importance;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.RobotConfig.ElevatorConfig.ElevatorState;

@Logged
public class ElevatorSubsystem extends SubsystemBase {
  // ElevatorSim
  // TalonFX sim state
  // Call iterate
  private Distance goalHeight = Meters.of(0.0);

  private TalonFX leader = new TalonFX(13);

  private TalonFX follower = new TalonFX(14);

  private Follower followerBase = new Follower(13, true);

  private TalonFXSimState leaderSimState = leader.getSimState();

  private TalonFXConfiguration talonFXConfiguration = new TalonFXConfiguration();

  private Slot0Configs slot0Configs =
      new Slot0Configs()
          .withKP(kP)
          .withKI(kI)
          .withKD(kD)
          .withKV(kV)
          .withKA(kA)
          .withKG(kG)
          .withKS(kS);

  private MotionMagicConfigs motionMagicConfigs =
      new MotionMagicConfigs()
          .withMotionMagicCruiseVelocity(kMaxVelocity)
          .withMotionMagicAcceleration(kMaxAcceleration);

  private MotionMagicVoltage voltageRequest = new MotionMagicVoltage(0.0);

  private VoltageOut voltageOut = new VoltageOut(0.0);

  public SysIdRoutine sysIdRoutine =
      new SysIdRoutine(
          new SysIdRoutine.Config(),
          new SysIdRoutine.Mechanism(
              volts -> leader.setControl(voltageOut.withOutput(volts.in(Volts))),
              log ->
                  log.motor("Leader")
                      .linearPosition(this.getHeight())
                      .linearVelocity(getLinearVelocity())
                      .voltage(getVoltage()),
              this));

  ElevatorSim elevatorSim =
      new ElevatorSim(
          DCMotor.getKrakenX60(kMotorNumber),
          kGearing,
          kCarriageMass,
          kDrumRadius,
          kMinHeight,
          kMaxHeight.in(Meters),
          kSimulateGravity,
          kStartingHeight);

  // Num motors, gearbox, weight, radius, min height, max height, simulate gravity, starting height,
  // stdev of measurement
  public ElevatorSubsystem() {
    talonFXConfiguration.Slot0 = slot0Configs;
    talonFXConfiguration.MotionMagic = motionMagicConfigs;
    leader.getConfigurator().apply(talonFXConfiguration);
    follower.setControl(followerBase);
  }

  @Override
  public void periodic() {}

  @Override
  public void simulationPeriodic() {
    leaderSimState.setSupplyVoltage(Volts.of(12.0));
    elevatorSim.setInputVoltage(leaderSimState.getMotorVoltage());
    elevatorSim.update(0.02);
    leaderSimState.setRawRotorPosition(
        angleFromHeightOf(Meters.of(elevatorSim.getPositionMeters())));
    leaderSimState.setRotorVelocity(
        RadiansPerSecond.of(
            angleFromHeightOf(Meters.of(elevatorSim.getVelocityMetersPerSecond())).in(Radians)));
  }

  public Command sysIdDynamic(Direction direction) {
    return sysIdRoutine.dynamic(direction);
  }

  public Command sysIdQuasistatic(Direction direction) {
    return sysIdRoutine.quasistatic(direction);
  }

  public Angle angleFromHeightOf(Distance distance) {
    return Rotations.of(distance.in(Meters) / kMetersPerRotation);
  }

  public Distance HeightFromAngleOf(Angle angle) {
    return Meters.of(angle.in(Rotations) * kMetersPerRotation);
  }

  public AngularVelocity rotationsPerSecondOf(LinearVelocity velocity) {
    return RotationsPerSecond.of(velocity.in(MetersPerSecond) * kMetersPerRotation);
  }

  @Logged
  public Distance getHeight() {
    return HeightFromAngleOf(leader.getPosition().getValue());
  }

  public void setHeight(Distance height) {
    leader.setControl(voltageRequest.withPosition(angleFromHeightOf(height)));
  }

  @Logged
  public Distance getReference() {
    return HeightFromAngleOf(Rotations.of(leader.getClosedLoopReference().getValue()));
  }

  @Logged(importance = Importance.CRITICAL)
  public double getVelocity() {
    return elevatorSim.getVelocityMetersPerSecond();
  }

  public LinearVelocity getLinearVelocity() {
    return MetersPerSecond.of(leader.getVelocity().getValueAsDouble() * kMetersPerRotation);
  }

  @Logged
  public Voltage getVoltage() {
    return leader.getMotorVoltage().getValue();
  }

  public void reset() {
    this.setHeight(Meters.of(0.0));
  }

  @Logged
  public Distance getGoalHeight() {
    return goalHeight;
  }

  public Command setElevatorHeight(ElevatorState height) {
    return run(
        () -> {
          setHeight(height.getHeight());
          goalHeight = height.getHeight();
        });
  }
}
