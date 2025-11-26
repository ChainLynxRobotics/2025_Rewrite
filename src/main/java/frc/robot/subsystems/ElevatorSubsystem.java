package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Rotations;
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
import static frc.robot.RobotConfig.ElevatorConfig.kStandardDeviation;
import static frc.robot.RobotConfig.ElevatorConfig.kStartingHeight;
import static frc.robot.RobotConfig.ElevatorConfig.kV;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.RobotConfig.ElevatorConfig.ElevatorState;

public class ElevatorSubsystem extends SubsystemBase {
  // ElevatorSim
  // TalonFX sim state
  // Call iterate
  private TalonFX leader = new TalonFX(13);

  private TalonFX follower = new TalonFX(14);

  private Follower followerBase = new Follower(13, true);

  private TalonFXSimState leaderSimState = leader.getSimState();

  private DutyCycleOut leaderDutyCycleOut = new DutyCycleOut(0.0);

  private TalonFXConfiguration talonFXConfiguration = new TalonFXConfiguration();

  private Slot0Configs slot0Configs =
      talonFXConfiguration
          .Slot0
          .withKP(kP)
          .withKI(kI)
          .withKD(kD)
          .withKV(kV)
          .withKA(kA)
          .withKG(kG)
          .withKS(kS);

  private MotionMagicConfigs motionMagicConfigs =
      talonFXConfiguration
          .MotionMagic
          .withMotionMagicCruiseVelocity(kMaxVelocity)
          .withMotionMagicAcceleration(kMaxAcceleration);

  private MotionMagicVoltage voltageRequest = new MotionMagicVoltage(0.0);

  private VoltageOut voltageOut = new VoltageOut(0.0);

  private SysIdRoutine sysIdRoutine =
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
          kMaxHeight,
          kSimulateGravity,
          kStartingHeight,
          kStandardDeviation);

  // Num motors, gearbox, weight, radius, min height, max height, simulate gravity, starting height,
  // stdev of measurement
  public ElevatorSubsystem() {
    leader.getConfigurator().apply(talonFXConfiguration);
    follower.setControl(followerBase);
  }

  @Override
  public void simulationPeriodic() {
    leaderSimState.setSupplyVoltage(Volts.of(12.0));
    elevatorSim.setInputVoltage(leaderSimState.getMotorVoltage());
    elevatorSim.update(0.02);
    leaderSimState.setRawRotorPosition(
        elevatorSim.getPositionMeters() / kMetersPerRotation * kGearing);
    leaderSimState.setRotorVelocity(elevatorSim.getVelocityMetersPerSecond() * kGearing);
  }

  public Angle angleOf(Distance distance) {
    return Rotations.of(distance.in(Meters) / kMetersPerRotation);
  }

  public Distance distanceOf(Angle angle) {
    return Meters.of(angle.in(Rotations) * kMetersPerRotation);
  }

  public Distance getHeight() {
    return distanceOf(leader.getPosition().getValue());
  }

  public void setHeight(Distance height) {
    leader.setControl(voltageRequest.withPosition(angleOf(height)));
  }

  public LinearVelocity getLinearVelocity() {
    return MetersPerSecond.of(leader.getVelocity().getValueAsDouble() * kMetersPerRotation);
  }

  public Voltage getVoltage() {
    return leader.getMotorVoltage().getValue();
  }

  public void reset() {
    this.setHeight(Meters.of(0.0));
  }

  public Command setElevatorHeight(ElevatorState height) {
    return run(
        () -> {
          setHeight(height.getHeight());
        });
  }
}
