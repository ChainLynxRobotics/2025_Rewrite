package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;
import static frc.robot.Constants.ElevatorConfig.elevatorSim;
import static frc.robot.Constants.ElevatorConfig.kMetersPerRotation;
import static frc.robot.Constants.ElevatorConfig.kT;
import static frc.robot.Constants.ElevatorConfig.motionMagicConfigs;
import static frc.robot.Constants.ElevatorConfig.simMotionMagicConfigs;
import static frc.robot.Constants.ElevatorConfig.simSlot0Configs;
import static frc.robot.Constants.ElevatorConfig.simTalonFXConfiguration;
import static frc.robot.Constants.ElevatorConfig.slot0Configs;
import static frc.robot.Constants.ElevatorConfig.talonFXConfiguration;

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.Logged.Importance;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.Constants.ElevatorConfig.ElevatorState;
import frc.robot.Robot;

@Logged
public class ElevatorSubsystem extends SubsystemBase {

  private Distance goalHeight = Meters.of(0.0); // For logging purposes

  private TalonFX leader = new TalonFX(14);

  private TalonFX follower = new TalonFX(15);

  private Follower followerBase = new Follower(14, true);

  private TalonFXSimState leaderSimState = leader.getSimState();

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

  // Num motors, gearbox, weight, radius, min height, max height, simulate gravity, starting height,
  // stdev of measurement
  public ElevatorSubsystem() {
    if (Robot.isReal()) {
      talonFXConfiguration.Slot0 = slot0Configs;
      talonFXConfiguration.MotionMagic = motionMagicConfigs;
      leader.getConfigurator().apply(talonFXConfiguration);
      follower.setControl(followerBase);
    } else if (Robot.isSimulation()) {
      simTalonFXConfiguration.Slot0 = simSlot0Configs;
      simTalonFXConfiguration.MotionMagic = simMotionMagicConfigs;
      leader.getConfigurator().apply(simTalonFXConfiguration);
      follower.setControl(followerBase);
    }
  }

  @Override
  public void periodic() {
    if (getHeight().gte(Meters.of(1.55))) {
      leader.stopMotor();
    }
  }

  @Override
  public void simulationPeriodic() {
    leaderSimState.setSupplyVoltage(Volts.of(12.0));
    elevatorSim.setInputVoltage(leaderSimState.getMotorVoltage());
    elevatorSim.update(kT);
    leaderSimState.setRawRotorPosition(
        angleFromHeightOf(Meters.of(elevatorSim.getPositionMeters())));
    leaderSimState.setRotorVelocity(
        RotationsPerSecond.of(
            angleFromHeightOf(Meters.of(elevatorSim.getVelocityMetersPerSecond())).in(Radians)));
  }

  @Logged
  public Command runSysId() {
    return sysIdRoutine
        .dynamic(Direction.kForward)
        .andThen(sysIdRoutine.dynamic(Direction.kReverse))
        .withTimeout(2.5)
        .andThen(sysIdRoutine.quasistatic(Direction.kReverse))
        .andThen(sysIdRoutine.quasistatic(Direction.kReverse));
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
