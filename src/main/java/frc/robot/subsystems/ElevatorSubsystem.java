package frc.robot.subsystems;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Volts;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.RobotConfig.ElevatorConfig.ElevatorState;
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

public class ElevatorSubsystem extends SubsystemBase {

  private TalonFX leader = new TalonFX(0);

  private Follower follower = new Follower(0, false);

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

  public ElevatorSubsystem() {
    leader.getConfigurator().apply(talonFXConfiguration);
  }

  public Distance getHeight() {
    return Meters.of(leader.getPosition().getValueAsDouble() * kMetersPerRotation);
  }

  public void setHeight(double height) {
    leader.setControl(voltageRequest.withPosition(height));
  }

  public LinearVelocity getLinearVelocity() {
    return MetersPerSecond.of(leader.getVelocity().getValueAsDouble() * kMetersPerRotation);
  }

  public Voltage getVoltage() {
    return leader.getMotorVoltage().getValue();
  }

  public void reset() {
    this.setHeight(0.0);
  }

  public Command setElevatorHeight(ElevatorState height) {
    return run(
        () -> {
          setHeight(height.getHeight());
        });
  }
}
