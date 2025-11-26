package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import static edu.wpi.first.units.Units.Rotation;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class Manipulator extends SubsystemBase {

  final SparkMax primaryMotor =
    new SparkMax(ManipulatorConstants.kPrimaryWristMotor, MotorType.kBrushless);
  final SparkMax secondaryMotor =
    new SparkMax(ManipulatorConstants.kSecondaryWristMotor, MotorType.kBrushless);
  final TrapezoidProfile movementProfile =
    new TrapezoidProfile(new Constraints(ManipulatorConstants.kMaxVel, ManipulatorConstants.kMaxAcc));

  public PIDController pidControler;

  private TrapezoidProfile.State goalSetpoint =
    new TrapezoidProfile.State();
  private TrapezoidProfile.State periodicSetpoint =
    new TrapezoidProfile.State();

  private final SparkMaxConfig motorConfig = new SparkMaxConfig();

  private TrapezoidProfile.State stopped;

  public Angle targetRotation = Rotation.of(0); 


  public Manipulator() {
    motorConfig
        .smartCurrentLimit(30)
        .idleMode(IdleMode.kBrake);

    primaryMotor.configure(
        motorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    secondaryMotor.configure(
        motorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    pidControler = new PIDController(ManipulatorConstants.kP, ManipulatorConstants.kI, ManipulatorConstants.kD);
    pidControler.setTolerance(ManipulatorConstants.kTolerance);
    reset(); //Add the Reset Function 
  }

  /**
   * Example command factory method.
   *
   * @return a command
   */
  public Command exampleMethodCommand() {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return runOnce(
        () -> {
          /* one-time action goes here */
        });
  }

  /**
   * An example method querying a boolean state of the subsystem (for example, a digital sensor).
   *
   * @return value of some boolean subsystem state, such as a digital sensor.
   */
  public boolean exampleCondition() {
    // Query some boolean state, such as a digital sensor.
    return false;
  }

  @Override
  public void periodic() {
    periodicSetpoint = movementProfile.calculate(ManipulatorConstants.kT, periodicSetpoint, goalSetpoint);

    double speed = pidControler.calculate(getCurrentAngle(), periodicSetpoint.position); //Get current angle Function
    setMotors(speed); //Set Motors Function
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

  public double getCurrentAngle() {
    return (primaryMotor.getAbsoluteEncoder().getPosition() - ManipulatorConstants.kEncoderOffset) * 360
        + ManipulatorConstants.kCadPositionOffset;
  }

  public boolean isAtPosition() {
    return pidControler.atSetpoint();
  }

  public void reset() {

    pidControler.reset();

    stopped = new TrapezoidProfile.State(getCurrentAngle(), 0);

    goalSetpoint = stopped;
    periodicSetpoint = stopped;

    setMotors(0);
  }

  private void setMotors(double speed) {
    primaryMotor.set(speed);
    secondaryMotor.set(-speed);
  }

  protected void setGoal(Angle rotation) {
    goalSetpoint = new TrapezoidProfile.State(rotation.baseUnitMagnitude(), 0);
    targetRotation = rotation;
  }

}