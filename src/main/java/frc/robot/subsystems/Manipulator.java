package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import static edu.wpi.first.units.Units.*;
import static frc.robot.subsystems.ManipulatorConstants.ManipulatorPosition;
import static frc.robot.subsystems.ManipulatorConstants.kVelocity;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class Manipulator extends SubsystemBase {

  

  final SparkMax rotationalMotor =
    new SparkMax(ManipulatorConstants.kRotationalMotor, MotorType.kBrushless);
  final SparkMax sideMotor =
    new SparkMax(ManipulatorConstants.kSideMotor, MotorType.kBrushless);

  final TrapezoidProfile movementProfileRot =
    new TrapezoidProfile(new Constraints(ManipulatorConstants.kMaxVel, ManipulatorConstants.kMaxAcc));
  final TrapezoidProfile movementProfileSide =
    new TrapezoidProfile(new Constraints(ManipulatorConstants.kMaxVel, ManipulatorConstants.kMaxAcc));


  public PIDController pidControlerRotate;

  public PIDController pidControlerSide;

  private TrapezoidProfile.State goalSetpointRot =
    new TrapezoidProfile.State();
  private TrapezoidProfile.State goalSetpointSide =
    new TrapezoidProfile.State();
  private TrapezoidProfile.State periodicSetpointRot =
    new TrapezoidProfile.State();
  private TrapezoidProfile.State periodicSetpointSide =
    new TrapezoidProfile.State();

  private final SparkMaxConfig motorConfig = new SparkMaxConfig();

  private TrapezoidProfile.State stopped;

  public Angle targetRotation = Rotation.of(0); 

  public AngularVelocity targetVelocity = RotationsPerSecond.of(0);

  public Manipulator() {
    motorConfig
        .smartCurrentLimit(30)
        .idleMode(IdleMode.kBrake);

    rotationalMotor.configure(
        motorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    sideMotor.configure(
        motorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    pidControlerRotate = new PIDController(ManipulatorConstants.kP, ManipulatorConstants.kI, ManipulatorConstants.kD);
    pidControlerRotate.setTolerance(ManipulatorConstants.kTolerance);

    pidControlerSide = new PIDController(ManipulatorConstants.kP, ManipulatorConstants.kI, ManipulatorConstants.kD);
    pidControlerSide.setTolerance(ManipulatorConstants.kTolerance);
    
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
    periodicSetpointRot = movementProfileRot.calculate(ManipulatorConstants.kT, periodicSetpointRot, goalSetpointRot);

    periodicSetpointSide = movementProfileSide.calculate(ManipulatorConstants.kT, periodicSetpointSide, goalSetpointSide);


    double speedRot = pidControlerRotate.calculate(getCurrentAngleRot(), periodicSetpointRot.position); //Get current angle Function

    double speedSide = pidControlerSide.calculate(getCurrentAngleSide(), periodicSetpointSide.position);

    rotationalMotor.set(speedRot);
    sideMotor.set(speedSide);
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

  public double getCurrentAngleRot() {
    return (rotationalMotor.getAbsoluteEncoder().getPosition() - ManipulatorConstants.kEncoderOffset) * 360
        + ManipulatorConstants.kCadPositionOffset;
  }

  public double getCurrentAngleSide() {
    return (sideMotor.getAbsoluteEncoder().getPosition() - ManipulatorConstants.kEncoderOffset) * 360
        + ManipulatorConstants.kCadPositionOffset;
  }


  public boolean isAtPositionRot() {
    return pidControlerRotate.atSetpoint();
  }

  public boolean isAtPositionSide() {
    return pidControlerSide.atSetpoint();
  }

  public void reset() {
    setGoalRot(Rotations.of(0));
    setGoalSide(RotationsPerSecond.of(0));
  }

  public void setGoalRot(Angle rotation) {
    goalSetpointRot = new TrapezoidProfile.State(rotation.baseUnitMagnitude(), 0);
    targetRotation = rotation;
  }

  protected void setGoalSide(AngularVelocity side) {
    goalSetpointSide = new TrapezoidProfile.State(side.baseUnitMagnitude(), 0);
    targetVelocity = side;
  }

  public Command moveToAngle(ManipulatorPosition position) {
    Angle angle = ManipulatorConstants.stateToAngle.get(position);
      return runOnce(() -> {
        setGoalRot(angle);
      }
    );
  }

  public Command goToVelocity() {
      return runOnce(() -> {
        setGoalSide(kVelocity);
      }
    );
  }
}
  