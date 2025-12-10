// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.Constants.ElevatorConfig.ElevatorState;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.ElevatorSubsystem;

@Logged
public class RobotContainer {
  private double MaxSpeed =
      TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
  private double MaxAngularRate =
      RotationsPerSecond.of(0.75)
          .in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

  /* Setting up bindings for necessary control of the swerve drive platform */
  private final SwerveRequest.FieldCentric drive =
      new SwerveRequest.FieldCentric()
          .withDeadband(MaxSpeed * 0.1)
          .withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
          .withDriveRequestType(
              DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
  private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
  private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

  private final Telemetry logger = new Telemetry(MaxSpeed);

  private final CommandXboxController commandXboxController = new CommandXboxController(0);

  private final Joystick joystick = new Joystick(1);

  public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

  public final ElevatorSubsystem elevator = new ElevatorSubsystem();

  public RobotContainer() {
    configureBindings();
  }

  private void configureBindings() {
    // Note that X is defined as forward according to WPILib convention,
    // and Y is defined as to the left according to WPILib convention.
    drivetrain.setDefaultCommand(
        // Drivetrain will execute this command periodically
        drivetrain.applyRequest(
            () ->
                drive
                    .withVelocityX(
                        -commandXboxController.getLeftY()
                            * MaxSpeed) // Drive forward with negative Y (forward)
                    .withVelocityY(
                        -commandXboxController.getLeftX()
                            * MaxSpeed) // Drive left with negative X (left)
                    .withRotationalRate(
                        -commandXboxController.getRightX()
                            * MaxAngularRate) // Drive counterclockwise with negative X (left)
            ));

    // Idle while the robot is disabled. This ensures the configured
    // neutral mode is applied to the drive motors while disabled.
    final var idle = new SwerveRequest.Idle();
    RobotModeTriggers.disabled()
        .whileTrue(drivetrain.applyRequest(() -> idle).ignoringDisable(true));

    commandXboxController.a().whileTrue(drivetrain.applyRequest(() -> brake));
    commandXboxController
        .b()
        .whileTrue(
            drivetrain.applyRequest(
                () ->
                    point.withModuleDirection(
                        new Rotation2d(
                            -commandXboxController.getLeftY(),
                            -commandXboxController.getLeftX()))));

    // Run SysId routines when holding back/start and X/Y.
    // Note that each routine should be run exactly once in a single log.
    commandXboxController
        .back()
        .and(commandXboxController.y())
        .whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
    commandXboxController
        .back()
        .and(commandXboxController.x())
        .whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
    commandXboxController
        .start()
        .and(commandXboxController.y())
        .whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
    commandXboxController
        .start()
        .and(commandXboxController.x())
        .whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

    // reset the field-centric heading on left bumper press
    commandXboxController
        .leftBumper()
        .onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

    drivetrain.registerTelemetry(logger::telemeterize);

    commandXboxController.x().onTrue(elevator.runSysId());

    commandXboxController.y().onTrue(elevator.setElevatorHeight(ElevatorState.L1));

    commandXboxController.a().onTrue(elevator.setElevatorHeight(ElevatorState.L2));

    commandXboxController.b().onTrue(elevator.setElevatorHeight(ElevatorState.L3));

    new Trigger(() -> joystick.getRawButton(1))
        .onTrue(elevator.setElevatorHeight(ElevatorState.BOTTOM));

    new Trigger(() -> joystick.getRawButton(2))
        .onTrue(elevator.setElevatorHeight(ElevatorState.L1));

    new Trigger(() -> joystick.getRawButton(3))
        .onTrue(elevator.setElevatorHeight(ElevatorState.L2));

    new Trigger(() -> joystick.getRawButton(4))
        .onTrue(elevator.setElevatorHeight(ElevatorState.L3));
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
