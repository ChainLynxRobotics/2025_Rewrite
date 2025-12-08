// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.epilogue.Epilogue;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.Logged.Importance;
import edu.wpi.first.epilogue.logging.errors.ErrorHandler;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

@Logged
public class Robot extends TimedRobot {

  private Command m_autonomousCommand;

  @Logged(importance = Importance.CRITICAL)
  private double wack = 5;

  private final RobotContainer m_robotContainer;

  public Robot() {

    m_robotContainer = new RobotContainer();

    // Initialize data logging.
    DataLogManager.start();

    Epilogue.configure(
        config -> {
          // Log only to disk, instead of the default NetworkTables logging
          // Note that this means data cannot be analyzed in realtime by a dashboard
          config.minimumImportance = Importance.DEBUG;

          // config.backend = new FileBackend(DataLogManager.getLog());

          if (isSimulation()) {
            // If running in simulation, then we'd want to re-throw any errors that
            // occur so we can debug and fix them!
            config.errorHandler = ErrorHandler.crashOnError();
          }

          // Change the root data path
          config.root = "Telemetry";
        });

    Epilogue.bind(this);
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void disabledExit() {}

  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void autonomousExit() {}

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  @Override
  public void teleopPeriodic() {}

  @Override
  public void teleopExit() {}

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void testExit() {}

  @Override
  public void simulationPeriodic() {}
}
