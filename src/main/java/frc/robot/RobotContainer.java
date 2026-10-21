// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.TankDriveSubsystem;

/**
 * Container for the robot's subsystems, default commands, operator controller bindings, and
 * autonomous command selection.
 */
public class RobotContainer {
  private final TankDriveSubsystem m_driveSubsystem = new TankDriveSubsystem();

  private final CommandXboxController m_driverController =
      new CommandXboxController(OperatorConstants.kDriverControllerPort);

  private final SendableChooser<Command> m_autoChooser = new SendableChooser<>();

  public RobotContainer() {
    configureDefaultCommands();
    configureBindings();
    configureAutonomousChooser();
  }

  private void configureDefaultCommands() {
    // Default to Tank Drive: Left stick Y controls left side, Right stick Y controls right side.
    // Xbox controller Y-axes are negative when pushed forward, so negate both inputs.
    m_driveSubsystem.setDefaultCommand(
        m_driveSubsystem.tankDriveCommand(
            () -> -m_driverController.getLeftY(),
            () -> -m_driverController.getRightY()));
  }

  private void configureBindings() {
    // Right trigger or right bumper: Hold for Boost Mode (100% speed scale)
    m_driverController
        .rightTrigger(OperatorConstants.kTriggerThreshold)
        .or(m_driverController.rightBumper())
        .whileTrue(m_driveSubsystem.boostModeCommand());

    // Left trigger or left bumper: Hold for Slow/Precision Mode (35% speed scale)
    m_driverController
        .leftTrigger(OperatorConstants.kTriggerThreshold)
        .or(m_driverController.leftBumper())
        .whileTrue(m_driveSubsystem.slowModeCommand());

    // Hold 'A' button to switch to Arcade Drive (Left Y throttle, Right X rotation)
    m_driverController
        .a()
        .whileTrue(
            m_driveSubsystem.arcadeDriveCommand(
                () -> -m_driverController.getLeftY(),
                () -> -m_driverController.getRightX()));

    // Press 'B' button to zero the robot heading
    m_driverController
        .b()
        .onTrue(Commands.runOnce(m_driveSubsystem::zeroHeading).ignoringDisable(true));

    // Press 'X' button to immediately stop the drivebase
    m_driverController.x().onTrue(Commands.runOnce(m_driveSubsystem::stop, m_driveSubsystem));
  }

  private void configureAutonomousChooser() {
    m_autoChooser.setDefaultOption(
        "Drive Forward 2.0m", m_driveSubsystem.driveDistanceCommand(2.0, 0.5));
    m_autoChooser.addOption("Do Nothing", Commands.none());
    m_autoChooser.addOption(
        "Drive Forward & Back 1.5m",
        m_driveSubsystem
            .driveDistanceCommand(1.5, 0.5)
            .andThen(Commands.waitSeconds(0.5))
            .andThen(m_driveSubsystem.driveDistanceCommand(-1.5, 0.5)));

    SmartDashboard.putData("Auto Chooser", m_autoChooser);
  }

  /**
   * Returns the autonomous command selected on SmartDashboard.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    Command selected = m_autoChooser.getSelected();
    return selected != null ? selected : Commands.none();
  }

  /**
   * Returns the {@link TankDriveSubsystem} instance.
   *
   * @return the tank drive subsystem
   */
  public TankDriveSubsystem getDriveSubsystem() {
    return m_driveSubsystem;
  }

  /**
   * Returns the driver {@link CommandXboxController}.
   *
   * @return the driver controller
   */
  public CommandXboxController getDriverController() {
    return m_driverController;
  }
}
