// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static org.junit.jupiter.api.Assertions.*;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.TankDriveSubsystem;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

/**
 * Unit and simulation tests for {@link TankDriveSubsystem} and {@link RobotContainer}.
 */
public class TankDriveSubsystemTest {
  private TankDriveSubsystem m_drive;

  @BeforeEach
  public void setUp() {
    assertTrue(HAL.initialize(500, 0));
    CommandScheduler.getInstance().cancelAll();
    CommandScheduler.getInstance().clearComposedCommands();
    m_drive = new TankDriveSubsystem();
  }

  @AfterEach
  public void tearDown() {
    CommandScheduler.getInstance().cancelAll();
    if (m_drive != null) {
      m_drive.stop();
      m_drive.close();
    }
  }

  @Test
  public void testTalonFXConfigurationParameters() {
    TalonFXConfiguration leftConfig =
        TankDriveSubsystem.createTalonFXConfiguration(InvertedValue.CounterClockwise_Positive);
    TalonFXConfiguration rightConfig =
        TankDriveSubsystem.createTalonFXConfiguration(InvertedValue.Clockwise_Positive);

    assertEquals(InvertedValue.CounterClockwise_Positive, leftConfig.MotorOutput.Inverted);
    assertEquals(InvertedValue.Clockwise_Positive, rightConfig.MotorOutput.Inverted);
    assertEquals(NeutralModeValue.Brake, leftConfig.MotorOutput.NeutralMode);
    assertEquals(NeutralModeValue.Brake, rightConfig.MotorOutput.NeutralMode);

    assertTrue(leftConfig.CurrentLimits.StatorCurrentLimitEnable);
    assertEquals(
        DriveConstants.kStatorCurrentLimitAmps, leftConfig.CurrentLimits.StatorCurrentLimit, 1e-6);
    assertTrue(leftConfig.CurrentLimits.SupplyCurrentLimitEnable);
    assertEquals(
        DriveConstants.kSupplyCurrentLimitAmps, leftConfig.CurrentLimits.SupplyCurrentLimit, 1e-6);

    assertEquals(
        DriveConstants.kGearRatio, leftConfig.Feedback.SensorToMechanismRatio, 1e-6);
    assertEquals(DriveConstants.kVelocityP, leftConfig.Slot0.kP, 1e-6);
    assertEquals(DriveConstants.kVelocityV, leftConfig.Slot0.kV, 1e-6);
    assertEquals(DriveConstants.kVelocityS, leftConfig.Slot0.kS, 1e-6);

    assertThrows(
        NullPointerException.class, () -> TankDriveSubsystem.createTalonFXConfiguration(null));
  }

  @Test
  public void testHardwareDeviceIds() {
    assertEquals(DriveConstants.kLeftLeaderMotorId, m_drive.getLeftLeader().getDeviceID());
    assertEquals(DriveConstants.kLeftFollowerMotorId, m_drive.getLeftFollower().getDeviceID());
    assertEquals(DriveConstants.kRightLeaderMotorId, m_drive.getRightLeader().getDeviceID());
    assertEquals(DriveConstants.kRightFollowerMotorId, m_drive.getRightFollower().getDeviceID());
  }

  @Test
  public void testTankDriveOpenLoopAndSquaring() {
    // Unsquared inputs: MathUtil.applyDeadband(0.5, 0.05) = (0.5 - 0.05) / (1.0 - 0.05) = 0.473684
    double expectedDeadbanded = (0.5 - OperatorConstants.kJoystickDeadband)
        / (1.0 - OperatorConstants.kJoystickDeadband);
    m_drive.tankDrive(0.5, -0.5, false);
    assertEquals(expectedDeadbanded, m_drive.getLeftCommandedDutyCycle(), 1e-4);
    assertEquals(-expectedDeadbanded, m_drive.getRightCommandedDutyCycle(), 1e-4);

    // Squared inputs: expectedDeadbanded^2 = 0.2243767
    m_drive.tankDrive(0.5, -0.5, true);
    assertEquals(
        expectedDeadbanded * expectedDeadbanded, m_drive.getLeftCommandedDutyCycle(), 1e-4);
    assertEquals(
        -m_drive.getLeftCommandedDutyCycle(), m_drive.getRightCommandedDutyCycle(), 1e-6);

    // Full forward
    m_drive.tankDrive(1.0, 1.0, true);
    assertEquals(1.0, m_drive.getLeftCommandedDutyCycle(), 1e-6);
    assertEquals(1.0, m_drive.getRightCommandedDutyCycle(), 1e-6);
  }

  @Test
  public void testDeadbandClampingAndNonFiniteInputs() {
    // Within deadband -> 0.0
    m_drive.tankDrive(0.03, -0.04, false);
    assertEquals(0.0, m_drive.getLeftCommandedDutyCycle(), 1e-6);
    assertEquals(0.0, m_drive.getRightCommandedDutyCycle(), 1e-6);

    // Out-of-bounds clamping -> [-1.0, 1.0]
    m_drive.tankDrive(2.5, -3.0, false);
    assertEquals(1.0, m_drive.getLeftCommandedDutyCycle(), 1e-6);
    assertEquals(-1.0, m_drive.getRightCommandedDutyCycle(), 1e-6);

    // Non-finite inputs -> 0.0
    m_drive.tankDrive(Double.NaN, Double.POSITIVE_INFINITY, false);
    assertEquals(0.0, m_drive.getLeftCommandedDutyCycle(), 1e-6);
    assertEquals(0.0, m_drive.getRightCommandedDutyCycle(), 1e-6);

    m_drive.setDutyCycle(Double.NEGATIVE_INFINITY, Double.NaN);
    assertEquals(0.0, m_drive.getLeftCommandedDutyCycle(), 1e-6);
    assertEquals(0.0, m_drive.getRightCommandedDutyCycle(), 1e-6);
  }

  @Test
  public void testArcadeAndCurvatureDrive() {
    // Pure forward arcade drive
    m_drive.arcadeDrive(1.0, 0.0, false);
    assertEquals(1.0, m_drive.getLeftCommandedDutyCycle(), 1e-6);
    assertEquals(1.0, m_drive.getRightCommandedDutyCycle(), 1e-6);

    // Point turn left (CCW+, zRotation > 0: left wheel backward, right wheel forward)
    m_drive.arcadeDrive(0.0, 1.0, false);
    assertEquals(-1.0, m_drive.getLeftCommandedDutyCycle(), 1e-6);
    assertEquals(1.0, m_drive.getRightCommandedDutyCycle(), 1e-6);

    // Curvature drive turn-in-place disabled when xSpeed == 0 -> outputs 0
    m_drive.curvatureDrive(0.0, 0.8, false);
    assertEquals(0.0, m_drive.getLeftCommandedDutyCycle(), 1e-6);
    assertEquals(0.0, m_drive.getRightCommandedDutyCycle(), 1e-6);

    // Curvature drive turn-in-place enabled when xSpeed == 0
    m_drive.curvatureDrive(0.0, 1.0, true);
    assertEquals(-1.0, m_drive.getLeftCommandedDutyCycle(), 1e-6);
    assertEquals(1.0, m_drive.getRightCommandedDutyCycle(), 1e-6);
  }

  @Test
  public void testVoltageControlAndClamping() {
    m_drive.driveVolts(6.0, -6.0);
    assertEquals(6.0, m_drive.getLeftCommandedVolts(), 1e-6);
    assertEquals(-6.0, m_drive.getRightCommandedVolts(), 1e-6);

    // Clamping to [-12V, 12V]
    m_drive.driveVolts(18.0, -20.0);
    assertEquals(12.0, m_drive.getLeftCommandedVolts(), 1e-6);
    assertEquals(-12.0, m_drive.getRightCommandedVolts(), 1e-6);

    // Non-finite safety
    m_drive.driveVolts(Double.NaN, Double.POSITIVE_INFINITY);
    assertEquals(0.0, m_drive.getLeftCommandedVolts(), 1e-6);
    assertEquals(0.0, m_drive.getRightCommandedVolts(), 1e-6);
  }

  @Test
  public void testClosedLoopVelocityAndChassisSpeeds() {
    m_drive.driveVelocity(2.0, 2.0);
    assertEquals(2.0, m_drive.getLeftTargetVelocityMetersPerSecond(), 1e-6);
    assertEquals(2.0, m_drive.getRightTargetVelocityMetersPerSecond(), 1e-6);
    assertTrue(m_drive.getLeftCommandedVolts() > 0.0);
    assertEquals(m_drive.getLeftCommandedVolts(), m_drive.getRightCommandedVolts(), 1e-6);

    // Desaturation via ChassisSpeeds
    m_drive.driveChassisSpeeds(new ChassisSpeeds(10.0, 0.0, 0.0));
    assertEquals(
        DriveConstants.kMaxLinearSpeedMetersPerSecond,
        m_drive.getLeftTargetVelocityMetersPerSecond(),
        1e-6);
    assertEquals(
        DriveConstants.kMaxLinearSpeedMetersPerSecond,
        m_drive.getRightTargetVelocityMetersPerSecond(),
        1e-6);

    // Null ChassisSpeeds stops the drivetrain safely
    m_drive.driveChassisSpeeds(null);
    assertEquals(0.0, m_drive.getLeftTargetVelocityMetersPerSecond(), 1e-6);
    assertEquals(0.0, m_drive.getRightTargetVelocityMetersPerSecond(), 1e-6);
    assertEquals(0.0, m_drive.getLeftCommandedVolts(), 1e-6);
  }

  @Test
  public void testBoostAndSlowModeScaling() {
    // Default normal mode
    assertFalse(m_drive.isBoostMode());
    assertFalse(m_drive.isSlowMode());
    assertEquals(OperatorConstants.kNormalSpeedScale, m_drive.getTeleopSpeedScale(), 1e-6);

    // Boost mode
    m_drive.setBoostMode(true);
    assertTrue(m_drive.isBoostMode());
    assertEquals(OperatorConstants.kBoostSpeedScale, m_drive.getTeleopSpeedScale(), 1e-6);

    // Boost takes priority if both boost and slow are enabled
    m_drive.setSlowMode(true);
    assertEquals(OperatorConstants.kBoostSpeedScale, m_drive.getTeleopSpeedScale(), 1e-6);

    // Slow mode when boost is disabled
    m_drive.setBoostMode(false);
    assertTrue(m_drive.isSlowMode());
    assertEquals(OperatorConstants.kSlowSpeedScale, m_drive.getTeleopSpeedScale(), 1e-6);

    m_drive.setSlowMode(false);

    // Verify tankDriveCommand uses teleop speed scale and boostModeCommand overrides to 1.0
    Command tankCmd = m_drive.tankDriveCommand(() -> 1.0, () -> 1.0);
    Command boostCmd = m_drive.boostModeCommand();

    tankCmd.initialize();
    tankCmd.execute();
    double normalDutyCycle = m_drive.getLeftCommandedDutyCycle();
    assertEquals(OperatorConstants.kNormalSpeedScale, normalDutyCycle, 1e-6);

    boostCmd.initialize();
    tankCmd.execute();
    assertEquals(1.0, m_drive.getLeftCommandedDutyCycle(), 1e-6);
    assertEquals(1.0, m_drive.getRightCommandedDutyCycle(), 1e-6);

    boostCmd.end(false);
    assertFalse(m_drive.isBoostMode());
    tankCmd.end(false);
    assertEquals(0.0, m_drive.getLeftCommandedDutyCycle(), 1e-6);
  }

  @Test
  public void testSimulationStraightDriveAndOdometry() {
    m_drive.resetOdometry(Pose2d.kZero);
    m_drive.driveVolts(8.0, 8.0);

    for (int i = 0; i < 50; i++) { // 1.0 second of simulated driving
      m_drive.simulationPeriodic();
      m_drive.periodic();
    }

    assertTrue(m_drive.getLeftPositionMeters() > 0.5);
    assertEquals(m_drive.getLeftPositionMeters(), m_drive.getRightPositionMeters(), 1e-4);
    assertTrue(m_drive.getLeftVelocityMetersPerSecond() > 0.5);
    assertEquals(
        m_drive.getLeftVelocityMetersPerSecond(),
        m_drive.getRightVelocityMetersPerSecond(),
        1e-4);
    assertTrue(m_drive.getPose().getX() > 0.5);
    assertEquals(0.0, m_drive.getPose().getY(), 1e-3);
    assertEquals(0.0, m_drive.getHeading().getDegrees(), 1e-3);
  }

  @Test
  public void testSimulationTurnInPlaceAndZeroHeading() {
    m_drive.resetOdometry(Pose2d.kZero);
    // Counter-clockwise turn: left backward (-6V), right forward (+6V)
    m_drive.driveVolts(-6.0, 6.0);

    for (int i = 0; i < 10; i++) {
      m_drive.simulationPeriodic();
      m_drive.periodic();
    }

    assertTrue(
        m_drive.getHeading().getDegrees() > 10.0,
        "Heading was: " + m_drive.getHeading().getDegrees());
    assertTrue(
        m_drive.getTurnRateDegreesPerSecond() > 10.0,
        "TurnRate was: " + m_drive.getTurnRateDegreesPerSecond());

    // Continue turning past +180 degrees to verify yaw rate remains positive across wraparound
    for (int i = 0; i < 20; i++) {
      m_drive.simulationPeriodic();
      m_drive.periodic();
      assertTrue(
          m_drive.getTurnRateDegreesPerSecond() > 0.0,
          "Yaw rate should remain positive across +/-180 deg wraparound");
    }

    // Zero heading preserves translation and resets heading to 0
    m_drive.zeroHeading();
    assertEquals(0.0, m_drive.getHeading().getDegrees(), 1e-4);
    assertEquals(0.0, m_drive.getPose().getRotation().getDegrees(), 1e-4);

    // Reset odometry to custom pose
    Pose2d targetPose = new Pose2d(3.2, -1.4, Rotation2d.fromDegrees(45.0));
    m_drive.resetOdometry(targetPose);
    assertEquals(3.2, m_drive.getPose().getX(), 1e-4);
    assertEquals(-1.4, m_drive.getPose().getY(), 1e-4);
    assertEquals(45.0, m_drive.getPose().getRotation().getDegrees(), 1e-4);
    assertEquals(0.0, m_drive.getLeftPositionMeters(), 1e-4);
    assertEquals(0.0, m_drive.getRightPositionMeters(), 1e-4);
  }

  @Test
  public void testDriveDistanceAutonomousCommand() {
    m_drive.resetOdometry(Pose2d.kZero);
    Command autoCmd = m_drive.driveDistanceCommand(1.0, 0.6);

    autoCmd.initialize();
    int steps = 0;
    while (!autoCmd.isFinished() && steps < 250) {
      autoCmd.execute();
      m_drive.simulationPeriodic();
      m_drive.periodic();
      steps++;
    }
    double avgDist = 0.5 * (m_drive.getLeftPositionMeters() + m_drive.getRightPositionMeters());
    autoCmd.end(false);

    assertTrue(
        autoCmd.isFinished(),
        "DriveDistance command should complete within 5 seconds, avgDist=" + avgDist);
    assertTrue(avgDist >= 1.0, "Robot should have traveled at least 1.0 meter, avgDist=" + avgDist);
    assertEquals(0.0, m_drive.getLeftCommandedDutyCycle(), 1e-6);
    assertEquals(0.0, m_drive.getRightCommandedDutyCycle(), 1e-6);

    // Reverse distance test (-0.5m)
    Command reverseCmd = m_drive.driveDistanceCommand(-0.5, 0.5);
    reverseCmd.initialize();
    steps = 0;
    while (!reverseCmd.isFinished() && steps < 250) {
      reverseCmd.execute();
      m_drive.simulationPeriodic();
      m_drive.periodic();
      steps++;
    }
    double afterReverseDist =
        0.5 * (m_drive.getLeftPositionMeters() + m_drive.getRightPositionMeters());
    reverseCmd.end(false);
    assertTrue(reverseCmd.isFinished(), "Reverse DriveDistance command should complete");
    assertTrue(afterReverseDist <= avgDist - 0.5 + 1e-3);

    // Invalid/zero distance or speed finishes immediately without hanging
    Command zeroDistCmd = m_drive.driveDistanceCommand(0.0, 0.5);
    zeroDistCmd.initialize();
    zeroDistCmd.execute();
    assertTrue(zeroDistCmd.isFinished());
    zeroDistCmd.end(false);

    Command zeroSpeedCmd = m_drive.driveDistanceCommand(1.0, 0.0);
    zeroSpeedCmd.initialize();
    zeroSpeedCmd.execute();
    assertTrue(zeroSpeedCmd.isFinished());
    zeroSpeedCmd.end(false);

    Command nanDistCmd = m_drive.driveDistanceCommand(Double.NaN, 0.5);
    nanDistCmd.initialize();
    nanDistCmd.execute();
    assertTrue(nanDistCmd.isFinished());
    nanDistCmd.end(false);
  }

  @Test
  public void testClosedLoopVelocitySimulationTracking() {
    m_drive.resetOdometry(Pose2d.kZero);
    m_drive.driveVelocity(2.0, 2.0);

    for (int i = 0; i < 60; i++) {
      m_drive.simulationPeriodic();
      m_drive.periodic();
    }

    assertTrue(m_drive.getLeftVelocityMetersPerSecond() > 1.2);
    assertEquals(
        m_drive.getLeftVelocityMetersPerSecond(),
        m_drive.getRightVelocityMetersPerSecond(),
        1e-4);

    // Command 0.0 m/s in closed-loop velocity mode and verify deceleration
    double peakVel = m_drive.getLeftVelocityMetersPerSecond();
    m_drive.driveVelocity(0.0, 0.0);
    for (int i = 0; i < 25; i++) {
      m_drive.simulationPeriodic();
      m_drive.periodic();
    }
    assertTrue(m_drive.getLeftVelocityMetersPerSecond() < peakVel * 0.5);
  }

  @Test
  public void testCanIdValidationAndNullChecks() {
    // Out-of-range CAN IDs
    assertThrows(
        IllegalArgumentException.class,
        () -> new TankDriveSubsystem(-1, 2, 3, 4, ""));
    assertThrows(
        IllegalArgumentException.class,
        () -> new TankDriveSubsystem(1, 63, 3, 4, ""));
    assertThrows(
        IllegalArgumentException.class,
        () -> new TankDriveSubsystem(1, 2, 3, -5, ""));

    // Duplicate motor CAN IDs
    assertThrows(
        IllegalArgumentException.class,
        () -> new TankDriveSubsystem(1, 1, 3, 4, ""));
    assertThrows(
        IllegalArgumentException.class,
        () -> new TankDriveSubsystem(1, 2, 1, 4, ""));

    // Null command suppliers
    assertThrows(
        NullPointerException.class, () -> m_drive.tankDriveCommand(null, () -> 0.0));
    assertThrows(
        NullPointerException.class, () -> m_drive.arcadeDriveCommand(() -> 0.0, null));
    assertThrows(
        NullPointerException.class,
        () -> m_drive.curvatureDriveCommand(() -> 0.0, () -> 0.0, null));

    // Null and NaN Pose2d in resetOdometry fallback to Pose2d.kZero
    m_drive.resetOdometry(null);
    assertEquals(Pose2d.kZero, m_drive.getPose());

    m_drive.resetOdometry(new Pose2d(Double.NaN, 1.0, Rotation2d.kZero));
    assertEquals(Pose2d.kZero, m_drive.getPose());
  }

  @Test
  public void testBrakeModeAndRobotLifecycle() {
    assertEquals(NeutralModeValue.Brake, m_drive.getNeutralMode());
    m_drive.setBrakeMode(false);
    assertEquals(NeutralModeValue.Coast, m_drive.getNeutralMode());
    m_drive.setBrakeMode(true);
    assertEquals(NeutralModeValue.Brake, m_drive.getNeutralMode());

    try (Robot robot = new Robot()) {
      assertNotNull(robot.getRobotContainer());
      assertNotNull(robot.getRobotContainer().getDriveSubsystem());
      assertNotNull(robot.getRobotContainer().getDriverController());
      assertNotNull(robot.getRobotContainer().getAutonomousCommand());

      robot.robotPeriodic();
      robot.disabledInit();
      robot.autonomousInit();
      robot.robotPeriodic();
      robot.teleopInit();
      robot.robotPeriodic();
      robot.testInit();
    }

    // Verify closing Robot unregistered its internal TankDriveSubsystem so m_drive is unaffected
    m_drive.driveVolts(6.0, 6.0);
    CommandScheduler.getInstance().run();
    assertEquals(6.0, m_drive.getLeftCommandedVolts(), 1e-6);
  }

  @Test
  public void testResetEncodersPreservesVelocityPoseAndHeadingAfterZeroHeading() {
    m_drive.stop();
    m_drive.resetOdometry(new Pose2d(1.0, 2.0, Rotation2d.fromDegrees(30.0)));
    assertEquals(30.0, m_drive.getHeading().getDegrees(), 1e-4);

    // Stationary tick after resetOdometry with non-zero heading must preserve 30 deg & 0 deg/s rate
    m_drive.simulationPeriodic();
    m_drive.periodic();
    assertEquals(30.0, m_drive.getHeading().getDegrees(), 1e-4);
    assertEquals(30.0, m_drive.getPose().getRotation().getDegrees(), 1e-4);
    assertEquals(0.0, m_drive.getTurnRateDegreesPerSecond(), 1e-4);

    // Drive in an arc so both translation, rotation, and velocities are non-zero
    m_drive.driveVolts(5.0, 9.0);
    for (int i = 0; i < 20; i++) {
      m_drive.simulationPeriodic();
      m_drive.periodic();
    }

    assertTrue(m_drive.getHeading().getDegrees() > 35.0);
    Pose2d poseBeforeZeroHeading = m_drive.getPose();

    // Zero heading and verify pose translation is preserved while rotation and Field2d become 0
    m_drive.zeroHeading();
    assertEquals(0.0, m_drive.getHeading().getDegrees(), 1e-4);
    assertEquals(0.0, m_drive.getPose().getRotation().getDegrees(), 1e-4);
    assertEquals(0.0, m_drive.getField2d().getRobotPose().getRotation().getDegrees(), 1e-4);
    assertEquals(poseBeforeZeroHeading.getX(), m_drive.getPose().getX(), 1e-4);
    assertEquals(poseBeforeZeroHeading.getY(), m_drive.getPose().getY(), 1e-4);

    // Drive straight along the new 0-degree heading
    m_drive.driveVolts(8.0, 8.0);
    for (int i = 0; i < 15; i++) {
      m_drive.simulationPeriodic();
      m_drive.periodic();
    }

    double leftVelBeforeReset = m_drive.getLeftVelocityMetersPerSecond();
    double rightVelBeforeReset = m_drive.getRightVelocityMetersPerSecond();
    Pose2d poseBeforeEncoderReset = m_drive.getPose();
    Rotation2d headingBeforeEncoderReset = m_drive.getHeading();
    assertTrue(leftVelBeforeReset > 1.0);
    assertTrue(m_drive.getLeftPositionMeters() > 0.5);

    // Reset encoders while moving: positions become 0, but velocity, pose, and heading stay intact
    m_drive.resetEncoders();
    assertEquals(0.0, m_drive.getLeftPositionMeters(), 1e-6);
    assertEquals(0.0, m_drive.getRightPositionMeters(), 1e-6);
    assertEquals(leftVelBeforeReset, m_drive.getLeftVelocityMetersPerSecond(), 1e-6);
    assertEquals(rightVelBeforeReset, m_drive.getRightVelocityMetersPerSecond(), 1e-6);
    assertEquals(poseBeforeEncoderReset.getX(), m_drive.getPose().getX(), 1e-6);
    assertEquals(poseBeforeEncoderReset.getY(), m_drive.getPose().getY(), 1e-6);
    assertEquals(
        headingBeforeEncoderReset.getDegrees(), m_drive.getHeading().getDegrees(), 1e-6);

    // Continue driving and verify odometry integrates smoothly from the preserved pose
    for (int i = 0; i < 10; i++) {
      m_drive.simulationPeriodic();
      m_drive.periodic();
    }
    assertTrue(m_drive.getLeftPositionMeters() > 0.1);
    assertTrue(m_drive.getPose().getX() > poseBeforeEncoderReset.getX() + 0.1);

    // Mid-cycle resetEncoders (after simulationPeriodic, before periodic) must not lose pending motion
    Pose2d poseBeforeStep = m_drive.getPose();
    m_drive.simulationPeriodic();
    double stepDeltaMeters = m_drive.getLeftPositionMeters();
    m_drive.resetEncoders();
    assertEquals(0.0, m_drive.getLeftPositionMeters(), 1e-6);
    assertEquals(0.0, m_drive.getRightPositionMeters(), 1e-6);
    assertTrue(m_drive.getPose().getX() > poseBeforeStep.getX());
    assertEquals(m_drive.getPose().getX(), m_drive.getField2d().getRobotPose().getX(), 1e-6);
    assertTrue(stepDeltaMeters > 0.0);
  }

  @Test
  public void testControlModeTransitionsAndCurrentTelemetryInSimulation() {
    m_drive.resetOdometry(Pose2d.kZero);

    // 1. Closed-loop velocity mode
    m_drive.driveVelocity(2.5, 2.5);
    for (int i = 0; i < 15; i++) {
      m_drive.simulationPeriodic();
      m_drive.periodic();
    }
    assertTrue(m_drive.getLeftVelocityMetersPerSecond() > 1.0);
    assertTrue(m_drive.getLeftStatorCurrentAmps() > 0.0);
    assertTrue(m_drive.getLeftSupplyCurrentAmps() > 0.0);

    // 2. Transition directly to Voltage mode
    m_drive.driveVolts(9.0, 9.0);
    assertEquals(0.0, m_drive.getLeftTargetVelocityMetersPerSecond(), 1e-6);
    assertEquals(9.0, m_drive.getLeftAppliedVolts(), 1e-6);
    for (int i = 0; i < 10; i++) {
      m_drive.simulationPeriodic();
      m_drive.periodic();
    }
    assertTrue(m_drive.getLeftVelocityMetersPerSecond() > 2.0);
    assertEquals(
        m_drive.getLeftStatorCurrentAmps() * 0.75, m_drive.getLeftSupplyCurrentAmps(), 1e-4);

    // 3. Transition to open-loop TankDrive (DutyCycle mode)
    m_drive.tankDrive(0.5, 0.5, false);
    for (int i = 0; i < 10; i++) {
      m_drive.simulationPeriodic();
      m_drive.periodic();
    }
    assertTrue(m_drive.getLeftCommandedDutyCycle() > 0.4);

    // 4. Transition to stop()
    m_drive.stop();
    assertEquals(0.0, m_drive.getLeftCommandedVolts(), 1e-6);
    assertEquals(0.0, m_drive.getLeftCommandedDutyCycle(), 1e-6);
    assertEquals(0.0, m_drive.getLeftSupplyCurrentAmps(), 1e-6);
  }
}
