// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.sim.ChassisReference;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelPositions;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OperatorConstants;
import java.util.Objects;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

/**
 * Tank drive subsystem powered by four CTRE TalonFX (Kraken X60 / Falcon 500) motors (left leader,
 * left follower, right leader, right follower).
 *
 * <p>Supports open-loop tank/arcade/curvature drive (via Phoenix 6 {@link DutyCycleOut}), direct
 * voltage control ({@link VoltageOut}), and closed-loop wheel velocity control ({@link
 * VelocityVoltage}), along with full wheel-encoder odometry and WPILib physics simulation.
 */
public class TankDriveSubsystem extends SubsystemBase implements AutoCloseable {
  // ─── Hardware Devices ──────────────────────────────────────────────────────
  private final TalonFX m_leftLeader;
  private final TalonFX m_leftFollower;
  private final TalonFX m_rightLeader;
  private final TalonFX m_rightFollower;

  // ─── Reusable Phoenix 6 Control Requests ───────────────────────────────────
  private final DutyCycleOut m_leftDutyCycleRequest = new DutyCycleOut(0.0).withEnableFOC(true);
  private final DutyCycleOut m_rightDutyCycleRequest = new DutyCycleOut(0.0).withEnableFOC(true);

  private final VoltageOut m_leftVoltageRequest = new VoltageOut(0.0).withEnableFOC(true);
  private final VoltageOut m_rightVoltageRequest = new VoltageOut(0.0).withEnableFOC(true);

  private final VelocityVoltage m_leftVelocityRequest =
      new VelocityVoltage(0.0).withSlot(0).withEnableFOC(true);
  private final VelocityVoltage m_rightVelocityRequest =
      new VelocityVoltage(0.0).withSlot(0).withEnableFOC(true);

  // ─── Cached Status Signals ─────────────────────────────────────────────────
  private final StatusSignal<Angle> m_leftPositionSignal;
  private final StatusSignal<AngularVelocity> m_leftVelocitySignal;
  private final StatusSignal<Voltage> m_leftVoltageSignal;
  private final StatusSignal<Current> m_leftStatorCurrentSignal;
  private final StatusSignal<Current> m_leftSupplyCurrentSignal;

  private final StatusSignal<Angle> m_rightPositionSignal;
  private final StatusSignal<AngularVelocity> m_rightVelocitySignal;
  private final StatusSignal<Voltage> m_rightVoltageSignal;
  private final StatusSignal<Current> m_rightStatorCurrentSignal;
  private final StatusSignal<Current> m_rightSupplyCurrentSignal;

  // ─── Kinematics, Odometry & Telemetry ──────────────────────────────────────
  private final DifferentialDriveKinematics m_kinematics =
      new DifferentialDriveKinematics(DriveConstants.kTrackWidthMeters);
  private final DifferentialDriveOdometry m_odometry;
  private final Field2d m_field = new Field2d();
  private Rotation2d m_headingOffset = Rotation2d.kZero;

  private enum ControlMode {
    STOPPED,
    DUTY_CYCLE,
    VOLTAGE,
    VELOCITY
  }

  // ─── Speed Scaling / Boost / Control Tracking ──────────────────────────────
  private boolean m_boostMode = false;
  private boolean m_slowMode = false;
  private NeutralModeValue m_neutralMode = NeutralModeValue.Brake;
  private ControlMode m_controlMode = ControlMode.STOPPED;

  private double m_leftCommandedDutyCycle = 0.0;
  private double m_rightCommandedDutyCycle = 0.0;
  private double m_leftCommandedVolts = 0.0;
  private double m_rightCommandedVolts = 0.0;
  private double m_leftTargetVelocityMetersPerSecond = 0.0;
  private double m_rightTargetVelocityMetersPerSecond = 0.0;

  // ─── Physics Simulation & Position Offsets ─────────────────────────────────
  private final DifferentialDrivetrainSim m_drivetrainSim;
  private double m_leftPositionOffsetMeters = 0.0;
  private double m_rightPositionOffsetMeters = 0.0;

  /**
   * Constructs a {@link TankDriveSubsystem} using the default CAN IDs from {@link DriveConstants}.
   */
  public TankDriveSubsystem() {
    this(
        DriveConstants.kLeftLeaderMotorId,
        DriveConstants.kLeftFollowerMotorId,
        DriveConstants.kRightLeaderMotorId,
        DriveConstants.kRightFollowerMotorId,
        Constants.kCANBusName);
  }

  /**
   * Constructs a {@link TankDriveSubsystem} with explicit CAN IDs and CAN bus name.
   *
   * @param leftLeaderId CAN ID for the left leader TalonFX
   * @param leftFollowerId CAN ID for the left follower TalonFX
   * @param rightLeaderId CAN ID for the right leader TalonFX
   * @param rightFollowerId CAN ID for the right follower TalonFX
   * @param canBusName CAN bus name ("" for roboRIO bus, or CANivore name)
   */
  public TankDriveSubsystem(
      int leftLeaderId,
      int leftFollowerId,
      int rightLeaderId,
      int rightFollowerId,
      String canBusName) {
    try {
      validateCanIds(leftLeaderId, leftFollowerId, rightLeaderId, rightFollowerId);
    } catch (RuntimeException ex) {
      edu.wpi.first.wpilibj2.command.CommandScheduler.getInstance().unregisterSubsystem(this);
      throw ex;
    }
    CANBus canBus = new CANBus(canBusName == null ? "" : canBusName);

    m_leftLeader = new TalonFX(leftLeaderId, canBus);
    m_leftFollower = new TalonFX(leftFollowerId, canBus);
    m_rightLeader = new TalonFX(rightLeaderId, canBus);
    m_rightFollower = new TalonFX(rightFollowerId, canBus);

    // Configure left side (CounterClockwise_Positive) and right side (Clockwise_Positive)
    TalonFXConfiguration leftConfig =
        createTalonFXConfiguration(InvertedValue.CounterClockwise_Positive);
    TalonFXConfiguration rightConfig =
        createTalonFXConfiguration(InvertedValue.Clockwise_Positive);

    applyConfiguration(m_leftLeader, leftConfig, "LeftLeader");
    applyConfiguration(m_leftFollower, leftConfig, "LeftFollower");
    applyConfiguration(m_rightLeader, rightConfig, "RightLeader");
    applyConfiguration(m_rightFollower, rightConfig, "RightFollower");

    // Set followers to strictly match their respective leaders
    m_leftFollower.setControl(new Follower(leftLeaderId, MotorAlignmentValue.Aligned));
    m_rightFollower.setControl(new Follower(rightLeaderId, MotorAlignmentValue.Aligned));

    // Zero initial encoder positions
    m_leftLeader.setPosition(0.0);
    m_leftFollower.setPosition(0.0);
    m_rightLeader.setPosition(0.0);
    m_rightFollower.setPosition(0.0);

    // Cache status signals and configure 100Hz update frequency for odometry signals
    m_leftPositionSignal = m_leftLeader.getPosition();
    m_leftVelocitySignal = m_leftLeader.getVelocity();
    m_leftVoltageSignal = m_leftLeader.getMotorVoltage();
    m_leftStatorCurrentSignal = m_leftLeader.getStatorCurrent();
    m_leftSupplyCurrentSignal = m_leftLeader.getSupplyCurrent();

    m_rightPositionSignal = m_rightLeader.getPosition();
    m_rightVelocitySignal = m_rightLeader.getVelocity();
    m_rightVoltageSignal = m_rightLeader.getMotorVoltage();
    m_rightStatorCurrentSignal = m_rightLeader.getStatorCurrent();
    m_rightSupplyCurrentSignal = m_rightLeader.getSupplyCurrent();

    BaseStatusSignal.setUpdateFrequencyForAll(
        100.0,
        m_leftPositionSignal,
        m_leftVelocitySignal,
        m_rightPositionSignal,
        m_rightVelocitySignal);

    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0,
        m_leftVoltageSignal,
        m_leftStatorCurrentSignal,
        m_leftSupplyCurrentSignal,
        m_rightVoltageSignal,
        m_rightStatorCurrentSignal,
        m_rightSupplyCurrentSignal);

    // Configure simulation plant (2 Kraken X60 motors per side)
    m_drivetrainSim =
        new DifferentialDrivetrainSim(
            DCMotor.getKrakenX60Foc(2),
            DriveConstants.kGearRatio,
            DriveConstants.kRobotMomentOfInertiaKgMetersSquared,
            DriveConstants.kRobotMassKg,
            DriveConstants.kWheelRadiusMeters,
            DriveConstants.kTrackWidthMeters,
            null);

    m_leftLeader.getSimState().Orientation = ChassisReference.CounterClockwise_Positive;
    m_leftFollower.getSimState().Orientation = ChassisReference.CounterClockwise_Positive;
    m_rightLeader.getSimState().Orientation = ChassisReference.Clockwise_Positive;
    m_rightFollower.getSimState().Orientation = ChassisReference.Clockwise_Positive;

    m_odometry =
        new DifferentialDriveOdometry(
            getHeading(), getLeftPositionMeters(), getRightPositionMeters());

    SmartDashboard.putData("Drive/Field", m_field);
  }

  /**
   * Builds a {@link TalonFXConfiguration} with current limits, gearing ratio, ramp rates, and
   * velocity PID/FF gains.
   *
   * @param inverted motor inversion direction
   * @return configured {@link TalonFXConfiguration}
   */
  public static TalonFXConfiguration createTalonFXConfiguration(InvertedValue inverted) {
    TalonFXConfiguration config = new TalonFXConfiguration();

    // Motor output & neutral mode
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    config.MotorOutput.Inverted = Objects.requireNonNull(inverted, "inverted cannot be null");

    // Stator & supply current limits
    config.CurrentLimits.StatorCurrentLimit = DriveConstants.kStatorCurrentLimitAmps;
    config.CurrentLimits.StatorCurrentLimitEnable = true;
    config.CurrentLimits.SupplyCurrentLimit = DriveConstants.kSupplyCurrentLimitAmps;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;
    config.CurrentLimits.SupplyCurrentLowerLimit = DriveConstants.kSupplyCurrentLowerLimitAmps;
    config.CurrentLimits.SupplyCurrentLowerTime = DriveConstants.kSupplyCurrentLowerTimeSeconds;

    // Feedback sensor ratio: converts motor rotor rotations -> wheel mechanism rotations
    config.Feedback.SensorToMechanismRatio = DriveConstants.kGearRatio;

    // Open-loop and closed-loop voltage/duty-cycle ramp periods
    config.OpenLoopRamps.DutyCycleOpenLoopRampPeriod = DriveConstants.kOpenLoopRampPeriodSeconds;
    config.OpenLoopRamps.VoltageOpenLoopRampPeriod = DriveConstants.kOpenLoopRampPeriodSeconds;
    config.ClosedLoopRamps.VoltageClosedLoopRampPeriod =
        DriveConstants.kClosedLoopRampPeriodSeconds;

    // Slot 0 Closed-Loop Velocity Gains (Volts per wheel rotation/second)
    config.Slot0.kP = DriveConstants.kVelocityP;
    config.Slot0.kI = DriveConstants.kVelocityI;
    config.Slot0.kD = DriveConstants.kVelocityD;
    config.Slot0.kS = DriveConstants.kVelocityS;
    config.Slot0.kV = DriveConstants.kVelocityV;
    config.Slot0.kA = DriveConstants.kVelocityA;

    return config;
  }

  private void applyConfiguration(
      TalonFX motor, TalonFXConfiguration config, String motorDescription) {
    StatusCode status = StatusCode.StatusCodeNotInitialized;
    for (int attempt = 0; attempt < DriveConstants.kConfigMaxRetries; attempt++) {
      status = motor.getConfigurator().apply(config);
      if (status.isOK()) {
        return;
      }
    }
    DriverStation.reportWarning(
        "Failed to apply TalonFX configuration to "
            + motorDescription
            + " (CAN ID "
            + motor.getDeviceID()
            + "): "
            + status,
        false);
  }

  // ─── Open-Loop Drive Methods ───────────────────────────────────────────────

  /**
   * Drives the robot using tank drive controls (independent left and right sticks), squaring inputs
   * by default for finer low-speed control.
   *
   * @param leftSpeed left side speed in [-1.0, 1.0] (positive is forward)
   * @param rightSpeed right side speed in [-1.0, 1.0] (positive is forward)
   */
  public void tankDrive(double leftSpeed, double rightSpeed) {
    tankDrive(leftSpeed, rightSpeed, true);
  }

  /**
   * Drives the robot using tank drive controls (independent left and right sticks).
   *
   * @param leftSpeed left side speed in [-1.0, 1.0] (positive is forward)
   * @param rightSpeed right side speed in [-1.0, 1.0] (positive is forward)
   * @param squareInputs whether to square the joystick inputs while preserving sign
   */
  public void tankDrive(double leftSpeed, double rightSpeed, boolean squareInputs) {
    double sanitizedLeft = sanitizeInput(leftSpeed, OperatorConstants.kJoystickDeadband);
    double sanitizedRight = sanitizeInput(rightSpeed, OperatorConstants.kJoystickDeadband);

    DifferentialDrive.WheelSpeeds wheelSpeeds =
        DifferentialDrive.tankDriveIK(sanitizedLeft, sanitizedRight, squareInputs);

    setDutyCycle(wheelSpeeds.left, wheelSpeeds.right);
  }

  /**
   * Drives the robot using arcade drive controls (forward/backward throttle and rotation), squaring
   * inputs by default.
   *
   * @param xSpeed forward/backward throttle in [-1.0, 1.0] (positive is forward)
   * @param zRotation rotation rate in [-1.0, 1.0] (positive is counter-clockwise / left)
   */
  public void arcadeDrive(double xSpeed, double zRotation) {
    arcadeDrive(xSpeed, zRotation, true);
  }

  /**
   * Drives the robot using arcade drive controls (forward/backward throttle and rotation).
   *
   * @param xSpeed forward/backward throttle in [-1.0, 1.0] (positive is forward)
   * @param zRotation rotation rate in [-1.0, 1.0] (positive is counter-clockwise / left)
   * @param squareInputs whether to square the joystick inputs while preserving sign
   */
  public void arcadeDrive(double xSpeed, double zRotation, boolean squareInputs) {
    double sanitizedX = sanitizeInput(xSpeed, OperatorConstants.kJoystickDeadband);
    double sanitizedRot = sanitizeInput(zRotation, OperatorConstants.kJoystickDeadband);

    DifferentialDrive.WheelSpeeds wheelSpeeds =
        DifferentialDrive.arcadeDriveIK(sanitizedX, sanitizedRot, squareInputs);

    setDutyCycle(wheelSpeeds.left, wheelSpeeds.right);
  }

  /**
   * Drives the robot using curvature (Cheesy) drive controls.
   *
   * @param xSpeed forward/backward throttle in [-1.0, 1.0]
   * @param zRotation curvature / turn rate in [-1.0, 1.0]
   * @param allowTurnInPlace if true, allows point turns when xSpeed is near zero
   */
  public void curvatureDrive(double xSpeed, double zRotation, boolean allowTurnInPlace) {
    double sanitizedX = sanitizeInput(xSpeed, OperatorConstants.kJoystickDeadband);
    double sanitizedRot = sanitizeInput(zRotation, OperatorConstants.kJoystickDeadband);

    DifferentialDrive.WheelSpeeds wheelSpeeds =
        DifferentialDrive.curvatureDriveIK(sanitizedX, sanitizedRot, allowTurnInPlace);

    setDutyCycle(wheelSpeeds.left, wheelSpeeds.right);
  }

  /**
   * Directly sets the left and right motor duty cycle outputs in [-1.0, 1.0].
   *
   * @param leftDutyCycle left output in [-1.0, 1.0]
   * @param rightDutyCycle right output in [-1.0, 1.0]
   */
  public void setDutyCycle(double leftDutyCycle, double rightDutyCycle) {
    double clampedLeft =
        Double.isFinite(leftDutyCycle) ? MathUtil.clamp(leftDutyCycle, -1.0, 1.0) : 0.0;
    double clampedRight =
        Double.isFinite(rightDutyCycle) ? MathUtil.clamp(rightDutyCycle, -1.0, 1.0) : 0.0;

    m_controlMode = ControlMode.DUTY_CYCLE;
    m_leftCommandedDutyCycle = clampedLeft;
    m_rightCommandedDutyCycle = clampedRight;
    m_leftCommandedVolts = clampedLeft * DriveConstants.kMaxVoltage;
    m_rightCommandedVolts = clampedRight * DriveConstants.kMaxVoltage;
    m_leftTargetVelocityMetersPerSecond = 0.0;
    m_rightTargetVelocityMetersPerSecond = 0.0;

    m_leftLeader.setControl(m_leftDutyCycleRequest.withOutput(clampedLeft));
    m_rightLeader.setControl(m_rightDutyCycleRequest.withOutput(clampedRight));
  }

  /**
   * Directly commands left and right motor voltages, clamped to [-12.0, 12.0] V.
   *
   * @param leftVolts left side voltage in Volts
   * @param rightVolts right side voltage in Volts
   */
  public void driveVolts(double leftVolts, double rightVolts) {
    double clampedLeft =
        Double.isFinite(leftVolts)
            ? MathUtil.clamp(leftVolts, -DriveConstants.kMaxVoltage, DriveConstants.kMaxVoltage)
            : 0.0;
    double clampedRight =
        Double.isFinite(rightVolts)
            ? MathUtil.clamp(rightVolts, -DriveConstants.kMaxVoltage, DriveConstants.kMaxVoltage)
            : 0.0;

    m_controlMode = ControlMode.VOLTAGE;
    m_leftCommandedVolts = clampedLeft;
    m_rightCommandedVolts = clampedRight;
    m_leftCommandedDutyCycle = clampedLeft / DriveConstants.kMaxVoltage;
    m_rightCommandedDutyCycle = clampedRight / DriveConstants.kMaxVoltage;
    m_leftTargetVelocityMetersPerSecond = 0.0;
    m_rightTargetVelocityMetersPerSecond = 0.0;

    m_leftLeader.setControl(m_leftVoltageRequest.withOutput(clampedLeft));
    m_rightLeader.setControl(m_rightVoltageRequest.withOutput(clampedRight));
  }

  // ─── Closed-Loop Velocity Drive Methods ────────────────────────────────────

  /**
   * Drives the left and right wheels at the specified linear velocities using Phoenix 6 closed-loop
   * {@link VelocityVoltage} control on Slot 0.
   *
   * @param leftMetersPerSecond target left wheel linear velocity (m/s)
   * @param rightMetersPerSecond target right wheel linear velocity (m/s)
   */
  public void driveVelocity(double leftMetersPerSecond, double rightMetersPerSecond) {
    double safeLeftMps =
        Double.isFinite(leftMetersPerSecond)
            ? MathUtil.clamp(
                leftMetersPerSecond,
                -DriveConstants.kMaxLinearSpeedMetersPerSecond,
                DriveConstants.kMaxLinearSpeedMetersPerSecond)
            : 0.0;
    double safeRightMps =
        Double.isFinite(rightMetersPerSecond)
            ? MathUtil.clamp(
                rightMetersPerSecond,
                -DriveConstants.kMaxLinearSpeedMetersPerSecond,
                DriveConstants.kMaxLinearSpeedMetersPerSecond)
            : 0.0;

    m_controlMode = ControlMode.VELOCITY;
    m_leftTargetVelocityMetersPerSecond = safeLeftMps;
    m_rightTargetVelocityMetersPerSecond = safeRightMps;

    // SensorToMechanismRatio = kGearRatio, so Phoenix 6 expects wheel rotations per second
    double leftWheelRps = safeLeftMps / DriveConstants.kWheelCircumferenceMeters;
    double rightWheelRps = safeRightMps / DriveConstants.kWheelCircumferenceMeters;

    m_leftCommandedVolts =
        computeClosedLoopVoltageEstimate(
            leftWheelRps,
            getLeftVelocityMetersPerSecond() / DriveConstants.kWheelCircumferenceMeters);
    m_rightCommandedVolts =
        computeClosedLoopVoltageEstimate(
            rightWheelRps,
            getRightVelocityMetersPerSecond() / DriveConstants.kWheelCircumferenceMeters);
    m_leftCommandedDutyCycle = m_leftCommandedVolts / DriveConstants.kMaxVoltage;
    m_rightCommandedDutyCycle = m_rightCommandedVolts / DriveConstants.kMaxVoltage;

    m_leftLeader.setControl(m_leftVelocityRequest.withVelocity(leftWheelRps));
    m_rightLeader.setControl(m_rightVelocityRequest.withVelocity(rightWheelRps));
  }

  /**
   * Drives the robot according to a {@link ChassisSpeeds} command (vx and omega) using differential
   * drive inverse kinematics and closed-loop wheel velocity control.
   *
   * @param speeds desired robot-relative chassis speeds
   */
  public void driveChassisSpeeds(ChassisSpeeds speeds) {
    if (speeds == null) {
      stop();
      return;
    }
    DifferentialDriveWheelSpeeds wheelSpeeds = m_kinematics.toWheelSpeeds(speeds);
    wheelSpeeds.desaturate(DriveConstants.kMaxLinearSpeedMetersPerSecond);
    driveVelocity(wheelSpeeds.leftMetersPerSecond, wheelSpeeds.rightMetersPerSecond);
  }

  /**
   * Stops the drivetrain motors immediately.
   */
  public void stop() {
    m_controlMode = ControlMode.STOPPED;
    m_leftCommandedDutyCycle = 0.0;
    m_rightCommandedDutyCycle = 0.0;
    m_leftCommandedVolts = 0.0;
    m_rightCommandedVolts = 0.0;
    m_leftTargetVelocityMetersPerSecond = 0.0;
    m_rightTargetVelocityMetersPerSecond = 0.0;

    m_leftLeader.setControl(m_leftDutyCycleRequest.withOutput(0.0));
    m_rightLeader.setControl(m_rightDutyCycleRequest.withOutput(0.0));
    m_leftLeader.stopMotor();
    m_rightLeader.stopMotor();
  }

  /**
   * Sets the neutral mode (Brake or Coast) on all four TalonFX drive motors.
   *
   * @param enableBrake true for Brake mode, false for Coast mode
   */
  public void setBrakeMode(boolean enableBrake) {
    m_neutralMode = enableBrake ? NeutralModeValue.Brake : NeutralModeValue.Coast;
    m_leftLeader.setNeutralMode(m_neutralMode);
    m_leftFollower.setNeutralMode(m_neutralMode);
    m_rightLeader.setNeutralMode(m_neutralMode);
    m_rightFollower.setNeutralMode(m_neutralMode);
  }

  /**
   * Returns the currently configured neutral mode of the drive motors.
   *
   * @return {@link NeutralModeValue#Brake} or {@link NeutralModeValue#Coast}
   */
  public NeutralModeValue getNeutralMode() {
    return m_neutralMode;
  }

  // ─── Boost & Slow Speed Mode Controls ──────────────────────────────────────

  /**
   * Enables or disables boost mode (100% teleop speed multiplier).
   *
   * @param enabled true to enable boost mode
   */
  public void setBoostMode(boolean enabled) {
    m_boostMode = enabled;
  }

  /**
   * Returns whether boost mode is currently active.
   *
   * @return true if boost mode is active
   */
  public boolean isBoostMode() {
    return m_boostMode;
  }

  /**
   * Enables or disables slow/precision mode.
   *
   * @param enabled true to enable slow mode
   */
  public void setSlowMode(boolean enabled) {
    m_slowMode = enabled;
  }

  /**
   * Returns whether slow mode is currently active.
   *
   * @return true if slow mode is active
   */
  public boolean isSlowMode() {
    return m_slowMode;
  }

  /**
   * Returns the active teleop speed multiplier based on boost and slow mode states. Boost mode
   * takes priority over slow mode if both are set.
   *
   * @return speed scale factor in (0.0, 1.0]
   */
  public double getTeleopSpeedScale() {
    if (m_boostMode) {
      return OperatorConstants.kBoostSpeedScale;
    }
    if (m_slowMode) {
      return OperatorConstants.kSlowSpeedScale;
    }
    return OperatorConstants.kNormalSpeedScale;
  }

  // ─── Command Factories ─────────────────────────────────────────────────────

  /**
   * Creates a command that drives the robot in tank drive mode using the provided left and right
   * joystick suppliers, scaled by {@link #getTeleopSpeedScale()}.
   *
   * @param leftSupplier supplier for left stick forward input in [-1.0, 1.0]
   * @param rightSupplier supplier for right stick forward input in [-1.0, 1.0]
   * @return the tank drive command
   */
  public Command tankDriveCommand(DoubleSupplier leftSupplier, DoubleSupplier rightSupplier) {
    Objects.requireNonNull(leftSupplier, "leftSupplier cannot be null");
    Objects.requireNonNull(rightSupplier, "rightSupplier cannot be null");
    return run(
            () -> {
              double scale = getTeleopSpeedScale();
              double left =
                  sanitizeInput(leftSupplier.getAsDouble(), OperatorConstants.kJoystickDeadband);
              double right =
                  sanitizeInput(rightSupplier.getAsDouble(), OperatorConstants.kJoystickDeadband);
              DifferentialDrive.WheelSpeeds speeds =
                  DifferentialDrive.tankDriveIK(left, right, true);
              setDutyCycle(speeds.left * scale, speeds.right * scale);
            })
        .finallyDo(this::stop)
        .withName("TankDrive");
  }

  /**
   * Creates a command that drives the robot in arcade drive mode using the provided throttle and
   * turn suppliers, scaled by {@link #getTeleopSpeedScale()}.
   *
   * @param xSpeedSupplier supplier for forward/backward throttle in [-1.0, 1.0]
   * @param zRotationSupplier supplier for turn rate in [-1.0, 1.0]
   * @return the arcade drive command
   */
  public Command arcadeDriveCommand(
      DoubleSupplier xSpeedSupplier, DoubleSupplier zRotationSupplier) {
    Objects.requireNonNull(xSpeedSupplier, "xSpeedSupplier cannot be null");
    Objects.requireNonNull(zRotationSupplier, "zRotationSupplier cannot be null");
    return run(
            () -> {
              double scale = getTeleopSpeedScale();
              double xSpeed =
                  sanitizeInput(xSpeedSupplier.getAsDouble(), OperatorConstants.kJoystickDeadband);
              double zRotation =
                  sanitizeInput(
                      zRotationSupplier.getAsDouble(), OperatorConstants.kJoystickDeadband);
              DifferentialDrive.WheelSpeeds speeds =
                  DifferentialDrive.arcadeDriveIK(xSpeed, zRotation, true);
              setDutyCycle(speeds.left * scale, speeds.right * scale);
            })
        .finallyDo(this::stop)
        .withName("ArcadeDrive");
  }

  /**
   * Creates a command that drives the robot in curvature drive mode.
   *
   * @param xSpeedSupplier supplier for forward/backward throttle in [-1.0, 1.0]
   * @param zRotationSupplier supplier for curvature/rotation in [-1.0, 1.0]
   * @param allowTurnInPlaceSupplier supplier for whether turn-in-place is enabled
   * @return the curvature drive command
   */
  public Command curvatureDriveCommand(
      DoubleSupplier xSpeedSupplier,
      DoubleSupplier zRotationSupplier,
      BooleanSupplier allowTurnInPlaceSupplier) {
    Objects.requireNonNull(xSpeedSupplier, "xSpeedSupplier cannot be null");
    Objects.requireNonNull(zRotationSupplier, "zRotationSupplier cannot be null");
    Objects.requireNonNull(allowTurnInPlaceSupplier, "allowTurnInPlaceSupplier cannot be null");
    return run(
            () -> {
              double scale = getTeleopSpeedScale();
              double xSpeed =
                  sanitizeInput(xSpeedSupplier.getAsDouble(), OperatorConstants.kJoystickDeadband);
              double zRotation =
                  sanitizeInput(
                      zRotationSupplier.getAsDouble(), OperatorConstants.kJoystickDeadband);
              DifferentialDrive.WheelSpeeds speeds =
                  DifferentialDrive.curvatureDriveIK(
                      xSpeed, zRotation, allowTurnInPlaceSupplier.getAsBoolean());
              setDutyCycle(speeds.left * scale, speeds.right * scale);
            })
        .finallyDo(this::stop)
        .withName("CurvatureDrive");
  }

  /**
   * Returns a command that enables boost mode while scheduled and disables it when ended.
   *
   * @return boost mode command (does not require the drive subsystem so it can run alongside
   *     teleop driving)
   */
  public Command boostModeCommand() {
    return Commands.startEnd(() -> setBoostMode(true), () -> setBoostMode(false))
        .withName("BoostMode");
  }

  /**
   * Returns a command that enables slow mode while scheduled and disables it when ended.
   *
   * @return slow mode command (does not require the drive subsystem so it can run alongside teleop
   *     driving)
   */
  public Command slowModeCommand() {
    return Commands.startEnd(() -> setSlowMode(true), () -> setSlowMode(false))
        .withName("SlowMode");
  }

  /**
   * Creates an autonomous command that drives straight for a target distance in meters at the given
   * duty cycle magnitude, then stops.
   *
   * @param distanceMeters target distance in meters (positive = forward, negative = reverse)
   * @param speedDutyCycle duty cycle magnitude in (0.0, 1.0]
   * @return command that drives the specified distance
   */
  public Command driveDistanceCommand(double distanceMeters, double speedDutyCycle) {
    final double[] startAverageMeters = new double[1];
    final boolean validInputs =
        Double.isFinite(distanceMeters)
            && Double.isFinite(speedDutyCycle)
            && Math.abs(distanceMeters) > 1e-6
            && Math.abs(speedDutyCycle) > 1e-6;
    final double clampedSpeed =
        validInputs ? MathUtil.clamp(Math.abs(speedDutyCycle), 0.0, 1.0) : 0.0;
    final double direction = validInputs ? Math.signum(distanceMeters) : 0.0;
    final double targetMagnitude = validInputs ? Math.abs(distanceMeters) : 0.0;

    return startRun(
            () ->
                startAverageMeters[0] =
                    0.5 * (getLeftPositionMeters() + getRightPositionMeters()),
            () -> {
              if (validInputs) {
                setDutyCycle(direction * clampedSpeed, direction * clampedSpeed);
              } else {
                stop();
              }
            })
        .until(
            () -> {
              if (!validInputs) {
                return true;
              }
              double currentAverage =
                  0.5 * (getLeftPositionMeters() + getRightPositionMeters());
              double signedProgress = (currentAverage - startAverageMeters[0]) * direction;
              return signedProgress >= targetMagnitude;
            })
        .finallyDo(this::stop)
        .withName("DriveDistance");
  }

  // ─── Sensors, Odometry & State Getters ─────────────────────────────────────

  /**
   * Returns the raw (un-offset) distance traveled by the left side in meters.
   */
  private double getRawLeftPositionMeters() {
    if (!RobotBase.isReal()) {
      return m_drivetrainSim.getLeftPositionMeters();
    }
    return m_leftPositionSignal.getValueAsDouble() * DriveConstants.kWheelCircumferenceMeters;
  }

  /**
   * Returns the raw (un-offset) distance traveled by the right side in meters.
   */
  private double getRawRightPositionMeters() {
    if (!RobotBase.isReal()) {
      return m_drivetrainSim.getRightPositionMeters();
    }
    return m_rightPositionSignal.getValueAsDouble() * DriveConstants.kWheelCircumferenceMeters;
  }

  /**
   * Returns the distance traveled by the left side in meters.
   *
   * @return left wheel position in meters
   */
  public double getLeftPositionMeters() {
    return getRawLeftPositionMeters() - m_leftPositionOffsetMeters;
  }

  /**
   * Returns the distance traveled by the right side in meters.
   *
   * @return right wheel position in meters
   */
  public double getRightPositionMeters() {
    return getRawRightPositionMeters() - m_rightPositionOffsetMeters;
  }

  /**
   * Returns the linear velocity of the left wheels in meters per second.
   *
   * @return left wheel velocity in m/s
   */
  public double getLeftVelocityMetersPerSecond() {
    if (!RobotBase.isReal()) {
      return m_drivetrainSim.getLeftVelocityMetersPerSecond();
    }
    return m_leftVelocitySignal.getValueAsDouble() * DriveConstants.kWheelCircumferenceMeters;
  }

  /**
   * Returns the linear velocity of the right wheels in meters per second.
   *
   * @return right wheel velocity in m/s
   */
  public double getRightVelocityMetersPerSecond() {
    if (!RobotBase.isReal()) {
      return m_drivetrainSim.getRightVelocityMetersPerSecond();
    }
    return m_rightVelocitySignal.getValueAsDouble() * DriveConstants.kWheelCircumferenceMeters;
  }

  /**
   * Returns the current wheel positions for differential drive odometry.
   *
   * @return {@link DifferentialDriveWheelPositions}
   */
  public DifferentialDriveWheelPositions getWheelPositions() {
    return new DifferentialDriveWheelPositions(getLeftPositionMeters(), getRightPositionMeters());
  }

  /**
   * Returns the current wheel speeds for differential drive kinematics.
   *
   * @return {@link DifferentialDriveWheelSpeeds}
   */
  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(
        getLeftVelocityMetersPerSecond(), getRightVelocityMetersPerSecond());
  }

  /**
   * Returns the robot's current robot-relative {@link ChassisSpeeds}.
   *
   * @return current {@link ChassisSpeeds}
   */
  public ChassisSpeeds getChassisSpeeds() {
    return m_kinematics.toChassisSpeeds(getWheelSpeeds());
  }

  /**
   * Returns the current robot heading derived from differential drive wheel encoders (CCW+
   * convention).
   *
   * @return robot heading as a {@link Rotation2d}
   */
  public Rotation2d getHeading() {
    double wheelDeltaMeters = getRightPositionMeters() - getLeftPositionMeters();
    double angleRadians = wheelDeltaMeters / DriveConstants.kTrackWidthMeters;
    if (!Double.isFinite(angleRadians)) {
      return m_headingOffset;
    }
    return Rotation2d.fromRadians(angleRadians).plus(m_headingOffset);
  }

  /**
   * Returns the current yaw angular velocity in degrees per second (CCW+ convention), derived from
   * differential wheel speeds.
   *
   * @return yaw rate in deg/s
   */
  public double getTurnRateDegreesPerSecond() {
    double omegaRadPerSec = getChassisSpeeds().omegaRadiansPerSecond;
    return Double.isFinite(omegaRadPerSec) ? Units.radiansToDegrees(omegaRadPerSec) : 0.0;
  }

  /**
   * Returns the current estimated robot pose on the field.
   *
   * @return current {@link Pose2d}
   */
  public Pose2d getPose() {
    return m_odometry.getPoseMeters();
  }

  /**
   * Resets the robot's odometry to the given field pose and zeros the wheel encoders.
   *
   * @param pose new field pose (if null, defaults to {@link Pose2d#kZero})
   */
  public void resetOdometry(Pose2d pose) {
    Pose2d safePose = sanitizePose(pose);

    if (!RobotBase.isReal()) {
      m_drivetrainSim.setPose(safePose);
      m_leftLeader.getSimState().setRawRotorPosition(0.0);
      m_leftLeader.getSimState().setRotorVelocity(0.0);
      m_leftFollower.getSimState().setRawRotorPosition(0.0);
      m_leftFollower.getSimState().setRotorVelocity(0.0);
      m_rightLeader.getSimState().setRawRotorPosition(0.0);
      m_rightLeader.getSimState().setRotorVelocity(0.0);
      m_rightFollower.getSimState().setRawRotorPosition(0.0);
      m_rightFollower.getSimState().setRotorVelocity(0.0);
    }

    m_leftPositionOffsetMeters = getRawLeftPositionMeters();
    m_rightPositionOffsetMeters = getRawRightPositionMeters();
    m_headingOffset = safePose.getRotation();

    m_odometry.resetPosition(
        safePose.getRotation(), new DifferentialDriveWheelPositions(0.0, 0.0), safePose);
    m_field.setRobotPose(safePose);
  }

  /**
   * Resets both left and right drive encoders to 0.0 while preserving the current robot pose,
   * heading, and physical wheel velocities.
   */
  public void resetEncoders() {
    Rotation2d currentHeading = getHeading();
    Pose2d currentPose = m_odometry.update(currentHeading, getWheelPositions());

    if (!RobotBase.isReal()) {
      m_leftLeader.getSimState().setRawRotorPosition(0.0);
      m_leftFollower.getSimState().setRawRotorPosition(0.0);
      m_rightLeader.getSimState().setRawRotorPosition(0.0);
      m_rightFollower.getSimState().setRawRotorPosition(0.0);
    }

    m_leftPositionOffsetMeters = getRawLeftPositionMeters();
    m_rightPositionOffsetMeters = getRawRightPositionMeters();
    m_headingOffset = currentHeading;
    m_odometry.resetPosition(
        currentHeading, new DifferentialDriveWheelPositions(0.0, 0.0), currentPose);
    m_field.setRobotPose(currentPose);
  }

  /**
   * Zeros the robot's heading to 0 degrees while preserving the robot's X/Y translation and wheel
   * encoder positions.
   */
  public void zeroHeading() {
    Pose2d currentPose = m_odometry.update(getHeading(), getWheelPositions());
    double wheelDeltaMeters = getRightPositionMeters() - getLeftPositionMeters();
    double angleRadians = wheelDeltaMeters / DriveConstants.kTrackWidthMeters;
    m_headingOffset =
        Double.isFinite(angleRadians)
            ? Rotation2d.fromRadians(-angleRadians)
            : Rotation2d.kZero;
    Pose2d zeroedPose = new Pose2d(currentPose.getTranslation(), Rotation2d.kZero);
    m_odometry.resetPosition(Rotation2d.kZero, getWheelPositions(), zeroedPose);
    m_field.setRobotPose(zeroedPose);
  }

  // ─── Electrical & Commanded Telemetry Getters ──────────────────────────────

  public double getLeftCommandedDutyCycle() {
    return m_leftCommandedDutyCycle;
  }

  public double getRightCommandedDutyCycle() {
    return m_rightCommandedDutyCycle;
  }

  public double getLeftCommandedVolts() {
    return m_leftCommandedVolts;
  }

  public double getRightCommandedVolts() {
    return m_rightCommandedVolts;
  }

  public double getLeftTargetVelocityMetersPerSecond() {
    return m_leftTargetVelocityMetersPerSecond;
  }

  public double getRightTargetVelocityMetersPerSecond() {
    return m_rightTargetVelocityMetersPerSecond;
  }

  public double getLeftAppliedVolts() {
    if (!RobotBase.isReal()) {
      return m_leftCommandedVolts;
    }
    return m_leftVoltageSignal.getValueAsDouble();
  }

  public double getRightAppliedVolts() {
    if (!RobotBase.isReal()) {
      return m_rightCommandedVolts;
    }
    return m_rightVoltageSignal.getValueAsDouble();
  }

  public double getLeftStatorCurrentAmps() {
    if (!RobotBase.isReal()) {
      return m_drivetrainSim.getLeftCurrentDrawAmps();
    }
    return m_leftStatorCurrentSignal.getValueAsDouble();
  }

  public double getRightStatorCurrentAmps() {
    if (!RobotBase.isReal()) {
      return m_drivetrainSim.getRightCurrentDrawAmps();
    }
    return m_rightStatorCurrentSignal.getValueAsDouble();
  }

  public double getLeftSupplyCurrentAmps() {
    if (!RobotBase.isReal()) {
      return m_drivetrainSim.getLeftCurrentDrawAmps() * Math.abs(m_leftCommandedDutyCycle);
    }
    return m_leftSupplyCurrentSignal.getValueAsDouble();
  }

  public double getRightSupplyCurrentAmps() {
    if (!RobotBase.isReal()) {
      return m_drivetrainSim.getRightCurrentDrawAmps() * Math.abs(m_rightCommandedDutyCycle);
    }
    return m_rightSupplyCurrentSignal.getValueAsDouble();
  }

  // ─── Hardware & Utility Accessors ──────────────────────────────────────────

  public TalonFX getLeftLeader() {
    return m_leftLeader;
  }

  public TalonFX getLeftFollower() {
    return m_leftFollower;
  }

  public TalonFX getRightLeader() {
    return m_rightLeader;
  }

  public TalonFX getRightFollower() {
    return m_rightFollower;
  }

  public DifferentialDriveKinematics getKinematics() {
    return m_kinematics;
  }

  public Field2d getField2d() {
    return m_field;
  }

  @Override
  public void close() {
    edu.wpi.first.wpilibj2.command.CommandScheduler.getInstance().unregisterSubsystem(this);
    m_leftLeader.close();
    m_leftFollower.close();
    m_rightLeader.close();
    m_rightFollower.close();
  }

  // ─── Periodic & Simulation Loops ───────────────────────────────────────────

  @Override
  public void periodic() {
    BaseStatusSignal.refreshAll(
        m_leftPositionSignal,
        m_leftVelocitySignal,
        m_leftVoltageSignal,
        m_leftStatorCurrentSignal,
        m_leftSupplyCurrentSignal,
        m_rightPositionSignal,
        m_rightVelocitySignal,
        m_rightVoltageSignal,
        m_rightStatorCurrentSignal,
        m_rightSupplyCurrentSignal);

    Pose2d pose = m_odometry.update(getHeading(), getWheelPositions());
    m_field.setRobotPose(pose);

    SmartDashboard.putNumber("Drive/LeftPositionMeters", getLeftPositionMeters());
    SmartDashboard.putNumber("Drive/RightPositionMeters", getRightPositionMeters());
    SmartDashboard.putNumber("Drive/LeftVelocityMps", getLeftVelocityMetersPerSecond());
    SmartDashboard.putNumber("Drive/RightVelocityMps", getRightVelocityMetersPerSecond());
    SmartDashboard.putNumber("Drive/HeadingDegrees", getHeading().getDegrees());
    SmartDashboard.putNumber("Drive/LeftAppliedVolts", getLeftAppliedVolts());
    SmartDashboard.putNumber("Drive/RightAppliedVolts", getRightAppliedVolts());
    SmartDashboard.putBoolean("Drive/BoostMode", m_boostMode);
    SmartDashboard.putBoolean("Drive/SlowMode", m_slowMode);
    SmartDashboard.putNumber("Drive/SpeedScale", getTeleopSpeedScale());
  }

  @Override
  public void simulationPeriodic() {
    final double dtSeconds = 0.020;
    double batteryVolts = RobotController.getBatteryVoltage();
    if (!Double.isFinite(batteryVolts) || batteryVolts <= 0.0) {
      batteryVolts = DriveConstants.kMaxVoltage;
    }

    m_leftLeader.getSimState().setSupplyVoltage(batteryVolts);
    m_leftFollower.getSimState().setSupplyVoltage(batteryVolts);
    m_rightLeader.getSimState().setSupplyVoltage(batteryVolts);
    m_rightFollower.getSimState().setSupplyVoltage(batteryVolts);

    // If closed-loop velocity mode is active, update voltage estimate based on current sim velocity
    if (m_controlMode == ControlMode.VELOCITY) {
      double leftTargetRps =
          m_leftTargetVelocityMetersPerSecond / DriveConstants.kWheelCircumferenceMeters;
      double rightTargetRps =
          m_rightTargetVelocityMetersPerSecond / DriveConstants.kWheelCircumferenceMeters;
      double leftCurrentRps =
          m_drivetrainSim.getLeftVelocityMetersPerSecond()
              / DriveConstants.kWheelCircumferenceMeters;
      double rightCurrentRps =
          m_drivetrainSim.getRightVelocityMetersPerSecond()
              / DriveConstants.kWheelCircumferenceMeters;
      m_leftCommandedVolts = computeClosedLoopVoltageEstimate(leftTargetRps, leftCurrentRps);
      m_rightCommandedVolts = computeClosedLoopVoltageEstimate(rightTargetRps, rightCurrentRps);
      m_leftCommandedDutyCycle = m_leftCommandedVolts / DriveConstants.kMaxVoltage;
      m_rightCommandedDutyCycle = m_rightCommandedVolts / DriveConstants.kMaxVoltage;
    }

    m_drivetrainSim.setInputs(m_leftCommandedVolts, m_rightCommandedVolts);
    m_drivetrainSim.update(dtSeconds);

    // Convert wheel linear distance & velocity to motor rotor rotations (before gear reduction)
    double leftRotorRotations =
        (getLeftPositionMeters() / DriveConstants.kWheelCircumferenceMeters)
            * DriveConstants.kGearRatio;
    double leftRotorRps =
        (getLeftVelocityMetersPerSecond()
                / DriveConstants.kWheelCircumferenceMeters)
            * DriveConstants.kGearRatio;

    double rightRotorRotations =
        (getRightPositionMeters() / DriveConstants.kWheelCircumferenceMeters)
            * DriveConstants.kGearRatio;
    double rightRotorRps =
        (getRightVelocityMetersPerSecond()
                / DriveConstants.kWheelCircumferenceMeters)
            * DriveConstants.kGearRatio;

    m_leftLeader.getSimState().setRawRotorPosition(leftRotorRotations);
    m_leftLeader.getSimState().setRotorVelocity(leftRotorRps);
    m_leftFollower.getSimState().setRawRotorPosition(leftRotorRotations);
    m_leftFollower.getSimState().setRotorVelocity(leftRotorRps);

    m_rightLeader.getSimState().setRawRotorPosition(rightRotorRotations);
    m_rightLeader.getSimState().setRotorVelocity(rightRotorRps);
    m_rightFollower.getSimState().setRawRotorPosition(rightRotorRotations);
    m_rightFollower.getSimState().setRotorVelocity(rightRotorRps);
  }

  // ─── Internal Helpers ──────────────────────────────────────────────────────

  private static void validateCanIds(
      int leftLeaderId,
      int leftFollowerId,
      int rightLeaderId,
      int rightFollowerId) {
    validateSingleCanId(leftLeaderId, "leftLeaderId");
    validateSingleCanId(leftFollowerId, "leftFollowerId");
    validateSingleCanId(rightLeaderId, "rightLeaderId");
    validateSingleCanId(rightFollowerId, "rightFollowerId");

    int[] motorIds = {leftLeaderId, leftFollowerId, rightLeaderId, rightFollowerId};
    for (int i = 0; i < motorIds.length; i++) {
      for (int j = i + 1; j < motorIds.length; j++) {
        if (motorIds[i] == motorIds[j]) {
          throw new IllegalArgumentException(
              "Duplicate TalonFX CAN ID detected: " + motorIds[i]);
        }
      }
    }
  }

  private static void validateSingleCanId(int canId, String name) {
    if (canId < 0 || canId > 62) {
      throw new IllegalArgumentException(
          name + " must be in [0, 62], but was: " + canId);
    }
  }

  private static Pose2d sanitizePose(Pose2d pose) {
    if (pose == null || pose.getRotation() == null) {
      return Pose2d.kZero;
    }
    if (!Double.isFinite(pose.getX())
        || !Double.isFinite(pose.getY())
        || !Double.isFinite(pose.getRotation().getRadians())) {
      return Pose2d.kZero;
    }
    return pose;
  }

  private static double sanitizeInput(double input, double deadband) {
    if (!Double.isFinite(input)) {
      return 0.0;
    }
    double clamped = MathUtil.clamp(input, -1.0, 1.0);
    return MathUtil.applyDeadband(clamped, deadband);
  }

  private static double computeClosedLoopVoltageEstimate(
      double targetWheelRps, double currentWheelRps) {
    double staticTerm =
        Math.abs(targetWheelRps) > 1e-6
            ? Math.signum(targetWheelRps) * DriveConstants.kVelocityS
            : 0.0;
    double ff = staticTerm + targetWheelRps * DriveConstants.kVelocityV;
    double fb = (targetWheelRps - currentWheelRps) * DriveConstants.kVelocityP;
    return MathUtil.clamp(ff + fb, -DriveConstants.kMaxVoltage, DriveConstants.kMaxVoltage);
  }
}
