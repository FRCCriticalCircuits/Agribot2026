// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.util.Units;

/**
 * Robot-wide numerical and boolean constants for the TalonFX Tank Drive robot.
 */
public final class Constants {
  private Constants() {}

  /** CAN bus name for CTRE Phoenix 6 devices ("" for default roboRIO bus, or "canivore"). */
  public static final String kCANBusName = "";

  public static final class OperatorConstants {
    private OperatorConstants() {}

    public static final int kDriverControllerPort = 0;
    public static final double kJoystickDeadband = 0.05;

    /** Standard teleop speed scale factor when neither boost nor slow mode is active. */
    public static final double kNormalSpeedScale = 0.75;

    /** Full-output speed scale factor when boost mode is active. */
    public static final double kBoostSpeedScale = 1.00;

    /** Precision speed scale factor when slow mode is active. */
    public static final double kSlowSpeedScale = 0.35;

    /** Trigger axis threshold for activating boost/slow modes. */
    public static final double kTriggerThreshold = 0.25;
  }

  public static final class DriveConstants {
    private DriveConstants() {}

    // ─── CAN Device IDs (CTRE TalonFX) ───────────────────────────────────────
    public static final int kLeftLeaderMotorId = 1;
    public static final int kLeftFollowerMotorId = 2;
    public static final int kRightLeaderMotorId = 3;
    public static final int kRightFollowerMotorId = 4;

    // ─── Physical Drivetrain Geometry & Gearing ──────────────────────────────
    /** Track width between left and right wheel centers in meters (22 inches). */
    public static final double kTrackWidthMeters = Units.inchesToMeters(22.0);

    /** Wheel radius in meters (6-inch diameter wheels -> 3-inch radius). */
    public static final double kWheelRadiusMeters = Units.inchesToMeters(3.0);

    /** Wheel circumference in meters. */
    public static final double kWheelCircumferenceMeters = 2.0 * Math.PI * kWheelRadiusMeters;

    /** Gearbox reduction (motor rotor rotations per 1 wheel mechanism rotation). */
    public static final double kGearRatio = 8.45;

    /** Maximum linear speed of the robot in meters per second. */
    public static final double kMaxLinearSpeedMetersPerSecond = 4.5;

    /** Maximum angular speed of the robot in radians per second. */
    public static final double kMaxAngularSpeedRadiansPerSecond = 2.0 * Math.PI;

    /** Robot mass in kilograms (used for physics simulation). */
    public static final double kRobotMassKg = 50.0;

    /** Robot moment of inertia in kg * m^2 (used for physics simulation). */
    public static final double kRobotMomentOfInertiaKgMetersSquared = 5.0;

    // ─── Electrical & Current Limit Configuration ────────────────────────────
    public static final double kMaxVoltage = 12.0;
    public static final double kStatorCurrentLimitAmps = 80.0;
    public static final double kSupplyCurrentLimitAmps = 40.0;
    public static final double kSupplyCurrentLowerLimitAmps = 30.0;
    public static final double kSupplyCurrentLowerTimeSeconds = 1.0;

    public static final double kOpenLoopRampPeriodSeconds = 0.10;
    public static final double kClosedLoopRampPeriodSeconds = 0.05;
    public static final int kConfigMaxRetries = 5;

    // ─── Closed-Loop Velocity Gains (Slot 0, Volts per wheel rotation/sec) ───
    public static final double kVelocityP = 0.25;
    public static final double kVelocityI = 0.0;
    public static final double kVelocityD = 0.0;
    public static final double kVelocityS = 0.15;
    public static final double kVelocityV = 1.15;
    public static final double kVelocityA = 0.05;
  }
}
