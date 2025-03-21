// Copyright 2021-2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot.subsystems.drive;

import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.RobotConfig;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;

public class DriveConstants {
  public static final double maxSpeedMetersPerSec = 4.0;
  public static final double odometryFrequency = 100.0; // Hz
  public static final double trackWidth = Units.inchesToMeters(25.0);
  public static final double wheelBase = Units.inchesToMeters(25.0);
  public static final double driveBaseRadius = Math.hypot(trackWidth / 2.0, wheelBase / 2.0);
  public static final Translation2d[] moduleTranslations =
      new Translation2d[] {
        new Translation2d(trackWidth / 2.0, wheelBase / 2.0),
        new Translation2d(trackWidth / 2.0, -wheelBase / 2.0),
        new Translation2d(-trackWidth / 2.0, wheelBase / 2.0),
        new Translation2d(-trackWidth / 2.0, -wheelBase / 2.0)
      };

  // Zeroed rotation values for each module, see setup instructions
  public static final Rotation2d frontLeftZeroRotation = new Rotation2d(0.943 + Math.PI);
  public static final Rotation2d frontRightZeroRotation = new Rotation2d(-2.461 + Math.PI);
  public static final Rotation2d backLeftZeroRotation = new Rotation2d(3.092 + Math.PI);
  public static final Rotation2d backRightZeroRotation = new Rotation2d(2.155);

  // Device CAN IDs
  public static final int pigeonCanId = 20; //not yet determined/ not needed;

  public static final int frontLeftDriveCanId = 1;
  public static final int backLeftDriveCanId = 7;
  public static final int frontRightDriveCanId = 3;
  public static final int backRightDriveCanId = 5;

  public static final int frontLeftTurnCanId = 2;
  public static final int backLeftTurnCanId = 8;
  public static final int frontRightTurnCanId = 4;
  public static final int backRightTurnCanId = 6;

  public static final int frontLeftCanCoderId = 9;
  public static final int frontRightCanCoderId = 10;
  public static final int backLeftCanCoderId = 12;
  public static final int backRightCanCoderId = 11;


  // Drive motor configuration
  public static final int driveMotorCurrentLimit = 50;
  public static final double wheelRadiusMeters = Units.inchesToMeters(1.88); // 1.88
  public static final double driveMotorReduction =
     (50.0 / 14.0) * (17.0 / 27.0) * (45.0 / 15.0); // SDS L2
  public static final DCMotor driveGearbox = DCMotor.getNeoVortex(1);

  // Drive encoder configuration
  public static final double driveEncoderPositionFactor =
      2 * Math.PI / driveMotorReduction; // Rotor Rotations -> Wheel Radians
  public static final double driveEncoderVelocityFactor =
      (2 * Math.PI) / 60.0 / driveMotorReduction; // Rotor RPM -> Wheel Rad/Sec

  // Drive PID configuration
  public static final double driveKp = 0.02;
  public static final double driveKd = 0.0;
  public static final double driveKs = 0.18703;
  public static final double driveKv = 0.11280;
  public static final double driveKt = driveMotorReduction / DCMotor.getNeoVortex(1).KtNMPerAmp;
  public static final double driveSimP = 0.5;
  public static final double driveSimD = 0.0;
  public static final double driveSimKs = 0.01191;
  public static final double driveSimKv = 0.11202;

  // Turn motor configuration
  public static final boolean turnInverted = true;
  public static final int turnMotorCurrentLimit = 35;
  public static final double turnMotorReduction = (150.0 / 7.0);
  public static final DCMotor turnGearbox = DCMotor.getNeoVortex(1);

  // Turn encoder configuration
  public static final boolean turnEncoderInverted = false;
  public static final double turnEncoderPositionFactor = 2 * Math.PI / turnMotorReduction; // Rotations -> Radians
  public static final double turnEncoderVelocityFactor = (2 * Math.PI) / 60.0 / turnMotorReduction; // RPM -> Rad/Sec

  // Turn PID configuration
  public static final double turnKp = 3.0;
  public static final double turnKd = 0.0;
  public static final double turnSimP = 8.0;
  public static final double turnSimD = 0.0;
  public static final double turnPIDMinInput = 0; // Radians
  public static final double turnPIDMaxInput = 2 * Math.PI; // Radians

  //Path following PID configuration
  public static final double xKp = 1.75;
  public static final double xKd = 0.0;
  public static final double yKp = 1.75;
  public static final double yKd = 0.0;
  public static final double rotationKp = 1.75;
  public static final double rotationKd = 0.0;
  // PathPlanner
  public static final double robotMassKg = 74.088;
  public static final double robotMOI = 6.883;
  public static final double wheelCOF = 1.2;
  public static final RobotConfig ppConfig =
      new RobotConfig(
          robotMassKg,
          robotMOI,
          new ModuleConfig(
              wheelRadiusMeters,
              maxSpeedMetersPerSec,
              wheelCOF,
              driveGearbox.withReduction(driveMotorReduction),
              driveMotorCurrentLimit,
              1),
          moduleTranslations);
}
