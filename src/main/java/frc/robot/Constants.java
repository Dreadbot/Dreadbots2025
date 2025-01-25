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

package frc.robot;

import edu.wpi.first.wpilibj.RobotBase;

/**
 * This class defines the runtime mode used by AdvantageKit. The mode is always "real" when running
 * on a roboRIO. Change the value of "simMode" to switch between "sim" (physics sim) and "replay"
 * (log replay from a file).
 */
public final class Constants {
  public static final Mode simMode = Mode.SIM;
  public static final Mode currentMode = RobotBase.isReal() ? Mode.REAL : simMode;

  public static enum Mode {
    /** Running on a real robot. */
    REAL,

    /** Running a physics simulator. */
    SIM,

    /** Replaying from a log file. */
    REPLAY
  }
  public static class EndEffectorConstants {
    public static final double INTAKE_VOLTAGE = -7.0;
    public static final double OUTAKE_VOLTAGE = 5.0;

  }
  public static final class AutoAlignConstants {
    public static final double TRANSLATION_KP = 5.0;
    public static final double TRANSLATION_KD = 0.0;
    public static final double TRANSLATION_VELOCITY = 2.0; // Meters/Sec
    public static final double TRANSLATION_ACCELERATION = 8.0; // Meters/Sec^2
    public static final double TRANSLATION_JERK = 10.0; // Meters/Sec^3
    public static final double ROTATION_KP = 10.0;
    public static final double ROTATION_KD = 0.0;
    public static final double ROTATION_MAX_VELOCITY = 10.0;
    public static final double ROTATION_MAX_ACCELERATION = 20.0;
    public static final double REEF_BRANCH_OFFSET = Units.inchesToMeters(12.938 / 2.0);
    private static Pose2d[] tmp = new Pose2d[6];
    static {
      Translation2d reefCenter =
        new Translation2d(Units.inchesToMeters(176.746), Units.inchesToMeters(158.501));
      Translation2d firstSide =
        new Translation2d(3.229, 4.026);
      for(int i = 0; i < 6; i++) {
        Translation2d side = firstSide.rotateAround(reefCenter, Rotation2d.fromDegrees(60 * (i - 1)));
        tmp[i] = new Pose2d(side, Rotation2d.fromDegrees(60 * (i - 1)));
        
      }
    }
    public static final List<Pose2d> POIs = List.of(tmp);
  }
}
