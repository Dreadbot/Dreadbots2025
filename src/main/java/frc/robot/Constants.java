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
  public static class ElevatorConstants {
    // fix numbers later
    public static final double RISE_VOLTAGE = 5.0;
    public static final double DROP_VOLTAGE = -5.0;
    public static final double DCMOTOR_MASS = 2;
    public static final double ELEVATOR_MASS = 7;
    public static final double DRIVING_DRUM_RADIUS = .5;
    public static final double GEARING = 3;
    public static final double MIN_HEIGHT = .5;
    public static final double MAX_HEIGHT = 1;
    public static final double STARTING_HEIGHT = .5;
  }
}
