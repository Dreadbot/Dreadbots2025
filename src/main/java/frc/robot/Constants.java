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

import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.Unit;
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
    public static final double ELEVATOR_MASS = Units.lbsToKilograms(15);
    public static final double DRIVING_DRUM_RADIUS = Units.inchesToMeters(1.44); 
    public static final double GEARING = 5;
    public static final double MIN_HEIGHT = Units.inchesToMeters(0);
    public static final double MAX_HEIGHT = Units.inchesToMeters(70);
    public static final double STARTING_HEIGHT = MIN_HEIGHT;
  }

  public static class SlapdownAlgaeConstants {
    public static final double INTAKE_VOLTAGE = -7.0;
    public static final double OUTAKE_VOLTAGE = 5.0;
    public static final int SLAPDOWNALGAE_DUTY_CYCLE_ENCODER = 0;
    // public static final String SLAPDOWNALGAE_ENCODER_OFFSET = null;
    // public static final String SLAPDOWNALGAE_ENCODER_SCALE = null;
    // public static final String SLAPDOWNALGAE_IN_OUT_TAKE_MOTOR = null;
    // public static final String SLAPDOWNALGAE_PIVOT_MOTOR = null;
    // public static final String PIVOT_SLAPDOWNALGAE_MOTOR = null;
    // public static final String INTAKE_SLAPDOWNALGAE_MOTOR = null;

    /* 
     * Taken from onshape in form m^2kg, 
     */
    public static final double SIM_INTAKE_MOI = 0.00011264;
    public static final double SIM_PIVOT_MOI = 0.15180934;

  }

  public static class WristConstants {
    //all filler values
    public static final double WRIST_ZERO = 0.0;
    public static final int TOP_LEFT_LIMIT_SWITCH_ID = 0;
    public static final int BOTTOM_LEFT_LIMIT_SWITCH_ID = 0;
    public static final int TOP_RIGHT_LIMIT_SWITCH_ID = 0;
    public static final int BOTTOM_RIGHT_LIMIT_SWITCH_ID = 0;
    public static final int WRIST_DUTY_CYCLE_ENCODER = 1;
    public static final int WRIST_ENCODER_OFFSET = 1;
    public static final int WRIST_ENCODER_SCALE = 1;
    public static final double WRIST_MAX_ANGLE = 1.0;
    //Start Angle in Degrees
    public static final double CORAL_POSITION_PICKUP = 35.0;
    public static final double CORAL_POSITION_L1 = 0.0;
    public static final double CORAL_POSITION_L2 = -55.0;
    public static final double CORAL_POSITION_L3 = -55.0;
    public static final double CORAL_POSITION_L4 = -40.0;
    //End Angle in Degrees
  }
}