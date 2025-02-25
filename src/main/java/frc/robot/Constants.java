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
import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
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
    public static final double INTAKE_VOLTAGE = 3.0;
    public static final double OUTAKE_VOLTAGE = -1.5;
    public static final double CORAL_THRESHOLD = 50.0;

  }
  public static class ElevatorConstants {
    public static final double DCMOTOR_MASS = 2;
    public static final double ELEVATOR_MASS = Units.lbsToKilograms(15);
    public static final double DRIVING_DRUM_RADIUS = Units.inchesToMeters(1.449); // Actual sprocket diameter
    public static final double GEARING = 12;
    public static final double MIN_HEIGHT = Units.inchesToMeters(25); //from ground
    public static final double MAX_HEIGHT = 2.124; //from ground
    public static final double STARTING_HEIGHT = MIN_HEIGHT + Units.inchesToMeters(10); //simulate homing sequence ONLY FOR SIM
    public static final double END_EFFECTOR_MIN_HEIGHT = MIN_HEIGHT;
    public static final double ZEROING_VOLTAGE = -0.5;
    public static final int BOTTOM_LIMIT_SWITCH_ID = 0;
    public static final double ELEVATOR_JOYSTICK_SLEW_VALUE = -0.00346;
    public static final int MOTOR_ID = 13;
  }


  public static class SlapdownAlgaeConstants {
    public static final double INTAKE_VOLTAGE = -5.0;
    public static final double OUTAKE_VOLTAGE = 5.0;
    public static final int SLAPDOWNALGAE_DUTY_CYCLE_ENCODER = 8;
    public static final double ENCODER_OFFSET = 48.125;
    public static final double HOME_ANGLE_DEGREES = 0;
    public static final double OUTTAKE_ANGLE_DEGREES = 3;
    public static final double INTAKE_ANGLE_DEGREES = 60.0;
    public static final double HOLD_ANGLE_DEGREES = 14.0;
    public static final double MAX_ANGLE_DEGREES = 80.0;
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
    public static final double WRIST_ENCODER_OFFSET = 26.9;
    public static final double WRIST_MAX_ANGLE = 360;
    public static final double WRIST_JOYSTICK_SLEW_VALUE = 3;
    public static final double WRIST_EXPECTED_ZERO = 0;
    //Start Angle in Degrees
    public static final double CORAL_POSITION_PICKUP = 35.0;
    public static final double CORAL_POSITION_L1 = 0.0;
    public static final double CORAL_POSITION_L2 = -55.0;
    public static final double CORAL_POSITION_L3 = -55.0;
    public static final double CORAL_POSITION_L4 = -40.0;
    //End Angle in Degrees

    /* 
    * SAFE ZONE EXPLANATION!!!!!
    * With our robot layout, there is a crossbar on the first stage (we ony have 
    * to care about this when we are going to max height)
    * Because of this, there are a couple of specific situations in which we need to move the wrist FIRST, and then move elevator.
    * Most of the time we can move both at same time
    */ 
    public static final double START_SAFE_ZONE = 0.0;
    public static final double END_SAFE_ZONE = 106;

  }

  public static final class AutoAlignConstants {
    public static final double TRANSLATION_KP = 1.0;
    public static final double TRANSLATION_KD = 0.0;
    public static final double TRANSLATION_VELOCITY = 1.5; // Meters/Sec
    public static final double TRANSLATION_ACCELERATION = 1.5; // Meters/Sec^2
    public static final double TRANSLATION_JERK = 1.0; // Meters/Sec^3
    public static final double ROTATION_KP = 2.0;
    public static final double ROTATION_KD = 0.0;
    public static final double ROTATION_MAX_VELOCITY = 5.0;
    public static final double ROTATION_MAX_ACCELERATION = 10.0;
    public static final double REEF_BRANCH_OFFSET = Units.inchesToMeters(12.938 / 2.0);
  }
}
