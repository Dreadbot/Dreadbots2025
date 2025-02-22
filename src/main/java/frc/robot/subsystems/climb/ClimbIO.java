package frc.robot.subsystems.climb;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.wpilibj.DoubleSolenoid;

public interface ClimbIO {
    
    @AutoLog
    public static class ClimbIOInputs {

        public boolean extendedClimb = false;
        public boolean extendedLock = false;
        public boolean extendedClaw = false;
    }
    
    public default void updateInputs(ClimbIOInputs inputs) {};

    public default void setClawEnabled(boolean setExtended) {};
    public default void setLockEnabled(boolean setExtended) {};
    public default void setClimbState(DoubleSolenoid.Value value) {};


    }
