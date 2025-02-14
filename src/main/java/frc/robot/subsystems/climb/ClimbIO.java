package frc.robot.subsystems.climb;

import org.littletonrobotics.junction.AutoLog;

public interface ClimbIO {
    
    @AutoLog
    public static class ClimbIOInputs {

        public boolean extendedClimb = false;
        public boolean extendedLock = false;
        
        }
    
 public default void updateInputs(ClimbIOInputs inputs) {};

 public default void setLockEnabled(boolean setExtended) {};

 public default void setClimbEnabled(boolean setExtended) {};

    }
