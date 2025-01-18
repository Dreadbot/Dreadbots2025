package frc.robot.subsystems.climb;

import org.littletonrobotics.junction.AutoLog;

import frc.robot.subsystems.endEffector.EndEffectorIO.EndEffectorIOInputs;

public interface ClimbIO {
    
    @AutoLog
    public static class ClimbIOInputs {

        public boolean extended = false;
        
    }

 public default void updateInputs(ClimbIOInputs inputs) {};

 public default void setEnabled(boolean setExtended) {};

}
