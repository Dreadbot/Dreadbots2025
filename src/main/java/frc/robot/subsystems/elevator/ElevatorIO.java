package frc.robot.subsystems.elevator;

import org.littletonrobotics.junction.AutoLog;


public interface ElevatorIO {

    
@AutoLog
public class ElevatorIOInputs {

    public double positionMeters = 0.0;
       
    public double velocityMPS = 0.0;
   
    public double currentAmps = 0.0;
    
    public double voltage = 0.0;
    
}


 public default void updateInputs(ElevatorIOInputs inputs) {}

 public default void runVoltage(double volts) {};

 public default void setMinPosition() {};

}