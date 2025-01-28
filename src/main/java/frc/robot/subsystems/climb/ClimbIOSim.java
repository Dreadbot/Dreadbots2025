package frc.robot.subsystems.climb;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;

public class ClimbIOSim implements ClimbIO {
    
    private Solenoid solenoid = new Solenoid(PneumaticsModuleType.REVPH, 0);
    private boolean extended = false;
      
    public ClimbIOSim() {
        this.solenoid = new Solenoid(0, null, 0); 

    }

    
    // @Override
    // public void swapStatus() {
    //     if (solenoid.isExtended) {
    //         solenoid.isRetracted;
    //     }

    //     else if (solenoid.isRetracted) {
    //         solenoid.isExtended;
    //     }
    // }
}

