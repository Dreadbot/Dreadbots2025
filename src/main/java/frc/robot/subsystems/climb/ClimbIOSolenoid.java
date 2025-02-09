package frc.robot.subsystems.climb;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import frc.robot.subsystems.wrist.WristIO.WristIOInputs;

public class ClimbIOSolenoid implements ClimbIO {
    private Solenoid solenoid = new Solenoid(PneumaticsModuleType.REVPH, 0);
    private boolean extended = false;

    public ClimbIOSolenoid() {
        this.solenoid = new Solenoid(0, null, 0); 

    }

    // @Override
    // public void updateInputs(SolenoidIOInputs inputs) {

       
    // } 
}
