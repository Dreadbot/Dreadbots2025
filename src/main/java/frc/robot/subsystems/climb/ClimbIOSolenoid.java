package frc.robot.subsystems.climb;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import frc.robot.subsystems.wrist.WristIO.WristIOInputs;

public class ClimbIOSolenoid implements ClimbIO {
    private Solenoid solenoidClimb;
    private Solenoid solenoidLock;
    private boolean extended = false;

    public ClimbIOSolenoid() {
        this.solenoidClimb = new Solenoid(0, null, 1); 
        this.solenoidLock = new Solenoid(0, null, 2); 
    }

    
    @Override
    public void updateInputs(ClimbIOInputs inputs) {
        inputs.extendedClimb = solenoidClimb.get();
        inputs.extendedLock = solenoidLock.get();
    };

    @Override
    public void setClimbEnabled(boolean setExtended) {
        solenoidClimb.set(setExtended);
    };

    @Override
    public void setLockEnabled(boolean setExtended) {
        solenoidLock.set(setExtended);
    };
}
