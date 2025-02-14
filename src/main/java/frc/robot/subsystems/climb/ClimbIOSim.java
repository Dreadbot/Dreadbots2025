package frc.robot.subsystems.climb;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.simulation.SolenoidSim;

public class ClimbIOSim implements ClimbIO {
    
    private final SolenoidSim climbingSolenoidSim;
    private final SolenoidSim lockingSolenoidSim;
    private boolean extended = false;
      
    public ClimbIOSim() {
        this.climbingSolenoidSim = new SolenoidSim(1, PneumaticsModuleType.REVPH, 1); 
        this.lockingSolenoidSim = new SolenoidSim(1, PneumaticsModuleType.REVPH, 2); 

    }

    @Override
    public void updateInputs(ClimbIOInputs inputs) {
        inputs.extendedClimb = climbingSolenoidSim.getOutput();
        inputs.extendedLock = lockingSolenoidSim.getOutput();
    };

    @Override
    public void setClimbEnabled(boolean setExtended) {
        climbingSolenoidSim.setOutput(setExtended);
    };

    @Override
    public void setLockEnabled(boolean setExtended) {
        lockingSolenoidSim.setOutput(setExtended);
    };
}

