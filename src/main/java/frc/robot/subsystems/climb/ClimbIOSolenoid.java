package frc.robot.subsystems.climb;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;

public class ClimbIOSolenoid implements ClimbIO {
    private Solenoid clawSolenoid;
    private Solenoid lockSolenoid;
    private DoubleSolenoid extendSolenoid;


    public ClimbIOSolenoid() {
        this.clawSolenoid = new Solenoid(PneumaticsModuleType.REVPH, 0);
        this.lockSolenoid = new Solenoid(PneumaticsModuleType.REVPH, 1);
        this.extendSolenoid = new DoubleSolenoid(PneumaticsModuleType.REVPH, 2, 3);
    }

    @Override
    public void updateInputs(ClimbIOInputs inputs) {
        inputs.extendedClaw = clawSolenoid.get();
        inputs.extendedClimb = extendSolenoid.get() == DoubleSolenoid.Value.kForward ? true : false; // Inline if statement: if (extendedSolenoid.get() == DoubleSolenoid.Value.kForward) { true } else { false }
        inputs.extendedLock = lockSolenoid.get();
    }

    @Override
    public void setClawEnabled(boolean setExtended) {
        clawSolenoid.set(!setExtended);
    }

    @Override
    public void setLockEnabled(boolean setExtended) {
        lockSolenoid.set(!setExtended);
    }

    @Override
    public void setClimbState(DoubleSolenoid.Value value) {
        extendSolenoid.set(value);
    }
}
