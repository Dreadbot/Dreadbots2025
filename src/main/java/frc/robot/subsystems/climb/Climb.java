package frc.robot.subsystems.climb;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.climb.ClimbIO.ClimbIOInputs;

public class Climb extends SubsystemBase {
    
    private ClimbIO io;
    private Climb climb;
    private boolean extended = false;
    private Solenoid solenoidClimb = new Solenoid(PneumaticsModuleType.REVPH, 0);
    private Solenoid solenoidLock = new Solenoid(PneumaticsModuleType.REVPH, 0);

    public Climb(ClimbIO io) { 
        this.io = io;
        this.solenoidClimb = new Solenoid(0, null, 0); 
        this.solenoidLock = new Solenoid(0, null, 0); 
    }

    @Override
    public void periodic() {}
    
    public void isExtended(boolean extended) {
        this.extended = true;
    }

    public void isRetracted(boolean extended) {
        this.extended = false;
    }

    public Command swapStatusClimb() {
         return runOnce(() -> {
         solenoidClimb.set(!solenoidClimb.get());
         });
        
    }

    public Command swapStatusLock() {
        return runOnce(() -> {
        solenoidLock.set(!solenoidLock.get());
        });
       
   }
    
}
