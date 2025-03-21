package frc.robot.subsystems.elevator;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import frc.robot.Constants.ElevatorConstants;

public class ElevatorIOSim implements ElevatorIO {
    private final ElevatorSim elevatorSim;
    private double volts;
    boolean topLimitSwitch = true;
    boolean bottomLimitSwitch = true;
    
    public ElevatorIOSim() {
        this.elevatorSim = new ElevatorSim(DCMotor.getNeoVortex(1), 
            ElevatorConstants.GEARING,
            ElevatorConstants.ELEVATOR_MASS, 
            ElevatorConstants.DRIVING_DRUM_RADIUS,
            ElevatorConstants.MIN_HEIGHT,
            ElevatorConstants.MAX_HEIGHT,
            true,
            ElevatorConstants.STARTING_HEIGHT
        );
        this.volts = 0.0;
    }

    @Override
    public void updateInputs(ElevatorIOInputs inputs) {
        elevatorSim.update(0.02);
        inputs.positionMeters = elevatorSim.getPositionMeters();
        inputs.velocityMPS = elevatorSim.getVelocityMetersPerSecond();
        inputs.currentAmps = elevatorSim.getCurrentDrawAmps();
        inputs.voltage = volts; 
    } 

    @Override
    public void runVoltage(double volts) {
        elevatorSim.setInputVoltage(volts);
        this.volts = volts;
    }

    @Override
    public void setMinPosition() {
        elevatorSim.setState(ElevatorConstants.END_EFFECTOR_MIN_HEIGHT, 0);
    }

    @Override
    public boolean getBottomLimitSwitch(){
        return !MathUtil.isNear(ElevatorConstants.MIN_HEIGHT, elevatorSim.getPositionMeters(), 0.01); //1 cm of tolerance.
    }

}