package frc.robot.subsystems.wrist;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.Constants.WristConstants;


public class WristIOSim implements WristIO {

    private final SingleJointedArmSim wrist;
    private double volts;
    private DigitalInput leftTopSwitch;
    private DigitalInput leftBottomSwitch;
    private DigitalInput rightTopSwitch;
    private DigitalInput rightBottomSwitch;
    

    public WristIOSim() {
        this.wrist = new SingleJointedArmSim(
            DCMotor.getNEO(1), 
            50.0, 
            SingleJointedArmSim.estimateMOI(0.15, Units.lbsToKilograms(10)), 
            0.15, 
            Units.degreesToRadians(-90), 
            Units.degreesToRadians(90), 
            true,
            Units.degreesToRadians(0)
            );

        this.leftTopSwitch = new DigitalInput(WristConstants.TOP_LEFT_LIMIT_SWITCH_ID);
        this.rightTopSwitch = new DigitalInput(WristConstants.TOP_RIGHT_LIMIT_SWITCH_ID);
        this.leftBottomSwitch = new DigitalInput(WristConstants.BOTTOM_LEFT_LIMIT_SWITCH_ID);
        this.rightBottomSwitch = new DigitalInput(WristConstants.BOTTOM_RIGHT_LIMIT_SWITCH_ID);
        volts = 0.0;
    }



    @Override
    public void updateInputs(WristIOInputs inputs) {
        wrist.update(0.02);

        inputs.appliedVolts = 0.0;
       
        inputs.RPS = (wrist.getVelocityRadPerSec());

        inputs.currentAmps = wrist.getCurrentDrawAmps();

        inputs.rotationDegrees = Units.radiansToDegrees(wrist.getAngleRads());

    } 

    @Override
    public void runVoltage(double volts) {
        wrist.setInputVoltage(volts);
        this.volts = volts;
    }

    
    
}