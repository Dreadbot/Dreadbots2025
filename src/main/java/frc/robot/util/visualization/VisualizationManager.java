package frc.robot.util.visualization;

import static edu.wpi.first.units.Units.Meters;

import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorConstants;

public class VisualizationManager extends SubsystemBase {

    private final Mechanism2d coralMechanism;
    private final MechanismLigament2d elevatorLigament;
    private final MechanismLigament2d wristLigament;

    private final Mechanism2d slapdownIntakeMechanism;
    private final MechanismLigament2d slapdownIntakeLigament;


    private final DoubleSupplier elevatorHeight;
    private final DoubleSupplier wristAngle;
    private final DoubleSupplier slapdownAngle;

    
    public VisualizationManager(DoubleSupplier elevatorHeight, DoubleSupplier wristAngle, DoubleSupplier slapdownAngle) {
        this.elevatorHeight = elevatorHeight;
        this.wristAngle = wristAngle;
        this.slapdownAngle = slapdownAngle;

        this.coralMechanism = new Mechanism2d(Units.inchesToMeters(32), 2); // extra space for wrist to rotate
        this.elevatorLigament = this.coralMechanism
            .getRoot("ElevatorBase", Units.inchesToMeters(30.0 - 8.0), 0)
            .append(new MechanismLigament2d("Elevator", ElevatorConstants.MIN_HEIGHT + Units.inchesToMeters(40), 90, 10, new Color8Bit(Color.kGray)));
        this.wristLigament = this.elevatorLigament
            .append(new MechanismLigament2d("Wrist", Units.inchesToMeters(9.25), 90, 12, new Color8Bit(Color.kMaroon))); // start horizontal to elevator 
        
        this.slapdownIntakeMechanism = new Mechanism2d(Units.inchesToMeters(30), 1);
        this.slapdownIntakeLigament = this.slapdownIntakeMechanism
            .getRoot("SlapdownBase", Units.inchesToMeters(4.25), Units.inchesToMeters(7.249))
            .append(new MechanismLigament2d("Slapdown", Units.inchesToMeters(16), 90));
    }


    @Override
    public void periodic() {
        elevatorLigament.setLength(elevatorHeight.getAsDouble());
        wristLigament.setAngle(Rotation2d.fromDegrees(wristAngle.getAsDouble() - 90));
        slapdownIntakeLigament.setAngle(Rotation2d.fromDegrees(180 - slapdownAngle.getAsDouble()));
        SmartDashboard.putData("Mechanisms/Coral", coralMechanism);
        SmartDashboard.putData("Mechanisms/Algae", slapdownIntakeMechanism);

    }
}
