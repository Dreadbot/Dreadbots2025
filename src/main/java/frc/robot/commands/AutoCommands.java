package frc.robot.commands;

import choreo.auto.AutoFactory;
import choreo.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.wrist.Wrist;

public class AutoCommands {
    private AutoFactory factory;
    private Drive drive;
    private Superstructure superstructure;

    public AutoCommands(Drive drive, Superstructure superstructure) {
        this.drive = drive;
        this.superstructure = superstructure;

        this.factory = new AutoFactory(
            drive::getPose,
            drive::setPose,
            drive::followTrajectory,
            true,
            drive,
            drive::logTrajectory
        );
    }

    public Command choreoTest() {
        return Commands.sequence(
            factory.resetOdometry("Processor-A2-FarPickup"),
            factory.trajectoryCmd("Processor-A2-FarPickup")
        );
    }   
}
