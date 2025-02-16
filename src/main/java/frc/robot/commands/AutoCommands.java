package frc.robot.commands;

import choreo.auto.AutoFactory;
import choreo.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.Superstructure.SuperstructureState;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.endEffector.EndEffector;
import frc.robot.subsystems.wrist.Wrist;

public class AutoCommands {
    private AutoFactory factory;
    private Drive drive;
    private Superstructure superstructure;
    private EndEffector endEffector;

    public AutoCommands(Drive drive, Superstructure superstructure, EndEffector endEffector) {
        this.drive = drive;
        this.superstructure = superstructure;
        this.endEffector = endEffector;
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

    public Command MidBargeC2B2ClosePickup(){
        return Commands.sequence(
            factory.resetOdometry("MidBarge-C2B2-ClosePickup",0),
            factory.trajectoryCmd("MidBarge-C2B2-ClosePickup",0),
            superstructure.requestSuperstructureState(SuperstructureState.L4).andThen(Commands.waitUntil(superstructure::isFinished)),
            endEffector.outtake().withTimeout(.5),
            superstructure.requestSuperstructureState(SuperstructureState.PICKUP).andThen(Commands.waitUntil(superstructure::isFinished)),
            factory.trajectoryCmd("MidBarge-C2B2-ClosePickup",1),
            endEffector.intake().until(endEffector::hasGamePiece),
            factory.trajectoryCmd("MidBarge-C2B2-ClosePickup",2),
            superstructure.requestSuperstructureState(SuperstructureState.L4).andThen(Commands.waitUntil(superstructure::isFinished)),
            endEffector.outtake().withTimeout(.5),
            superstructure.requestSuperstructureState(SuperstructureState.STOW)
        );
    }
}
