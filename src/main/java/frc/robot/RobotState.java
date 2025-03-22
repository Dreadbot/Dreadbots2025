package frc.robot;

import org.littletonrobotics.junction.Logger;

public class RobotState {
    public CurrentAction currentAction;
    private static RobotState instance;

    private RobotState() {
        currentAction = CurrentAction.MANUAL_CONTROL;
    }

    public static RobotState getInstance() {
        if (instance == null) {
            instance = new RobotState();
        }
        return instance;
    }

    public void setRobotAction(CurrentAction action)  {
        Logger.recordOutput("RobotState/CurrentAction", action);
        this.currentAction = action;
    }

    public CurrentAction getCurrentAction() {
        return this.currentAction;
    }

    public enum CurrentAction {
        MANUAL_CONTROL,
        AUTO_ALIGN
    }
}
