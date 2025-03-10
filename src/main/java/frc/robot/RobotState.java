package frc.robot;

public class RobotState {
    
    public CurrentAction currentAction;
    private static RobotState instance;

    public RobotState() {
        currentAction = CurrentAction.MANUAL_CONTROL;
    }



    public enum CurrentAction {
        MANUAL_CONTROL,
        AUTO_ALIGN
    }
}
