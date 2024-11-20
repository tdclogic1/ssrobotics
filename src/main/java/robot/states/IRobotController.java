package src.main.java.robot.states;

public enum RobotState {
    IDLE,
    MOVING,
    PAUSED,
    ERROR,
    E_STOP
}

// File: src/main/java/robot/interfaces/IRobotController.java
package robot.interfaces;

import robot.states.RobotState;

public interface IRobotController {
    boolean initialize();
    boolean moveToPosition(double[] position);
    boolean rotateToOrientation(double[] orientation);
    RobotState getCurrentState();
    boolean emergencyStop();
}


}


