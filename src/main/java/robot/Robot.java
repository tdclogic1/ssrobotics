package robot;

import robot.interfaces.*;
import robot.states.RobotState;

public class Robot {
    private final IRobotController controller;
    private final IKinematics kinematics;
    private final ISafetySystem safetySystem;
    private final ISensorSystem sensorSystem;
    private final IMotionPlanner motionPlanner;

    public Robot(IRobotController controller,
                IKinematics kinematics,
                ISafetySystem safetySystem,
                ISensorSystem sensorSystem,
                IMotionPlanner motionPlanner) {
        this.controller = controller;
        this.kinematics = kinematics;
        this.safetySystem = safetySystem;
        this.sensorSystem = sensorSystem;
        this.motionPlanner = motionPlanner;
    }

    public boolean initialize() {
        return controller.initialize() &&
               safetySystem.initialize() &&
               sensorSystem.initialize();
    }

    public RobotState getCurrentState() {
        return controller.getCurrentState();
    }

    public void emergencyStop() {
        safetySystem.triggerEmergencyStop();
        controller.emergencyStop();
    }
}


