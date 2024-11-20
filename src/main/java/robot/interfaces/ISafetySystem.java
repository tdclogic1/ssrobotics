package robot.interfaces;

public interface IKinematics {
    double[][] forwardKinematics(double[] jointAngles);
    double[] inverseKinematics(double[][] targetPose);
    boolean validateJointAngles(double[] angles);
}

// File: src/main/java/robot/interfaces/ISafetySystem.java
package src.main.java.robot.interfaces;

import java.util.List;

public interface ISafetySystem {
    boolean initialize();
    boolean checkMotionSafety(List<List<Double>> plannedPath);
    boolean isPathWithinLimits(List<List<Double>> path);
    void triggerEmergencyStop();
}