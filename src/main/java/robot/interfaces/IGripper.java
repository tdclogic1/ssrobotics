package robot.interfaces;

import java.util.List;
import java.util.Map;

public interface IMotionPlanner {
    List<List<Double>> planPath(List<Double> start, List<Double> goal);
    List<Map<String, Object>> generateTrajectory(List<List<Double>> path, double maxVelocity);
    boolean validatePath(List<List<Double>> path);
}