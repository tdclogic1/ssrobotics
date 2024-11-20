package src.main.java.robot.interfaces;

import java.util.Map;

public class ISensorSystem {
        boolean initialize();
    Map<String, Object> getAllSensorData();
    boolean calibrateSensors();
    boolean checkSensorHealth();
}
