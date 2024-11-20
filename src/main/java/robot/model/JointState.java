package robot.model;

public class JointState {
    private final double position;
    private final double velocity;
    private final double effort;
    private final double temperature;

    public JointState(double position, double velocity, double effort, double temperature) {
        this.position = position;
        this.velocity = velocity;
        this.effort = effort;
        this.temperature = temperature;
    }

    // Getters
    public double getPosition() { return position; }
    public double getVelocity() { return velocity; }
    public double getEffort() { return effort; }
    public double getTemperature() { return temperature; }
}