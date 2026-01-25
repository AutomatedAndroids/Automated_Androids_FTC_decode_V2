package util;

public class OldDriverFilter2 {
    private double lastOutput = 0;
    private final double deadband;
    private final double minOutput;
    private final double maxSpeed;
    private final double alpha;
    private final double maxAccel;
    private final double maxDecel;

    public OldDriverFilter2(double deadband, double minOutput, double maxSpeed,
                            double alpha, double maxAccel, double maxDecel) {
        this.deadband = deadband;
        this.minOutput = minOutput;
        this.maxSpeed = maxSpeed;
        this.alpha = alpha;
        this.maxAccel = maxAccel;
        this.maxDecel = maxDecel;
    }

    public double calculate(double input) {
        // 1. Apply Deadband
        if (Math.abs(input) < deadband) input = 0;

        // 2. Scale to max speed
        double target = input * maxSpeed;

        // 3. Simple Slew Rate (Acceleration Limiting)
        // Note: In a real robot, you'd multiply maxAccel by deltaTime
        double error = target - lastOutput;
        if (error > maxAccel) target = lastOutput + maxAccel;
        else if (error < maxDecel) target = lastOutput + maxDecel;

        // 4. Alpha Filter (Low Pass / Smoothing)
        double output = (alpha * target) + (1.0 - alpha) * lastOutput;

        // 5. Min output check
        if (Math.abs(output) < minOutput && Math.abs(output) > 1e-5) {
            output = Math.signum(output) * minOutput;
        }

        lastOutput = output;
        return output;
    }
}