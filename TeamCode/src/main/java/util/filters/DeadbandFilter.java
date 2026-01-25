package util.filters;

public class DeadbandFilter implements Filter {
    private final double deadband;

    public DeadbandFilter(double deadband) {
        this.deadband = deadband;
    }

    @Override
    public double calculate(double input) {
        if (Math.abs(input) < deadband) {
            return 0.0;
        }
        // Optional: Rescale the output so it starts at 0 from the edge of the deadband
        return (input - (Math.signum(input) * deadband)) / (1.0 - deadband);
    }
}