package util.filters;

public class ScaleFilter implements Filter {
    private final double scale;

    public ScaleFilter(double scale) {
        this.scale = scale;
    }

    @Override
    public double calculate(double input) {
        return input * scale;
    }
}