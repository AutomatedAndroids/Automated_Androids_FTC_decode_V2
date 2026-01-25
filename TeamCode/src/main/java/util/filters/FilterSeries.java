package util.filters;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

public class FilterSeries implements Filter {
    private final List<Filter> filters = new ArrayList<>();

    public FilterSeries(Filter... filters) {
        this.filters.addAll(Arrays.asList(filters));
    }

    @Override
    public double calculate(double input) {
        double output = input;
        for (Filter filter : filters) {
            output = filter.calculate(output);
        }
        return output;
    }
}