package jmetal.problem.multiobjective;

import java.util.ArrayList;
import java.util.List;
import jmetal.core.problem.doubleproblem.impl.AbstractDoubleProblem;
import jmetal.core.solution.doublesolution.DoubleSolution;

/** Class representing problem Viennet2 */
@SuppressWarnings("serial")
public class Viennet2 extends AbstractDoubleProblem {

  /** Constructor. Creates a default instance of the Viennet2 problem */
  public Viennet2() {
    int numberOfVariables = 2 ;
    numberOfObjectives(3);
    numberOfConstraints(0);
    name("Viennet2");

    List<Double> lowerLimit = new ArrayList<>(numberOfVariables);
    List<Double> upperLimit = new ArrayList<>(numberOfVariables);

    for (int i = 0; i < numberOfVariables; i++) {
      lowerLimit.add(-4.0);
      upperLimit.add(4.0);
    }

    variableBounds(lowerLimit, upperLimit);
  }

  /** Evaluate() method */
  @Override
  public DoubleSolution evaluate(DoubleSolution solution) {
    int numberOfVariables = numberOfVariables();

    double[] f = new double[solution.objectives().length];
    double[] x = new double[numberOfVariables];

    for (int i = 0; i < numberOfVariables; i++) {
      x[i] = solution.variables().get(i);
    }

    // First function
    f[0] = (x[0] - 2) * (x[0] - 2) / 2.0 + (x[1] + 1) * (x[1] + 1) / 13.0 + 3.0;

    // Second function
    f[1] =
        (x[0] + x[1] - 3.0) * (x[0] + x[1] - 3.0) / 36.0
            + (-x[0] + x[1] + 2.0) * (-x[0] + x[1] + 2.0) / 8.0
            - 17.0;

    // Third function
    f[2] =
        (x[0] + 2 * x[1] - 1) * (x[0] + 2 * x[1] - 1) / 175.0
            + (2 * x[1] - x[0]) * (2 * x[1] - x[0]) / 17.0
            - 13.0;

    for (int i = 0; i < solution.objectives().length; i++) solution.objectives()[i] = f[i];

    return solution;
  }
}