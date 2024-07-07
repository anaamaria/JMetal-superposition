package jmetal.algorithm.examples.multiobjective;

import java.util.ArrayList;
import java.util.List;

import jmetal.core.solution.doublesolution.DoubleSolution;

public class ParetoFrontGenerator {

    public double[][] generateTrueParetoFront(List<DoubleSolution> solutions) {
        List<DoubleSolution> paretoFront = new ArrayList<>();

        for (DoubleSolution candidate : solutions) {
            boolean dominated = false;
            List<DoubleSolution> toRemove = new ArrayList<>();

            for (DoubleSolution member : paretoFront) {
                if (dominates(member, candidate)) {
                    dominated = true;
                    break;
                } else if (dominates(candidate, member)) {
                    toRemove.add(member);
                }
            }

            if (!dominated) {
                paretoFront.removeAll(toRemove);
                paretoFront.add(candidate);
            }
        }

        return convertSolutionsToObjectivesArray(paretoFront);
    }

    public double[][] convertSolutionsToObjectivesArray(List<DoubleSolution> solutions) {
        double[][] objectivesArray = new double[solutions.size()][];
        for (int i = 0; i < solutions.size(); i++) {
            objectivesArray[i] = solutions.get(i).objectives();
        }
        return objectivesArray;
    }

    private boolean dominates(DoubleSolution solution1, DoubleSolution solution2) {
        double[] point1 = solution1.objectives();
        double[] point2 = solution2.objectives();

        int i;
        int betterInAnyObjective;

        betterInAnyObjective = 0;
        for (i = 0; i < solution1.objectives().length && point1[i] >= point2[i]; i++) {
            if (point1[i] > point2[i]) {
                betterInAnyObjective = 1;
            }
        }

        return ((i >= solution1.objectives().length) && (betterInAnyObjective > 0));
    }
}