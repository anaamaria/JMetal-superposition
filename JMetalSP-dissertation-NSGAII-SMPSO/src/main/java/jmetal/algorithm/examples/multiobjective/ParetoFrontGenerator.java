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

        double[][] paretoObjectives = new double[paretoFront.size()][];
        for (int i = 0; i < paretoFront.size(); i++) {
            paretoObjectives[i] = paretoFront.get(i).objectives();
        }

        return paretoObjectives;
    }

    private boolean dominates(DoubleSolution solution1, DoubleSolution solution2) {
        double[] objectives1 = solution1.objectives();
        double[] objectives2 = solution2.objectives();

        boolean betterInOneAspect = false;
        for (int i = 0; i < objectives1.length; i++) {
            if (objectives1[i] < objectives2[i]) {
                betterInOneAspect = true;
            } else if (objectives1[i] > objectives2[i]) {
                return false;
            }
        }

        return betterInOneAspect;
    }
}