package jmetal.algorithm.examples.superposition.NSGAII_SMPSO;

import jmetal.algorithm.examples.superposition.AbstractSuperPositionGA;
import jmetal.algorithm.examples.superposition.AbstractSuperPositionGAPSOCombinator;
import jmetal.algorithm.examples.superposition.AbstractSuperPositionPSO;
import jmetal.core.solution.Solution;
import jmetal.core.util.ranking.Ranking;

import java.util.ArrayList;
import java.util.List;

public class SuperPositionCombinator1<S extends Solution<?>> extends AbstractSuperPositionGAPSOCombinator<S> {
    public SuperPositionCombinator1(AbstractSuperPositionGA<S, List<S>> alg1, AbstractSuperPositionPSO<S, List<S>> alg2) {
        super(alg1, alg2);
    }

    @Override
    protected List<S> ApplySuperPosition()
    {
        // 1. COMBINE POPULATIONS
        List<S> combinedPopulation = new ArrayList<>();

        // 2. RANKING AND TAKING THE BEST FRONT - we make this step to take the best 100 individuals from the combined population
        combinedPopulation.addAll(algorithm1.getPopulation());
        combinedPopulation.addAll(algorithm2.getSwarm()); // aici am schimbat din getPopulation in getSwarm

        // 3. GENERATE NEW POPULATION - here we set the 100 best as the new generation
        Ranking<S> fronts = computeRanking(combinedPopulation);

        List<S> newPopulation = take(fronts, algorithm1.getMaxPopulationSize());

        // 4. SET NEW POPULATION FOR EACH ALGORITHM
        algorithm1.setPopulation(newPopulation);
        algorithm2.setSwarm(newPopulation); // aici am schimbat din getPopulation in setSwarm

        return newPopulation;
    }
}
