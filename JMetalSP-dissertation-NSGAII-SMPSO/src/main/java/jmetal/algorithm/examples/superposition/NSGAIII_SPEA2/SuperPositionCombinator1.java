package jmetal.algorithm.examples.superposition.NSGAIII_SPEA2;

import jmetal.algorithm.examples.superposition.AbstractSuperPositionGA;
import jmetal.algorithm.examples.superposition.AbstractSuperPositionGAGACombinator;
import jmetal.core.solution.Solution;
import jmetal.core.util.ranking.Ranking;
import jmetal.core.util.ranking.impl.FastNonDominatedSortRanking;


import java.util.ArrayList;
import java.util.List;

public class SuperPositionCombinator1<S extends Solution<?>> extends AbstractSuperPositionGAGACombinator<S> {

    public SuperPositionCombinator1(AbstractSuperPositionGA<S, List<S>> algorithm1, AbstractSuperPositionGA<S, List<S>> algorithm2)
    {
        super(algorithm1, algorithm2);
    }

    @Override
    protected List<S> ApplySuperPosition()
    {
        // 1. COMBINE POPULATIONS
        List<S> combinedPopulation = new ArrayList<>();

        // 2. RANKING AND TAKING THE BEST FRONT - we make this step to take the best 100 individuals from the combined population
        combinedPopulation.addAll(algorithm1.getPopulation());
        combinedPopulation.addAll(algorithm2.getPopulation());

        // 3. GENERATE NEW POPULATION - here we set the 100 best as the new generation
        Ranking<S> fronts = computeRanking(combinedPopulation);

        List<S> newPopulation = Take(fronts, algorithm1.getMaxPopulationSize());

        // 4. SET NEW POPULATION FOR EACH ALGORITHM
        algorithm1.setPopulation(newPopulation);
        algorithm2.setPopulation(newPopulation);

        return newPopulation;
    }
} //nimic schimbat aici