package jmetal.algorithm.multiobjective.nsgaiii;

import jmetal.core.algorithm.impl.GeneticAlgorithmWrapper;
import jmetal.core.solution.Solution;

public class NSGAIIIWrapper<S extends Solution<?>> extends GeneticAlgorithmWrapper<S>
{
    public NSGAIIIWrapper(NSGAIII<S> algorithm)
    {
        super(algorithm);
    }
}