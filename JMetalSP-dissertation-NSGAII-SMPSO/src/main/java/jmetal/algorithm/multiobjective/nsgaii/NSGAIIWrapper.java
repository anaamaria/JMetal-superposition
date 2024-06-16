package jmetal.algorithm.multiobjective.nsgaii;

import jmetal.core.algorithm.impl.GeneticAlgorithmWrapper;
import jmetal.core.solution.Solution;

public class NSGAIIWrapper<S extends Solution<?>> extends GeneticAlgorithmWrapper<S>
{
    public NSGAIIWrapper(NSGAII<S> algorithm)
    {
        super(algorithm);
    }
}
