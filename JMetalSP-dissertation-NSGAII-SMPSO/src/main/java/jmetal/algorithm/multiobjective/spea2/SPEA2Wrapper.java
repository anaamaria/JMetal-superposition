package jmetal.algorithm.multiobjective.spea2;

import jmetal.core.algorithm.impl.GeneticAlgorithmWrapper;
import jmetal.core.solution.Solution;

public class SPEA2Wrapper<S extends Solution<?>> extends GeneticAlgorithmWrapper<S>
{
    public SPEA2Wrapper(SPEA2<S> algorithm)
    {
        super(algorithm);
    }
}