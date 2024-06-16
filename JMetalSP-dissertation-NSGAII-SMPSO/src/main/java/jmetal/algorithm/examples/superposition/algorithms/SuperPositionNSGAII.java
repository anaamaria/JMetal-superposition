package jmetal.algorithm.examples.superposition.algorithms;

import jmetal.algorithm.examples.superposition.AbstractSuperPositionGA;
import jmetal.core.algorithm.impl.GeneticAlgorithmWrapper;
import jmetal.core.solution.Solution;

import java.util.List;

public class SuperPositionNSGAII<S extends Solution<?>> extends AbstractSuperPositionGA<S, List<S>>
{
    public SuperPositionNSGAII(GeneticAlgorithmWrapper<S> wrapper)
    {
        super(wrapper);
    }

    @Override
    public String name()
    {
        return "SuperPositionNSGAII";
    }

    @Override
    public String description()
    {
        return "SuperPosition Non-dominated Sorting Genetic Algorithm II";
    }

    @Override
    public List<S> result()
    {
        return algorithmWrapper.result();
    }
}
