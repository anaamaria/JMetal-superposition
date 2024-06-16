package jmetal.algorithm.examples.superposition.algorithms;

import jmetal.algorithm.examples.superposition.AbstractSuperPositionGA;
import jmetal.core.algorithm.impl.GeneticAlgorithmWrapper;
import jmetal.core.solution.Solution;

import java.util.List;

public class SuperPositionNSGAIII<S extends Solution<?>> extends AbstractSuperPositionGA<S, List<S>>
{
    public SuperPositionNSGAIII(GeneticAlgorithmWrapper<S> wrapper)
    {
        super(wrapper);
    }

    @Override
    public String name()
    {
        return "SuperPositionNSGAIII";
    }

    @Override
    public String description()
    {
        return "SuperPosition Non-dominated Sorting Genetic Algorithm III";
    }

    @Override
    public List<S> result()
    {
        return algorithmWrapper.result();
    }
}
//nimic schimbat aici