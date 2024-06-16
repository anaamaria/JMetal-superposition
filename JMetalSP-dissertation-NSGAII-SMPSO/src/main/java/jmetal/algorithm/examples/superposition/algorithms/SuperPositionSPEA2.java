package jmetal.algorithm.examples.superposition.algorithms;

import jmetal.algorithm.examples.superposition.AbstractSuperPositionGA;
import jmetal.core.algorithm.impl.GeneticAlgorithmWrapper;
import jmetal.core.problem.Problem;
import jmetal.core.solution.Solution;

import java.util.List;

public class SuperPositionSPEA2<S extends Solution<?>> extends AbstractSuperPositionGA<S, List<S>>
{

    public SuperPositionSPEA2(GeneticAlgorithmWrapper<S> wrapper)
    {
        super(wrapper);
    }

    @Override
    public String name()
    {
        return "SuperPositionSPEA2";
    }

    @Override
    public String description()
    {
        return "Super Position Strength Pareto Evolutionary Algorithm 2";
    }

    @Override
    public List<S> result()
    {
        return algorithmWrapper.result();
    }
}
//nimic schimbat aici
