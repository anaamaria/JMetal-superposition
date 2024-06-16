package jmetal.algorithm.examples.superposition.algorithms;

import jmetal.algorithm.examples.superposition.AbstractSuperPositionGA;
import jmetal.algorithm.examples.superposition.AbstractSuperPositionPSO;
import jmetal.core.algorithm.impl.GeneticAlgorithmWrapper;
import jmetal.core.algorithm.impl.ParticleSwarmWrapper;
import jmetal.core.solution.Solution;

import java.util.List;

public class SuperPositionSMPSO <S extends Solution<?>> extends AbstractSuperPositionPSO<S, List<S>>
{
    public SuperPositionSMPSO(ParticleSwarmWrapper<S> wrapper) {
        super(wrapper);
    }

    @Override
    public String name()
    {
        return "SuperPositionSMPSO";
    }

    @Override
    public String description()
    {
        return "SuperPosition Speed-constrained Multi-objective PSO";
    }

    @Override
    public List<S> result() { return algorithmWrapper.result(); } //aici a trebuit sa adaug metoda de result in ParticleSwarmWrapper.java
}
