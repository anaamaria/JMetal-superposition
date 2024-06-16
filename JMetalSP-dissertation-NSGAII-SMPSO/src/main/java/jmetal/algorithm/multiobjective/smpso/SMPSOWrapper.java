package jmetal.algorithm.multiobjective.smpso;

import jmetal.core.algorithm.impl.AbstractParticleSwarmOptimization;
import jmetal.core.algorithm.impl.GeneticAlgorithmWrapper;
import jmetal.core.algorithm.impl.ParticleSwarmWrapper;
import jmetal.core.solution.Solution;

import java.util.List;

public class SMPSOWrapper <S extends Solution<?>> extends ParticleSwarmWrapper<S>
{
    public SMPSOWrapper(AbstractParticleSwarmOptimization<S, List<S>> algorithm) {
        super(algorithm);
    }
}
