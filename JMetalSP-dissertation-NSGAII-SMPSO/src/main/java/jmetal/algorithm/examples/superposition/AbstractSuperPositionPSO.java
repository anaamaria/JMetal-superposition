package jmetal.algorithm.examples.superposition;

import jmetal.algorithm.examples.superposition.events.IAction;
import jmetal.core.algorithm.Algorithm;
import jmetal.core.algorithm.impl.AbstractEvolutionaryAlgorithm;
import jmetal.core.algorithm.impl.GeneticAlgorithmWrapper;
import jmetal.core.algorithm.impl.ParticleSwarmWrapper;

import java.util.List;

public abstract class AbstractSuperPositionPSO<S, R> implements Algorithm<R>
{
    protected ParticleSwarmWrapper<S> algorithmWrapper;

    public volatile boolean isWaiting;

    public IAction onNewGeneration;

    public AbstractSuperPositionPSO(ParticleSwarmWrapper<S> wrapper)
    {
        algorithmWrapper = wrapper;

        isWaiting = false;
    }

    @Override
    public void run()
    {
        createInitialSwarm();
        evaluateSwarm();
        initializeVelocity();
        initializeParticlesMemory() ;
        initializeLeader() ;

        initProgress();

        while (!isStoppingConditionReached()) {
            updateVelocity();
            updatePosition();
            perturbation();
            evaluateSwarm();
            updateLeaders();
            updateParticlesMemory();

            onNewGeneration.invoke();

            updateProgress();

            isWaiting = true;

            while (isWaiting)
            {
                Thread.onSpinWait();
            }
        }
    }

    public void createInitialSwarm() {
        List<S> initialSwarm = algorithmWrapper.createInitialSwarm();
        algorithmWrapper.setSwarm(initialSwarm);
    }

    public void evaluateSwarm() {
        List<S> currentSwarm = algorithmWrapper.getSwarm();
        List<S> evaluatedSwarm =  algorithmWrapper.evaluateSwarm(currentSwarm);
        algorithmWrapper.setSwarm(evaluatedSwarm);
    }

    public void initializeLeader() {
        List<S> currentSwarm = algorithmWrapper.getSwarm();
        algorithmWrapper.initializeLeader(currentSwarm);
    }

    public void initializeParticlesMemory() {
        List<S> currentSwarm = algorithmWrapper.getSwarm();
        algorithmWrapper.initializeParticlesMemory(currentSwarm);
    }

    public void initializeVelocity() {
        List<S> currentSwarm = algorithmWrapper.getSwarm();
        algorithmWrapper.initializeVelocity(currentSwarm);
    }

    public void updateVelocity() {
        List<S> currentSwarm = algorithmWrapper.getSwarm();
        algorithmWrapper.updateVelocity(currentSwarm);
    }

    public void updatePosition() {
        List<S> currentSwarm = algorithmWrapper.getSwarm();
        algorithmWrapper.updatePosition(currentSwarm);
    }

    public void perturbation() {
        List<S> currentSwarm = algorithmWrapper.getSwarm();
        algorithmWrapper.perturbation(currentSwarm);
    }

    public void updateLeaders() {
        List<S> currentSwarm = algorithmWrapper.getSwarm();
        algorithmWrapper.updateLeaders(currentSwarm);
    }

    public  void updateParticlesMemory() {
        List<S> currentSwarm = algorithmWrapper.getSwarm();
        algorithmWrapper.updateParticlesMemory(currentSwarm);
    }

    public void initProgress()
    {
        algorithmWrapper.initProgress();
    }

    public void updateProgress()
    {
        algorithmWrapper.updateProgress();
    }

    public boolean isStoppingConditionReached()
    {
        return algorithmWrapper.isStoppingConditionReached();
    }

    public List<S> getSwarm()
    {
        return algorithmWrapper.getSwarm();
    }

    public void setSwarm(List<S> newSwarm)
    {
        algorithmWrapper.setSwarm(newSwarm);
    }

    public void forceStoppingCondition() //folosita pt a forta conditia de oprire a algoritmului, modificand nr de iteratii
    {
        try {
            int maxIterations = algorithmWrapper.getMaxIterations();
            algorithmWrapper.setIterations(maxIterations);
        }
        catch (Exception ex)
        {
            // TODO
        }
    }
}