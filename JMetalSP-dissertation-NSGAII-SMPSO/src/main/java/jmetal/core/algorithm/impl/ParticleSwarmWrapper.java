package jmetal.core.algorithm.impl;

import java.lang.reflect.Field;
import java.util.List;

public class ParticleSwarmWrapper<S>
{
    protected AbstractParticleSwarmOptimization<S, List<S>> algorithm;

    public ParticleSwarmWrapper(AbstractParticleSwarmOptimization<S, List<S>> algorithm)
    {
        this.algorithm = algorithm;
    }

    public List<S> createInitialSwarm()
    {
        return algorithm.createInitialSwarm();
    }

    public List<S> evaluateSwarm(List<S> swarm)
    {
        return  algorithm.evaluateSwarm(swarm);
    }

    public void initializeLeader(List<S> swarm)
    {
        algorithm.initializeLeader(swarm);
    }

    public void initializeParticlesMemory(List<S> swarm)
    {
        algorithm.initializeParticlesMemory(swarm);
    }

    public void initializeVelocity(List<S> swarm)
    {
        algorithm.initializeVelocity(swarm);
    }

    public void updateVelocity(List<S> swarm)
    {
        algorithm.updateVelocity(swarm);
    }

    public void updatePosition(List<S> swarm)
    {
        algorithm.updatePosition(swarm);
    }

    public void perturbation(List<S> swarm)
    {
        algorithm.perturbation(swarm);
    }

    public void updateLeaders(List<S> swarm)
    {
        algorithm.updateLeaders(swarm);
    }

    public  void updateParticlesMemory(List<S> swarm)
    {
        algorithm.updateParticlesMemory(swarm);
    }

    public void initProgress()
    {
        algorithm.initProgress();
    }

    public void updateProgress()
    {
        algorithm.updateProgress();
    }

    public boolean isStoppingConditionReached()
    {
        return algorithm.isStoppingConditionReached();
    }
    public List<S> result()
    {
        return algorithm.result();
    }

    public List<S> getSwarm()
    {
        return algorithm.getSwarm();
    }

    public void setSwarm(List<S> swarm)
    {
        algorithm.setSwarm(swarm);
    }

    //asta am adaugat din GeneticAlgorithmWrapper
    public int getMaxIterations() throws Exception
    {
        Class myClass = algorithm.getClass();
        Field myField = getField(myClass, "maxIterations");
        myField.setAccessible(true);
        return (int) myField.get(algorithm);
    }

    public void setIterations(int n) throws Exception
    {
        Class myClass = algorithm.getClass();
        Field myField = getField(myClass, "iterations");
        myField.setAccessible(true);
        myField.set(algorithm, n);
    }

    protected Field getField(Class clazz, String fieldName) throws NoSuchFieldException
    {
        try
        {
            return clazz.getDeclaredField(fieldName);
        }
        catch (NoSuchFieldException e)
        {
            Class superClass = clazz.getSuperclass();
            if (superClass == null)
            {
                throw e;
            }
            else
            {
                return getField(superClass, fieldName);
            }
        }
    }
}