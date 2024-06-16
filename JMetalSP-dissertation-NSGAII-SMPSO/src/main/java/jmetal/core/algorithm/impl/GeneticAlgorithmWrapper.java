package jmetal.core.algorithm.impl;

import jmetal.core.problem.Problem;

import java.lang.reflect.Field;
import java.util.List;

public class GeneticAlgorithmWrapper<S>
{
    protected AbstractGeneticAlgorithm<S, List<S>> algorithm;

    public GeneticAlgorithmWrapper(AbstractGeneticAlgorithm<S, List<S>> algorithm)
    {
        this.algorithm = algorithm;
    }

    public List<S> createInitialPopulation()
    {
        return algorithm.createInitialPopulation();
    }

    public List<S> evaluatePopulation(List<S> population)
    {
        return algorithm.evaluatePopulation(population);
    }

    public boolean isStoppingConditionReached()
    {
        return algorithm.isStoppingConditionReached();
    }

    public List<S> selection(List<S> population)
    {
        return algorithm.selection(population);
    }

    public List<S> reproduction(List<S> population)
    {
        return algorithm.reproduction(population);
    }

    public List<S> replacement(List<S> population, List<S> offspringPopulation)
    {
        return algorithm.replacement(population, offspringPopulation);
    }

    public void initProgress()
    {
        algorithm.initProgress();
    }

    public void updateProgress()
    {
        algorithm.updateProgress();
    }

    public Problem<S> getProblem()
    {
        return algorithm.getProblem();
    }

    public List<S> result()
    {
        return algorithm.result();
    }

    public int getMaxPopulationSize(){ return algorithm.maxPopulationSize; }

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