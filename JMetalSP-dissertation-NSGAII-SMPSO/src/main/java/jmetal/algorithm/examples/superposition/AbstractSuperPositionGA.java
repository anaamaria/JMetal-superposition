package jmetal.algorithm.examples.superposition;

import jmetal.algorithm.examples.superposition.events.IAction;
import jmetal.core.algorithm.impl.AbstractEvolutionaryAlgorithm;
import jmetal.core.algorithm.impl.GeneticAlgorithmWrapper;

import java.util.List;

public abstract class AbstractSuperPositionGA<S, R> extends AbstractEvolutionaryAlgorithm<S, R> implements Runnable
{
    protected GeneticAlgorithmWrapper<S> algorithmWrapper;
    public volatile boolean isWaiting; //folosit pt a executa algoritmii in paralel
    public IAction onNewGeneration; //obiect de tipul IAction care reprezinta o actiune ce va fi executata la fiecare noua generatie

    public AbstractSuperPositionGA(GeneticAlgorithmWrapper<S> wrapper)
    {
        algorithmWrapper = wrapper;

        isWaiting = false;

        setProblem(algorithmWrapper.getProblem());
    }

    @Override
    public void run()
    {
        List<S> offspringPopulation;
        List<S> matingPopulation;

        population = createInitialPopulation();
        population = evaluatePopulation(population);
        initProgress();
        while (!isStoppingConditionReached()) {
            matingPopulation = selection(population);
            offspringPopulation = reproduction(matingPopulation);
            offspringPopulation = evaluatePopulation(offspringPopulation);
            population = replacement(population, offspringPopulation);

            onNewGeneration.invoke();

            updateProgress();

            isWaiting = true;

            while (isWaiting)
            {
                Thread.onSpinWait();
            }
        }
    }

    @Override
    protected List<S> createInitialPopulation()
    {
        return algorithmWrapper.createInitialPopulation();
    }

    @Override
    protected List<S> evaluatePopulation(List<S> population)
    {
        return  algorithmWrapper.evaluatePopulation(population);
    }

    @Override
    public boolean isStoppingConditionReached()
    {
        return algorithmWrapper.isStoppingConditionReached();
    }

    @Override
    protected List<S> selection(List<S> population)
    {
        return algorithmWrapper.selection(population);
    }

    @Override
    protected List<S> reproduction(List<S> population)
    {
        return algorithmWrapper.reproduction(population);
    }

    @Override
    protected List<S> replacement(List<S> population, List<S> offspringPopulation)
    {
        return algorithmWrapper.replacement(population, offspringPopulation);
    }

    @Override
    protected void initProgress()
    {
        algorithmWrapper.initProgress();
    }

    @Override
    protected void updateProgress()
    {
        algorithmWrapper.updateProgress();
    }

    public int getMaxPopulationSize() {
        return algorithmWrapper.getMaxPopulationSize() ;
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
//nimic schimbat aici