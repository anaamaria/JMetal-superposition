package jmetal.algorithm.examples.superposition;

import jmetal.algorithm.examples.superposition.events.IAction;
import jmetal.core.solution.Solution;
import jmetal.core.util.ranking.Ranking;
import jmetal.core.util.ranking.impl.FastNonDominatedSortRanking;

import java.util.ArrayList;
import java.util.List;

public abstract class AbstractSuperPositionGAGACombinator<S extends Solution<?>> implements Runnable
{
    protected AbstractSuperPositionGA<S, List<S>> algorithm1;
    protected AbstractSuperPositionGA<S, List<S>> algorithm2;
    protected boolean bothAlgorithmsFinished;
    protected List<S> resultPopulation;
    public IAction onNewGenerationDone;
    public IAction onSuperPositionDone;

    public AbstractSuperPositionGAGACombinator(AbstractSuperPositionGA<S, List<S>> alg1,
                                               AbstractSuperPositionGA<S, List<S>> alg2)
    {
        algorithm1 = alg1;
        algorithm2 = alg2;

        bothAlgorithmsFinished = false;
    }

    protected abstract List<S> ApplySuperPosition();
    
    public List<S> getResult()
    {
        return resultPopulation;
    }

    //bucla principala a combinatorului, Verifică dacă ambii algoritmi sunt în starea de așteptare (isWaiting) și dacă unul dintre algoritmi a atins
    // condiția de oprire. Forțează condiția de oprire pentru algoritmul care a atins-o mai devreme și setează bothAlgorithmsFinished la true.
    @Override
    public void run()
    {
        while(!bothAlgorithmsFinished)
        {
            if(algorithm1.isWaiting && algorithm2.isWaiting)
            {
                if(algorithm1.isStoppingConditionReached())
                {
                    if(!algorithm2.isStoppingConditionReached())
                    {
                        algorithm2.forceStoppingCondition();
                    }

                    bothAlgorithmsFinished = true;

                    algorithm1.isWaiting = false;
                    algorithm2.isWaiting = false;

                    resultPopulation = ApplySuperPosition();

                    onNewGenerationDone.invoke();

                    onSuperPositionDone.invoke();
                }
                else if(algorithm2.isStoppingConditionReached())
                {
                    algorithm1.forceStoppingCondition();

                    bothAlgorithmsFinished = true;

                    algorithm1.isWaiting = false;
                    algorithm2.isWaiting = false;

                    resultPopulation = ApplySuperPosition();

                    onSuperPositionDone.invoke();
                }
                else
                {
                    resultPopulation = ApplySuperPosition();

                    onNewGenerationDone.invoke();

                    algorithm1.isWaiting = false;
                    algorithm2.isWaiting = false;
                }
            }
        }
    }

    // 2. RANKING AND TAKING THE BEST FRONT - we make this step to take the best 100 individuals from the combined population
    protected Ranking<S> computeRanking(List<S> solutionList)
    {
        Ranking<S> ranking = new FastNonDominatedSortRanking<>();
        ranking.compute(solutionList) ;

        return ranking ;
    }

    // 3. GENERATE NEW POPULATION - here we set the 100 best as the new generation
    protected List<S> Take(Ranking<S> fronts, int populationCount)
    {
        List<S> population = new ArrayList<>();

        int noOfSelectedIndividuals = 0;
        int noOfFronts = fronts.getNumberOfSubFronts();

        for (int frontIndex = 0; frontIndex <= noOfFronts - 1; frontIndex++)
        {
            List<S> currentFront = fronts.getSubFront(frontIndex);

            int currentFrontSize = currentFront.size();

            int currentIndividualIndex = 0;

            while (noOfSelectedIndividuals < populationCount && currentIndividualIndex < currentFrontSize)
            {
                population.add(currentFront.get(currentIndividualIndex));
                noOfSelectedIndividuals++;
                currentIndividualIndex++;
            }
        }

        return population;
    }
}

//nimic schimbat aici