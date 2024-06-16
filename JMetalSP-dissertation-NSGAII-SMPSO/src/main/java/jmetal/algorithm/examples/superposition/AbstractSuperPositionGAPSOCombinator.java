package jmetal.algorithm.examples.superposition;

import jmetal.algorithm.examples.superposition.events.IAction;
import jmetal.algorithm.multiobjective.smpso.SMPSO;
import jmetal.core.solution.Solution;
import jmetal.core.solution.doublesolution.DoubleSolution;
import jmetal.core.util.ranking.Ranking;
import jmetal.core.util.ranking.impl.FastNonDominatedSortRanking;

import java.util.ArrayList;
import java.util.List;

public abstract class AbstractSuperPositionGAPSOCombinator<S extends Solution<?>>  implements Runnable//am adaugat partea cu extends
{
    protected AbstractSuperPositionGA<S, List<S>> algorithm1;
    protected AbstractSuperPositionPSO<S, List<S>> algorithm2;
    protected boolean bothAlgorithmsFinished;
    protected List<S> resultPopulation;
    public IAction onNewGenerationDone; //am adaugat asta
    public IAction onSuperPositionDone; //am adaugat asta

    public AbstractSuperPositionGAPSOCombinator(AbstractSuperPositionGA<S, List<S>> alg1,
                                                AbstractSuperPositionPSO<S, List<S>> alg2)
    {
        algorithm1 = alg1;
        algorithm2 = alg2;

        bothAlgorithmsFinished = false;
    }

    protected abstract List<S> ApplySuperPosition(); //am facut asta protected, inainte era public abstract void (vezi pe git)

    public void start()
    {
        bothAlgorithmsFinished = false;
    }

    public List<S> getResult()
    {
        return resultPopulation;
    }

    //de aici am adaugat eu
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
    protected List<S> take(Ranking<S> fronts, int populationCount)
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

    protected void setPSOAttribute(List<S> population) {

//        if(algorithm2.algorithmWrapper.getAlgorithmClass() == SMPSO.class) {
//
//            algorithm2.algorithmWrapper.getField(SMPSO.class, )
//
//            for(S solution : population){
//
//            }
//        }
    }
}