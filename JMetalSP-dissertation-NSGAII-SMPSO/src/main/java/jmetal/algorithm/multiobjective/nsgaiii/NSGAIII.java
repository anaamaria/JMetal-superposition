package jmetal.algorithm.multiobjective.nsgaiii;

import java.util.ArrayList;
import java.util.List;
import java.util.Vector;
import jmetal.core.algorithm.impl.AbstractGeneticAlgorithm;
import jmetal.algorithm.multiobjective.nsgaiii.util.EnvironmentalSelection;
import jmetal.algorithm.multiobjective.nsgaiii.util.ReferencePoint;
import jmetal.core.solution.Solution;
import jmetal.core.util.JMetalLogger;
import jmetal.core.util.SolutionListUtils;
import jmetal.core.util.evaluator.SolutionListEvaluator;
import jmetal.core.util.ranking.Ranking;
import jmetal.core.util.ranking.impl.FastNonDominatedSortRanking;

/**
 * Created by ajnebro on 30/10/14.
 * Modified by Juanjo on 13/11/14
 *
 * This implementation is based on the code of Tsung-Che Chiang
 * http://web.ntnu.edu.tw/~tcchiang/publications/nsga3cpp/nsga3cpp.htm
 */
@SuppressWarnings("serial")
public class NSGAIII<S extends Solution<?>> extends AbstractGeneticAlgorithm<S, List<S>> {
  protected int iterations ;
  protected int maxIterations ;

  protected SolutionListEvaluator<S> evaluator ;

  protected int numberOfDivisions  ;
  protected List<ReferencePoint<S>> referencePoints = new Vector<>() ;

  /** Constructor */
  public NSGAIII(NSGAIIIBuilder<S> builder) { // can be created from the NSGAIIIBuilder within the same package
    super(builder.getProblem()) ;
    maxIterations = builder.getMaxIterations() ;

    crossoverOperator =  builder.getCrossoverOperator() ;
    mutationOperator  =  builder.getMutationOperator() ;
    selectionOperator =  builder.getSelectionOperator() ;

    evaluator = builder.getEvaluator() ;

    /// NSGAIII
    numberOfDivisions = builder.getNumberOfDivisions() ;

    (new ReferencePoint<S>()).generateReferencePoints(referencePoints,getProblem().numberOfObjectives() , numberOfDivisions);

    setMaxPopulationSize(builder.getPopulationSize());

    JMetalLogger.logger.info("rpssize: " + referencePoints.size()); ;
  }

  @Override
  protected void initProgress() {
    iterations = 1 ;
  }

  @Override
  protected void updateProgress() {
    iterations++ ;
  }

  @Override
  protected boolean isStoppingConditionReached() {
    return iterations >= maxIterations;
  }

  @Override
  protected List<S> evaluatePopulation(List<S> population) {
    population = evaluator.evaluate(population, getProblem()) ;

    return population ;
  }

  @Override
  protected List<S> selection(List<S> population) {
    List<S> matingPopulation = new ArrayList<>(population.size()) ;
    for (int i = 0; i < getMaxPopulationSize(); i++) {
      S solution = selectionOperator.execute(population);
      matingPopulation.add(solution) ;
    }

    return matingPopulation;
  }

  @Override
  protected List<S> reproduction(List<S> population) {
    List<S> offspringPopulation = new ArrayList<>(getMaxPopulationSize());
    for (int i = 0; i < getMaxPopulationSize(); i+=2) {
      List<S> parents = new ArrayList<>(2);
      parents.add(population.get(i));
      parents.add(population.get(Math.min(i + 1, getMaxPopulationSize()-1)));

      List<S> offspring = crossoverOperator.execute(parents);

      mutationOperator.execute(offspring.get(0));
      mutationOperator.execute(offspring.get(1));

      offspringPopulation.add(offspring.get(0));
      offspringPopulation.add(offspring.get(1));
    }
    return offspringPopulation ;
  }

  
  private List<ReferencePoint<S>> getReferencePointsCopy() {
	  List<ReferencePoint<S>> copy = new ArrayList<>();
	  for (ReferencePoint<S> r : this.referencePoints) {
		  copy.add(new ReferencePoint<>(r));
	  }
	  return copy;
  }
  
  @Override
  protected List<S> replacement(List<S> population, List<S> offspringPopulation) {
   
	List<S> jointPopulation = new ArrayList<>();
    jointPopulation.addAll(population) ;
    jointPopulation.addAll(offspringPopulation) ;

    Ranking<S> ranking = computeRanking(jointPopulation);
    
    //List<Solution> pop = crowdingDistanceSelection(ranking);
    List<S> last = new ArrayList<>();
    List<S> pop = new ArrayList<>();
    List<List<S>> fronts = new ArrayList<>();
    int rankingIndex = 0;
    int candidateSolutions = 0;
    while (candidateSolutions < getMaxPopulationSize()) {
      last = ranking.getSubFront(rankingIndex);
      fronts.add(last);
      candidateSolutions += last.size();
      if ((pop.size() + last.size()) <= getMaxPopulationSize())
        pop.addAll(last);
      rankingIndex++;
    }

    if (pop.size() == this.getMaxPopulationSize())
      return pop;
    
    // A copy of the reference list should be used as parameter of the environmental selection
    EnvironmentalSelection<S> selection =
            new EnvironmentalSelection<>(fronts,getMaxPopulationSize() - pop.size(),getReferencePointsCopy(),
                    getProblem().numberOfObjectives());
    
    var choosen = selection.execute(last);
    pop.addAll(choosen);
     
    return pop;
  }

  @Override
  public List<S> result() {
    return getNonDominatedSolutions(getPopulation()) ;
  }

  protected Ranking<S> computeRanking(List<S> solutionList) {
    Ranking<S> ranking = new FastNonDominatedSortRanking<>() ;
    ranking.compute(solutionList) ;

    return ranking ;
  }

  protected List<S> getNonDominatedSolutions(List<S> solutionList) {
    return SolutionListUtils.getNonDominatedSolutions(solutionList) ;
  }

  @Override public String name() {
    return "NSGAIII" ;
  }

  @Override public String description() {
    return "Nondominated Sorting Genetic Algorithm version III" ;
  }

}