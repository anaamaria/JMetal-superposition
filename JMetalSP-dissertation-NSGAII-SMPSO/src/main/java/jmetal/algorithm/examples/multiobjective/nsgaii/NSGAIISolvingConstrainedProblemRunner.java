package jmetal.algorithm.examples.multiobjective.nsgaii;

import java.io.IOException;
import java.util.List;
import java.util.stream.Collectors;
import jmetal.core.algorithm.Algorithm;
import jmetal.algorithm.examples.AlgorithmRunner;
import jmetal.algorithm.multiobjective.nsgaii.NSGAIIBuilder;
import jmetal.core.operator.crossover.CrossoverOperator;
import jmetal.core.operator.crossover.impl.SBXCrossover;
import jmetal.core.operator.mutation.MutationOperator;
import jmetal.core.operator.mutation.impl.PolynomialMutation;
import jmetal.core.operator.selection.SelectionOperator;
import jmetal.core.operator.selection.impl.BinaryTournamentSelection;
import jmetal.core.problem.Problem;
import jmetal.problem.ProblemFactory;
import jmetal.core.qualityindicator.QualityIndicatorUtils;
import jmetal.core.solution.doublesolution.DoubleSolution;
import jmetal.core.util.AbstractAlgorithmRunner;
import jmetal.core.util.ConstraintHandling;
import jmetal.core.util.JMetalLogger;
import jmetal.core.util.SolutionListUtils;
import jmetal.core.util.VectorUtils;
import jmetal.core.util.comparator.RankingAndCrowdingDistanceComparator;
import jmetal.core.util.comparator.constraintcomparator.impl.OverallConstraintViolationDegreeComparator;
import jmetal.core.util.comparator.dominanceComparator.impl.DominanceWithConstraintsComparator;
import jmetal.core.util.errorchecking.JMetalException;

/**
 * Class to configure and run the NSGA-II algorithm
 *
 * @author Antonio J. Nebro <antonio@lcc.uma.es>
 */
public class NSGAIISolvingConstrainedProblemRunner extends AbstractAlgorithmRunner {

  /**
   * @param args Command line arguments.
   */
  public static void main(String[] args) throws JMetalException, IOException {
    String problemName = "jmetal.core.problem.multiobjective.Osyczka2";
    String referenceParetoFront = "resources/referenceFrontsCSV/Osyczka2.csv";

    Problem<DoubleSolution> problem = ProblemFactory.<DoubleSolution>loadProblem(problemName);

    double crossoverProbability = 0.9;
    double crossoverDistributionIndex = 20.0;
    CrossoverOperator<DoubleSolution> crossover = new SBXCrossover(crossoverProbability,
        crossoverDistributionIndex);

    double mutationProbability = 1.0 / problem.numberOfVariables();
    double mutationDistributionIndex = 20.0;
    MutationOperator<DoubleSolution> mutation = new PolynomialMutation(mutationProbability,
        mutationDistributionIndex);

    SelectionOperator<List<DoubleSolution>, DoubleSolution> selection = new BinaryTournamentSelection<>(
        new RankingAndCrowdingDistanceComparator<>());

    int populationSize = 100;
    Algorithm<List<DoubleSolution>> algorithm =
        new NSGAIIBuilder<>(problem, crossover, mutation, populationSize)
            .setSelectionOperator(selection)
            .setMaxEvaluations(25000)
            .setDominanceComparator(new DominanceWithConstraintsComparator<>(
                new OverallConstraintViolationDegreeComparator<>()))
            .build();

    AlgorithmRunner algorithmRunner = new AlgorithmRunner.Executor(algorithm).execute();

    List<DoubleSolution> population = algorithm.result().stream().filter(
        ConstraintHandling::isFeasible).collect(
        Collectors.toList());

    long computingTime = algorithmRunner.getComputingTime();

    JMetalLogger.logger.info("Total execution time: " + computingTime + "ms");

    printFinalSolutionSet(population);
    QualityIndicatorUtils.printQualityIndicators(
        SolutionListUtils.getMatrixWithObjectiveValues(population),
        VectorUtils.readVectors(referenceParetoFront, ","));
  }
}