package jmetal.component.examples.multiobjective.nsgaii;

import java.io.IOException;
import java.util.List;
import jmetal.component.algorithm.EvolutionaryAlgorithm;
import jmetal.component.algorithm.multiobjective.NSGAIIBuilder;
import jmetal.component.catalogue.common.termination.Termination;
import jmetal.component.catalogue.common.termination.impl.TerminationByEvaluations;
import jmetal.lab.visualization.plot.PlotFront;
import jmetal.lab.visualization.plot.impl.PlotSmile;
import jmetal.core.operator.crossover.impl.SBXCrossover;
import jmetal.core.operator.mutation.impl.PolynomialMutation;
import jmetal.core.problem.Problem;
import jmetal.problem.ProblemFactory;
import jmetal.core.qualityindicator.QualityIndicatorUtils;
import jmetal.core.solution.doublesolution.DoubleSolution;
import jmetal.core.util.JMetalLogger;
import jmetal.core.util.SolutionListUtils;
import jmetal.core.util.VectorUtils;
import jmetal.core.util.errorchecking.JMetalException;
import jmetal.core.util.fileoutput.SolutionListOutput;
import jmetal.core.util.fileoutput.impl.DefaultFileOutputContext;
import jmetal.core.util.legacy.front.impl.ArrayFront;
import jmetal.core.util.pseudorandom.JMetalRandom;

/**
 * Class to configure and run the NSGA-II algorithm to solve a bi-objective problem and plotting
 * the result front with Smile (https://haifengl.github.io/)
 *
 * @author Antonio J. Nebro <antonio@lcc.uma.es>
 */
public class NSGAIIWithSmile2DChartExample {
  public static void main(String[] args) throws JMetalException, IOException {
    String problemName = "jmetal.problem.multiobjective.zdt.ZDT6";
    String referenceParetoFront = "resources/referenceFrontsCSV/ZDT6.csv";

    Problem<DoubleSolution> problem = ProblemFactory.<DoubleSolution>loadProblem(problemName);

    double crossoverProbability = 0.9;
    double crossoverDistributionIndex = 20.0;
    var crossover = new SBXCrossover(crossoverProbability, crossoverDistributionIndex);

    double mutationProbability = 1.0 / problem.numberOfVariables();
    double mutationDistributionIndex = 20.0;
    var mutation = new PolynomialMutation(mutationProbability, mutationDistributionIndex);

    int populationSize = 100;
    int offspringPopulationSize = populationSize;

    Termination termination = new TerminationByEvaluations(22000);

    EvolutionaryAlgorithm<DoubleSolution> nsgaii = new NSGAIIBuilder<>(
                    problem,
                    populationSize,
                    offspringPopulationSize,
                    crossover,
                    mutation)
        .setTermination(termination)
        .build();

    nsgaii.run();

    List<DoubleSolution> population = nsgaii.result();
    JMetalLogger.logger.info("Total execution time : " + nsgaii.totalComputingTime() + "ms");
    JMetalLogger.logger.info("Number of evaluations: " + nsgaii.numberOfEvaluations());

    new SolutionListOutput(population)
            .setVarFileOutputContext(new DefaultFileOutputContext("VAR.csv", ","))
            .setFunFileOutputContext(new DefaultFileOutputContext("FUN.csv", ","))
            .print();

    JMetalLogger.logger.info("Random seed: " + JMetalRandom.getInstance().getSeed());
    JMetalLogger.logger.info("Objectives values have been written to file FUN.csv");
    JMetalLogger.logger.info("Variables values have been written to file VAR.csv");

    QualityIndicatorUtils.printQualityIndicators(
        SolutionListUtils.getMatrixWithObjectiveValues(population),
        VectorUtils.readVectors(referenceParetoFront, ","));

    PlotFront plot = new PlotSmile(new ArrayFront(population).getMatrix(), problem.name() + " (NSGA-II)") ;
    plot.plot();
  }
}
