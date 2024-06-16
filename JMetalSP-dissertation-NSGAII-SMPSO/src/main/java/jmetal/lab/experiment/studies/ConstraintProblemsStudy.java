package jmetal.lab.experiment.studies;

import java.io.IOException;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import jmetal.core.algorithm.Algorithm;
import jmetal.algorithm.multiobjective.gde3.GDE3Builder;
import jmetal.algorithm.multiobjective.mocell.MOCellBuilder;
import jmetal.algorithm.multiobjective.nsgaii.NSGAIIBuilder;
import jmetal.algorithm.multiobjective.smpso.SMPSOBuilder;
import jmetal.algorithm.multiobjective.spea2.SPEA2Builder;
import jmetal.lab.experiment.Experiment;
import jmetal.lab.experiment.ExperimentBuilder;
import jmetal.lab.experiment.component.impl.ComputeQualityIndicators;
import jmetal.lab.experiment.component.impl.ExecuteAlgorithms;
import jmetal.lab.experiment.component.impl.GenerateBoxplotsWithR;
import jmetal.lab.experiment.component.impl.GenerateFriedmanTestTables;
import jmetal.lab.experiment.component.impl.GenerateLatexTablesWithStatistics;
import jmetal.lab.experiment.component.impl.GenerateReferenceParetoSetAndFrontFromDoubleSolutions;
import jmetal.lab.experiment.component.impl.GenerateWilcoxonTestTablesWithR;
import jmetal.lab.experiment.util.ExperimentAlgorithm;
import jmetal.lab.experiment.util.ExperimentProblem;
import jmetal.core.operator.crossover.impl.DifferentialEvolutionCrossover;
import jmetal.core.operator.crossover.impl.SBXCrossover;
import jmetal.core.operator.mutation.impl.PolynomialMutation;
import jmetal.core.operator.selection.impl.BinaryTournamentSelection;
import jmetal.core.operator.selection.impl.DifferentialEvolutionSelection;
import jmetal.core.problem.Problem;
import jmetal.core.problem.doubleproblem.DoubleProblem;
import jmetal.problem.multiobjective.Binh2;
import jmetal.problem.multiobjective.ConstrEx;
import jmetal.problem.multiobjective.Golinski;
import jmetal.problem.multiobjective.Srinivas;
import jmetal.problem.multiobjective.Tanaka;
import jmetal.problem.multiobjective.Water;
import jmetal.core.qualityindicator.impl.Epsilon;
import jmetal.core.qualityindicator.impl.InvertedGenerationalDistancePlus;
import jmetal.core.qualityindicator.impl.NormalizedHypervolume;
import jmetal.core.qualityindicator.impl.hypervolume.impl.PISAHypervolume;
import jmetal.core.solution.doublesolution.DoubleSolution;
import jmetal.core.util.archive.impl.CrowdingDistanceArchive;
import jmetal.core.util.errorchecking.JMetalException;
import jmetal.core.util.evaluator.impl.SequentialSolutionListEvaluator;

/**
 * Example of experimental study based on solving the unconstrained problems included in jMetal.
 * <p>
 * This jmetal.experiment assumes that the reference Pareto front are known and that, given a problem named
 * P, there is a corresponding file called P.pf containing its corresponding Pareto front. If this
 * is not the case, please refer to class {@link DTLZStudy} to see an example of how to explicitly
 * indicate the name of those files.
 * <p>
 * Six quality indicators are used for performance assessment.
 * <p>
 * The steps to carry out the jmetal.experiment are:
 * 1. Configure the jmetal.experiment
 * 2. Execute the algorithms
 * 3. Generate the reference Pareto fronts
 * 4. Compute the quality indicators
 * 5. Generate Latex tables reporting means and medians
 * 6. Generate Latex tables with the result of applying the Wilcoxon Rank Sum Test
 * 7. Generate Latex tables with the ranking obtained by applying the Friedman test
 * 8. Generate R scripts to obtain boxplots
 *
 * @author Antonio J. Nebro <antonio@lcc.uma.es>
 */
public class ConstraintProblemsStudy {
  private static final int INDEPENDENT_RUNS = 25;

  public static void main(String[] args) throws IOException {
    if (args.length != 1) {
      throw new JMetalException("Needed arguments: experimentBaseDirectory");
    }
    String experimentBaseDirectory = args[0];

    List<ExperimentProblem<DoubleSolution>> problemList = new ArrayList<>();
    problemList.add(new ExperimentProblem<>(new Binh2()));
    problemList.add(new ExperimentProblem<>(new ConstrEx()));
    problemList.add(new ExperimentProblem<>(new Golinski()));
    problemList.add(new ExperimentProblem<>(new Srinivas()));
    problemList.add(new ExperimentProblem<>(new Tanaka()));
    problemList.add(new ExperimentProblem<>(new Water()));

    List<ExperimentAlgorithm<DoubleSolution, List<DoubleSolution>>> algorithmList =
            configureAlgorithmList(problemList);

    Experiment<DoubleSolution, List<DoubleSolution>> experiment =
            new ExperimentBuilder<DoubleSolution, List<DoubleSolution>>("ConstrainedProblemsStudy")
                    .setAlgorithmList(algorithmList)
                    .setProblemList(problemList)
                    .setExperimentBaseDirectory(experimentBaseDirectory)
                    .setOutputParetoFrontFileName("FUN")
                    .setOutputParetoSetFileName("VAR")
                    .setReferenceFrontDirectory(experimentBaseDirectory + "/ConstrainedProblemsStudy/referenceFronts")
                    .setIndicatorList(Arrays.asList(
                            new Epsilon(),
                            new PISAHypervolume(),
                            new NormalizedHypervolume(),
                            new InvertedGenerationalDistancePlus()))
                    .setIndependentRuns(INDEPENDENT_RUNS)
                    .setNumberOfCores(8)
                    .build();

    new ExecuteAlgorithms<>(experiment).run();
    new GenerateReferenceParetoSetAndFrontFromDoubleSolutions(experiment).run();
    new ComputeQualityIndicators<>(experiment).run();
    new GenerateLatexTablesWithStatistics(experiment).run();
    new GenerateWilcoxonTestTablesWithR<>(experiment).run();
    new GenerateFriedmanTestTables<>(experiment).run();
    new GenerateBoxplotsWithR<>(experiment).setRows(3).setColumns(3).run();
  }

  /**
   * The algorithm list is composed of pairs {@link Algorithm} + {@link Problem} which form part of
   * a {@link ExperimentAlgorithm}, which is a decorator for class {@link Algorithm}. The {@link
   * ExperimentAlgorithm} has an optional tag component, that can be set as it is shown in this example,
   * where four variants of a same algorithm are defined.
   */
  static List<ExperimentAlgorithm<DoubleSolution, List<DoubleSolution>>> configureAlgorithmList(
          List<ExperimentProblem<DoubleSolution>> problemList) {
    List<ExperimentAlgorithm<DoubleSolution, List<DoubleSolution>>> algorithms = new ArrayList<>();
    for (int run = 0; run < INDEPENDENT_RUNS; run++) {

      for (int i = 0; i < problemList.size(); i++) {
        Algorithm<List<DoubleSolution>> algorithm = new NSGAIIBuilder<>(
                problemList.get(i).getProblem(),
                new SBXCrossover(1.0, 20),
                new PolynomialMutation(1.0 / problemList.get(i).getProblem().numberOfVariables(), 20.0),
                100)
                .setMaxEvaluations(25000)
                .build();
        algorithms.add(new ExperimentAlgorithm<>(algorithm, problemList.get(i), run));
      }

      for (int i = 0; i < problemList.size(); i++) {
        Algorithm<List<DoubleSolution>> algorithm = new SPEA2Builder<DoubleSolution>(
                problemList.get(i).getProblem(),
                new SBXCrossover(1.0, 10.0),
                new PolynomialMutation(1.0 / problemList.get(i).getProblem().numberOfVariables(), 20.0))
                .build();
        algorithms.add(new ExperimentAlgorithm<>(algorithm, problemList.get(i), run));
      }

      for (int i = 0; i < problemList.size(); i++) {
        double mutationProbability = 1.0 / problemList.get(i).getProblem().numberOfVariables();
        double mutationDistributionIndex = 20.0;
        Algorithm<List<DoubleSolution>> algorithm = new SMPSOBuilder((DoubleProblem) problemList.get(i).getProblem(),
                new CrowdingDistanceArchive<DoubleSolution>(100))
                .setMutation(new PolynomialMutation(mutationProbability, mutationDistributionIndex))
                .setMaxIterations(250)
                .setSwarmSize(100)
                .setSolutionListEvaluator(new SequentialSolutionListEvaluator<DoubleSolution>())
                .build();
        algorithms.add(new ExperimentAlgorithm<>(algorithm, problemList.get(i), run));
      }
      for (int i = 0; i < problemList.size(); i++) {
        double cr = 0.5;
        double f = 0.5;

        Algorithm<List<DoubleSolution>> algorithm = new GDE3Builder((DoubleProblem) problemList.get(i).getProblem())
                .setCrossover(new DifferentialEvolutionCrossover(cr, f, DifferentialEvolutionCrossover.DE_VARIANT.RAND_1_BIN))
                .setSelection(new DifferentialEvolutionSelection())
                .setMaxEvaluations(25000)
                .setPopulationSize(100)
                .setSolutionSetEvaluator(new SequentialSolutionListEvaluator<>())
                .build();
        algorithms.add(new ExperimentAlgorithm<>(algorithm, problemList.get(i), run));
      }

      for (int i = 0; i < problemList.size(); i++) {
        Algorithm<List<DoubleSolution>> algorithm = new MOCellBuilder<DoubleSolution>(
                (DoubleProblem) problemList.get(i).getProblem(),
                new SBXCrossover(1.0, 20.0),
                new PolynomialMutation(1.0 / problemList.get(i).getProblem().numberOfVariables(), 20.0))
                .setSelectionOperator(new BinaryTournamentSelection<>())
                .setMaxEvaluations(25000)
                .setPopulationSize(100)
                .setArchive(new CrowdingDistanceArchive<DoubleSolution>(100))
                .build();
        algorithms.add(new ExperimentAlgorithm<>(algorithm, problemList.get(i), run));
      }
    }
    return algorithms;
  }

}