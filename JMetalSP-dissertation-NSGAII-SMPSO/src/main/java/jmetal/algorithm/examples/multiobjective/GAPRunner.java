package jmetal.algorithm.examples.multiobjective;

import jmetal.algorithm.examples.superposition.AbstractSuperPositionGA;
import jmetal.algorithm.examples.superposition.AbstractSuperPositionGAPSOCombinator;
import jmetal.algorithm.examples.superposition.AbstractSuperPositionPSO;
import jmetal.algorithm.examples.superposition.NSGAIII_SMPSO.SuperPositionCombinator1;
import jmetal.algorithm.examples.superposition.algorithms.SuperPositionNSGAII;
import jmetal.algorithm.examples.superposition.algorithms.SuperPositionNSGAIII;
import jmetal.algorithm.examples.superposition.algorithms.SuperPositionSMPSO;
import jmetal.algorithm.multiobjective.nsgaii.NSGAII;
import jmetal.algorithm.multiobjective.nsgaii.NSGAIIWrapper;
import jmetal.algorithm.multiobjective.nsgaiii.NSGAIII;
import jmetal.algorithm.multiobjective.nsgaiii.NSGAIIIBuilder;
import jmetal.algorithm.multiobjective.nsgaiii.NSGAIIIWrapper;
import jmetal.algorithm.multiobjective.smpso.SMPSO;
import jmetal.algorithm.multiobjective.smpso.SMPSOBuilder;
import jmetal.algorithm.multiobjective.smpso.SMPSOWrapper;
import jmetal.algorithm.multiobjective.nsgaii.NSGAIIBuilder;
import jmetal.core.algorithm.impl.GeneticAlgorithmWrapper;
import jmetal.core.algorithm.impl.ParticleSwarmWrapper;
import jmetal.core.operator.crossover.CrossoverOperator;
import jmetal.core.operator.crossover.impl.SBXCrossover;
import jmetal.core.operator.mutation.MutationOperator;
import jmetal.core.operator.mutation.impl.PolynomialMutation;
import jmetal.core.operator.selection.SelectionOperator;
import jmetal.core.operator.selection.impl.BinaryTournamentSelection;
import jmetal.core.problem.Problem;
import jmetal.core.problem.doubleproblem.DoubleProblem;
import jmetal.core.problem.doubleproblem.impl.AbstractDoubleProblem;
import jmetal.core.problem.doubleproblem.impl.GapProblem;
import jmetal.core.qualityindicator.impl.GenerationalDistance;
import jmetal.core.qualityindicator.impl.SetCoverage;
import jmetal.core.qualityindicator.impl.hypervolume.Hypervolume;
import jmetal.core.qualityindicator.impl.hypervolume.impl.PISAHypervolume;
import jmetal.core.solution.doublesolution.DoubleSolution;
import jmetal.core.util.AbstractAlgorithmRunner;
import jmetal.core.util.JMetalLogger;
import jmetal.core.util.archive.BoundedArchive;
import jmetal.core.util.archive.impl.CrowdingDistanceArchive;
import jmetal.core.util.comparator.RankingAndCrowdingDistanceComparator;
import jmetal.core.util.evaluator.impl.SequentialSolutionListEvaluator;
import jmetal.problem.ProblemFactory;
import jmetal.problem.multiobjective.dtlz.DTLZ1;
import org.xml.sax.SAXException;

import javax.xml.parsers.ParserConfigurationException;
import javax.xml.transform.TransformerException;
import java.io.IOException;
import java.util.List;

public class GAPRunner extends AbstractAlgorithmRunner {

    private static double[] computeCoverageValue(DoubleSolution solution) {
        double[] coverageValues = new double[solution.variables().size()];
        for (int i = 0; i < solution.variables().size(); i++) {
            coverageValues[i] = solution.variables().get(i);
        }
        return coverageValues;
    }

    private static double[][] generateTrueParetoFront(DoubleProblem problem, int numberOfPoints) {
        int numberOfObjectives = problem.numberOfObjectives();
        double[][] paretoFront = new double[numberOfPoints][numberOfObjectives];

        for (int i = 0; i < numberOfPoints; i++) {
            double[] point = new double[numberOfObjectives];
            double sum = 0.0;
            for (int j = 0; j < numberOfObjectives; j++) {
                if (j == 0) {
                    point[j] = (double) i / (numberOfPoints - 1);
                } else {
                    point[j] = 0.5 * (1 - Math.cos((double) (i * Math.PI) / (numberOfPoints - 1)));
                }
                sum += point[j];
            }
            for (int j = 0; j < numberOfObjectives; j++) {
                paretoFront[i][j] = point[j] / sum;
            }
        }
        return paretoFront;
    }

    public static void evaluateMetrics(List<DoubleSolution> population, double[][] referenceParetoFront) {
        // Convert the population and reference Pareto front to double arrays for evaluation
        double[][] populationArray = population.stream()
                .map(solution -> solution.variables().stream().mapToDouble(Double::doubleValue).toArray())
                .toArray(double[][]::new);

        double[][] adjustedReferenceParetoFront = new double[referenceParetoFront.length][2];
        for (int i = 0; i < referenceParetoFront.length; i++) {
            System.arraycopy(referenceParetoFront[i], 0, adjustedReferenceParetoFront[i], 0, 2);
        }

        // Hypervolume
        PISAHypervolume hypervolume = new PISAHypervolume(adjustedReferenceParetoFront);
        double hypervolumeValue = hypervolume.compute(populationArray);
        JMetalLogger.logger.info("Hypervolume: " + hypervolumeValue);

        // Set Coverage A->B
        SetCoverage setCoverage = new SetCoverage(adjustedReferenceParetoFront);
        double setCoverageValueAB = setCoverage.compute(populationArray);
        JMetalLogger.logger.info("Set Coverage (A -> B): " + setCoverageValueAB);
    }

    private static boolean dominates(double[] solution, double[] point) {
        boolean dominates = false;
        for (int i = 0; i < solution.length; i++) {
            if (solution[i] > point[i]) {
                return false;
            } else if (solution[i] < point[i]) {
                dominates = true;
            }
        }
        return dominates;
    }

    private static void runBat() {
        String batFilePath = "C:\\Users\\Ana\\Downloads\\JMetalSP-dissertation-NSGAII-SMPSO\\JMetalSP-dissertation-NSGAII-SMPSO\\simulator\\gap_dump_1717571900584_default-mibench-netw-dijkstra\\run.bat";
        ProcessBuilder processBuilder = new ProcessBuilder("cmd.exe", "/c", batFilePath);

        try {
            Process process = processBuilder.start();
            int exitCode = process.waitFor();
            System.out.println("Batch file executed with exit code: " + exitCode);
        } catch (IOException e) {
            System.err.println("IOException occurred while executing batch file: " + e.getMessage());
            e.printStackTrace();
        } catch (InterruptedException e) {
            System.err
                    .println("InterruptedException occurred while waiting for batch file execution: " + e.getMessage());
            e.printStackTrace();
            Thread.currentThread().interrupt(); // Restore interrupted status
        }
    }

    public static void main(String[] args) throws IOException, ParserConfigurationException, SAXException, TransformerException {
        //String problemName = "jmetal.problem.multiobjective.gap.GAPProblem";
        //Problem<DoubleSolution> problem = ProblemFactory.<DoubleSolution>loadProblem(problemName);

        //AbstractDoubleProblem problem = new DTLZ1(3, 2); //3 2
        runBat();
        AbstractDoubleProblem problem = new GapProblem(6, 2);

        // PARAMETERS for NSGAII
        double crossoverProbability1 = 0.9;
        double crossoverDistributionIndex1 = 20.0;
        CrossoverOperator<DoubleSolution> crossover1 = new SBXCrossover(crossoverProbability1, crossoverDistributionIndex1);

        double mutationProbability1 = 1.0 / problem.numberOfVariables();
        double mutationDistributionIndex1 = 20.0;
        MutationOperator<DoubleSolution> mutation1 = new PolynomialMutation(mutationProbability1, mutationDistributionIndex1);

        SelectionOperator<List<DoubleSolution>, DoubleSolution> selection1 = new BinaryTournamentSelection<>(new RankingAndCrowdingDistanceComparator<>());

        NSGAIIBuilder<DoubleSolution> builder = new NSGAIIBuilder<DoubleSolution>(problem, crossover1, mutation1, 50); //100
        builder.setMaxEvaluations(250)
                .setMatingPoolSize(100);


        // Build and run NSGA-II algorithm
        NSGAII<DoubleSolution> algorithm = builder.build();

//         NSGAIII<DoubleSolution> nsgaIII =
//                 new NSGAIIIBuilder<>(problem)
//                         .setCrossoverOperator(crossover1)
//                         .setMutationOperator(mutation1)
//                         .setSelectionOperator(selection1)
//                         .setPopulationSize(35)
//                         .setMaxIterations(10)
//                         .setNumberOfDivisions(4)
//                         .build();

        // PARAMETERS + BUILD for SMPSO
        //DoubleProblem problem = (DoubleProblem) ProblemFactory.<DoubleSolution>loadProblem(problemName);
        BoundedArchive<DoubleSolution> archive = new CrowdingDistanceArchive<>(50); //100

        double mutationProbability = 1.0 / problem.numberOfVariables();
        double mutationDistributionIndex = 20.0;
        var mutation = new PolynomialMutation(mutationProbability, mutationDistributionIndex);

        SMPSO smpso = new SMPSOBuilder((DoubleProblem) problem, archive)
                .setMutation(mutation)
                .setMaxIterations(250)
                .setSwarmSize(50) //100
                .setSolutionListEvaluator(new SequentialSolutionListEvaluator<DoubleSolution>())
                .build();

        GeneticAlgorithmWrapper<DoubleSolution> nsgaIIWrapper
                = new NSGAIIWrapper<>(algorithm);

        ParticleSwarmWrapper<DoubleSolution> smpsoWrapper
                = new SMPSOWrapper<>(smpso);

        AbstractSuperPositionGA<DoubleSolution, List<DoubleSolution>> superPositionNSGAII
                = new SuperPositionNSGAII<>(nsgaIIWrapper);

        superPositionNSGAII.onNewGeneration = () -> System.out.println("New NSGA-II generation done!");

        AbstractSuperPositionPSO<DoubleSolution, List<DoubleSolution>> superPositionSMPSO
                = new SuperPositionSMPSO<>(smpsoWrapper);

        superPositionSMPSO.onNewGeneration = () -> System.out.println("----------\nNew SMPSO generation done!");

        AbstractSuperPositionGAPSOCombinator<DoubleSolution> superPositionCombinator
                = new SuperPositionCombinator1<>(superPositionNSGAII, superPositionSMPSO);

        superPositionCombinator.onNewGenerationDone = () -> System.out.println("New Super-Position generation done!");
        superPositionCombinator.onSuperPositionDone = () -> {

            System.out.println("----------\nSuper-Position have finished!");

            List<DoubleSolution> result = superPositionCombinator.getResult();

//            SetCoverage setCoverageMetric = new SetCoverage();
//            double[][] coverageValues = new double[result.size()][problem.numberOfVariables()];
//            for (int i = 0; i < result.size(); i++) {
//                coverageValues[i] = computeCoverageValue(result.get(i)); // Define your method to compute coverage value
//            }
//            double coverage = setCoverageMetric.compute(coverageValues);
//            System.out.println("SetCoverage: " + coverage);

            int noOfObj = problem.numberOfObjectives();

            System.out.println("Super-Position result (objectives) population:");

            for(int solIndex = 0; solIndex <= result.size() - 1; solIndex++)
            {
                System.out.print("Individual " + solIndex + ": " );

                for(int objIndex = 0; objIndex <= noOfObj - 1; objIndex++)
                {
                    System.out.print(result.get(solIndex).objectives()[objIndex] + " ");
                }

                System.out.println();
            }

            //  Generate the true Pareto front
            double[][] referenceParetoFront = generateTrueParetoFront(problem, 2);

            //  Evaluate the solutions using quality indicators
            evaluateMetrics(result, referenceParetoFront);
        };

        Thread combinatorThread = new Thread(superPositionCombinator);
        Thread nsgaIIThread = new Thread(superPositionNSGAII);
        Thread smpsoThread = new Thread(superPositionSMPSO);

        combinatorThread.start();
        nsgaIIThread.start();
        smpsoThread.start();
    }
}
