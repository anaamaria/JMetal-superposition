package jmetal.algorithm.examples.multiobjective;

import jmetal.algorithm.examples.superposition.AbstractSuperPositionGA;
import jmetal.algorithm.examples.superposition.AbstractSuperPositionGAPSOCombinator;
import jmetal.algorithm.examples.superposition.AbstractSuperPositionPSO;
import jmetal.algorithm.examples.superposition.NSGAIII_SMPSO.SuperPositionCombinator1;
import jmetal.algorithm.examples.superposition.algorithms.SuperPositionNSGAII;
import jmetal.algorithm.examples.superposition.algorithms.SuperPositionSMPSO;
import jmetal.algorithm.multiobjective.nsgaii.NSGAII;
import jmetal.algorithm.multiobjective.nsgaii.NSGAIIWrapper;
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
import jmetal.core.problem.doubleproblem.DoubleProblem;
import jmetal.core.problem.doubleproblem.impl.AbstractDoubleProblem;
import jmetal.core.problem.doubleproblem.impl.GapProblem;
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

import org.xml.sax.SAXException;

import javax.xml.parsers.ParserConfigurationException;
import javax.xml.transform.TransformerException;
import java.io.IOException;
import java.util.List;

public class GAPRunner extends AbstractAlgorithmRunner {

    public static void evaluateMetrics(List<DoubleSolution> population) {
        ParetoFrontGenerator paretoFrontGenerator = new ParetoFrontGenerator();

        double[][] referenceParetoFront = paretoFrontGenerator.generateTrueParetoFront(population);

        // Convert the population to a double array for evaluation
        double[][] frontPareto = paretoFrontGenerator.convertSolutionsToObjectivesArray(population);
        
        // Hypervolume
        PISAHypervolume hypervolume = new PISAHypervolume();
        double hypervolumeValue = hypervolume.compute(frontPareto);
        JMetalLogger.logger.info("Hypervolume: " + hypervolumeValue);

        // Set Coverage A->B
        SetCoverage setCoverage = new SetCoverage();
        double setCoverageValueAB = setCoverage.compute(frontPareto, referenceParetoFront);
        JMetalLogger.logger.info("Set Coverage (A -> B): " + setCoverageValueAB);
    }

    public static void main(String[] args) throws IOException, ParserConfigurationException, SAXException, TransformerException {
        AbstractDoubleProblem problem = new GapProblem();

        // PARAMETERS for NSGAII
        double crossoverProbability1 = 0.9;
        double crossoverDistributionIndex1 = 20.0;
        CrossoverOperator<DoubleSolution> crossover1 = new SBXCrossover(crossoverProbability1, crossoverDistributionIndex1);

        double mutationProbability1 = 1.0 / problem.numberOfVariables();
        double mutationDistributionIndex1 = 20.0;
        MutationOperator<DoubleSolution> mutation1 = new PolynomialMutation(mutationProbability1, mutationDistributionIndex1);

        SelectionOperator<List<DoubleSolution>, DoubleSolution> selection1 = new BinaryTournamentSelection<>(new RankingAndCrowdingDistanceComparator<>());

        NSGAIIBuilder<DoubleSolution> builder = new NSGAIIBuilder<DoubleSolution>(problem, crossover1, mutation1, 50); //100
        builder.setMaxEvaluations(100)
                .setMatingPoolSize(2);


        // Build and run NSGA-II algorithm
        NSGAII<DoubleSolution> algorithm = builder.build();

        // PARAMETERS + BUILD for SMPSO
        //DoubleProblem problem = (DoubleProblem) ProblemFactory.<DoubleSolution>loadProblem(problemName);
        BoundedArchive<DoubleSolution> archive = new CrowdingDistanceArchive<>(2); //100

        double mutationProbability = 1.0 / problem.numberOfVariables();
        double mutationDistributionIndex = 20.0;
        var mutation = new PolynomialMutation(mutationProbability, mutationDistributionIndex);

        SMPSO smpso = new SMPSOBuilder((DoubleProblem) problem, archive)
                .setMutation(mutation)
                .setMaxIterations(100)
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

            //  Evaluate the solutions using quality indicators
            evaluateMetrics(result);
       };

        Thread combinatorThread = new Thread(superPositionCombinator);
        Thread nsgaIIThread = new Thread(superPositionNSGAII);
        Thread smpsoThread = new Thread(superPositionSMPSO);

        combinatorThread.start();
        nsgaIIThread.start();
        smpsoThread.start();
    }
}