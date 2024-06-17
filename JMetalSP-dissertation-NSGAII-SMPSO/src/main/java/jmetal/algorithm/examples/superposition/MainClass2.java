package jmetal.algorithm.examples.superposition;

import jmetal.algorithm.examples.superposition.NSGAIII_SMPSO.SuperPositionCombinator1;
import jmetal.algorithm.examples.superposition.algorithms.SuperPositionNSGAIII;
import jmetal.algorithm.examples.superposition.algorithms.SuperPositionSMPSO;
import jmetal.algorithm.multiobjective.nsgaiii.NSGAIII;
import jmetal.algorithm.multiobjective.nsgaiii.NSGAIIIBuilder;
import jmetal.algorithm.multiobjective.nsgaiii.NSGAIIIWrapper;
import jmetal.algorithm.multiobjective.smpso.SMPSO;
import jmetal.algorithm.multiobjective.smpso.SMPSOBuilder;
import jmetal.algorithm.multiobjective.smpso.SMPSOWrapper;
import jmetal.core.algorithm.impl.GeneticAlgorithmWrapper;
import jmetal.core.algorithm.impl.ParticleSwarmWrapper;
import jmetal.core.operator.crossover.CrossoverOperator;
import jmetal.core.operator.crossover.impl.SBXCrossover;
import jmetal.core.operator.mutation.MutationOperator;
import jmetal.core.operator.mutation.impl.PolynomialMutation;
import jmetal.core.operator.selection.SelectionOperator;
import jmetal.core.operator.selection.impl.BinaryTournamentSelection;
import jmetal.core.problem.doubleproblem.impl.AbstractDoubleProblem;
import jmetal.core.problem.doubleproblem.impl.GapProblem;
import jmetal.core.solution.doublesolution.DoubleSolution;
import jmetal.core.util.archive.BoundedArchive;
import jmetal.core.util.archive.impl.CrowdingDistanceArchive;
import jmetal.core.util.comparator.RankingAndCrowdingDistanceComparator;
import jmetal.core.util.errorchecking.JMetalException;
import jmetal.core.util.evaluator.impl.SequentialSolutionListEvaluator;

import java.util.List;

public class MainClass2 {
    public static void main(String[] args) throws JMetalException
    {
        AbstractDoubleProblem problem = new GapProblem(6, 2);

        // PARAMETERS for NSGAIII
        double crossoverProbability1 = 0.9;
        double crossoverDistributionIndex1 = 20.0;
        CrossoverOperator<DoubleSolution> crossover1 = new SBXCrossover(crossoverProbability1, crossoverDistributionIndex1);

        double mutationProbability1 = 1.0 / problem.numberOfVariables();
        double mutationDistributionIndex1 = 20.0;
        MutationOperator<DoubleSolution> mutation1 = new PolynomialMutation(mutationProbability1, mutationDistributionIndex1);

        SelectionOperator<List<DoubleSolution>, DoubleSolution> selection1 = new BinaryTournamentSelection<>(new RankingAndCrowdingDistanceComparator<>());

        // BUILD NSGAIII
        NSGAIII<DoubleSolution> nsgaIII =
                new NSGAIIIBuilder<>(problem)
                        .setCrossoverOperator(crossover1)
                        .setMutationOperator(mutation1)
                        .setSelectionOperator(selection1)
                        .setPopulationSize(10)
                        .setMaxIterations(10)
                        .setNumberOfDivisions(4)
                        .build();

        // PARAMETERS + BUILD for SMPSO
        //DoubleProblem problem = (DoubleProblem) ProblemFactory.<DoubleSolution>loadProblem(problemName);
        BoundedArchive<DoubleSolution> archive = new CrowdingDistanceArchive<>(100);

        double mutationProbability = 1.0 / problem.numberOfVariables();
        double mutationDistributionIndex = 20.0;
        var mutation = new PolynomialMutation(mutationProbability, mutationDistributionIndex);

        SMPSO smpso = new SMPSOBuilder(problem, archive)
                .setMutation(mutation)
                .setMaxIterations(10)
                .setSwarmSize(10)
                .setSolutionListEvaluator(new SequentialSolutionListEvaluator<DoubleSolution>())
                .build();

        GeneticAlgorithmWrapper<DoubleSolution> nsgaIIIWrapper
                = new NSGAIIIWrapper<>(nsgaIII);

        ParticleSwarmWrapper<DoubleSolution> smpsoWrapper
                = new SMPSOWrapper<>(smpso);

        AbstractSuperPositionGA<DoubleSolution, List<DoubleSolution>> superPositionNSGAIII
                = new SuperPositionNSGAIII<>(nsgaIIIWrapper);

        superPositionNSGAIII.onNewGeneration = () -> System.out.println("New NSGA-III generation done!");

        AbstractSuperPositionPSO<DoubleSolution, List<DoubleSolution>> superPositionSMPSO
                = new SuperPositionSMPSO<>(smpsoWrapper);

        superPositionSMPSO.onNewGeneration = () -> System.out.println("----------\nNew SMPSO generation done!");

        AbstractSuperPositionGAPSOCombinator<DoubleSolution> superPositionCombinator
                = new SuperPositionCombinator1<>(superPositionNSGAIII, superPositionSMPSO);

        superPositionCombinator.onNewGenerationDone = () -> System.out.println("New Super-Position generation done!");
        superPositionCombinator.onSuperPositionDone = () -> {

            System.out.println("----------\nSuper-Position have finished!");

            List<DoubleSolution> result = superPositionCombinator.getResult();
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
        };

        Thread combinatorThread = new Thread(superPositionCombinator);
        Thread nsgaIIIThread = new Thread(superPositionNSGAIII);
        Thread smpsoThread = new Thread(superPositionSMPSO);

        combinatorThread.start();
        nsgaIIIThread.start();
        smpsoThread.start();
    }
}
