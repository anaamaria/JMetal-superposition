package jmetal.algorithm.examples.superposition;

import jmetal.algorithm.examples.superposition.NSGAIII_SPEA2.SuperPositionCombinator1;
import jmetal.algorithm.examples.superposition.algorithms.SuperPositionNSGAIII;
import jmetal.algorithm.examples.superposition.algorithms.SuperPositionSPEA2;
import jmetal.algorithm.multiobjective.nsgaiii.NSGAIII;
import jmetal.algorithm.multiobjective.nsgaiii.NSGAIIIBuilder;
import jmetal.algorithm.multiobjective.nsgaiii.NSGAIIIWrapper;
import jmetal.algorithm.multiobjective.spea2.SPEA2;
import jmetal.algorithm.multiobjective.spea2.SPEA2Builder;
import jmetal.algorithm.multiobjective.spea2.SPEA2Wrapper;
import jmetal.core.algorithm.impl.GeneticAlgorithmWrapper;
import jmetal.core.operator.crossover.CrossoverOperator;
import jmetal.core.operator.crossover.impl.SBXCrossover;
import jmetal.core.operator.mutation.MutationOperator;
import jmetal.core.operator.mutation.impl.PolynomialMutation;
import jmetal.core.operator.selection.SelectionOperator;
import jmetal.core.operator.selection.impl.BinaryTournamentSelection;
import jmetal.core.problem.doubleproblem.impl.AbstractDoubleProblem;
import jmetal.core.solution.doublesolution.DoubleSolution;
import jmetal.core.util.comparator.RankingAndCrowdingDistanceComparator;
import jmetal.core.util.errorchecking.JMetalException;
import jmetal.problem.multiobjective.dtlz.DTLZ1;

import java.util.List;

public class MainClass {

    public static void main(String[] args) throws JMetalException
    {
        AbstractDoubleProblem problem = new DTLZ1(3, 2);

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
                        .setPopulationSize(35)
                        .setMaxIterations(10)
                        .setNumberOfDivisions(4)
                        .build();

        // PARAMETERS for SPEA2
        double crossoverProbability = 0.9;
        double crossoverDistributionIndex = 20.0;
        var crossover = new SBXCrossover(crossoverProbability, crossoverDistributionIndex);

        double mutationProbability = 1.0 / problem.numberOfVariables();
        double mutationDistributionIndex = 20.0;
        var mutation = new PolynomialMutation(mutationProbability, mutationDistributionIndex);

        var selection = new BinaryTournamentSelection<DoubleSolution>();

        // BUILD SPEA2
        SPEA2<DoubleSolution> spea2 = new SPEA2Builder<>(problem, crossover, mutation)
                .setSelectionOperator(selection)
                .setMaxIterations(10)
                .setPopulationSize(35)
                .setK(1)
                .build();

        GeneticAlgorithmWrapper<DoubleSolution> nsgaIIIWrapper
                = new NSGAIIIWrapper<>(nsgaIII);

        GeneticAlgorithmWrapper<DoubleSolution> spea2Wrapper
                = new SPEA2Wrapper<>(spea2);

        AbstractSuperPositionGA<DoubleSolution, List<DoubleSolution>> superPositionNSGAIII
                = new SuperPositionNSGAIII<>(nsgaIIIWrapper);

        superPositionNSGAIII.onNewGeneration = () -> System.out.println("----------\nNew NSGA-III generation done!");

        AbstractSuperPositionGA<DoubleSolution, List<DoubleSolution>> superPositionSPEA2
                = new SuperPositionSPEA2<>(spea2Wrapper);

        superPositionSPEA2.onNewGeneration = () -> System.out.println("New SPEA2 generation done!");

        AbstractSuperPositionGAGACombinator<DoubleSolution> superPositionCombinator
                = new SuperPositionCombinator1<>(superPositionNSGAIII, superPositionSPEA2);

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
        Thread spea2Thread = new Thread(superPositionSPEA2);

        combinatorThread.start();
        nsgaIIIThread.start();
        spea2Thread.start();
    }
}