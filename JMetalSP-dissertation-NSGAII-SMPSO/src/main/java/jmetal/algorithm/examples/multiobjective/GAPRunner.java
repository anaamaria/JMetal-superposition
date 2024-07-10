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
import jmetal.problem.multiobjective.dtlz.DTLZ1;

import org.xml.sax.SAXException;

import javax.xml.parsers.ParserConfigurationException;
import javax.xml.transform.TransformerException;
import java.io.IOException;
import java.util.List;

public class GAPRunner extends AbstractAlgorithmRunner {

        public static double[][] normalizeFrontPareto(double[][] frontPareto) {
                int numSolutions = frontPareto.length;
                int numObjectives = frontPareto[0].length;
                double[][] normalizedFrontPareto = new double[numSolutions][numObjectives];
                double[] minValues = new double[numObjectives];
                double[] maxValues = new double[numObjectives];
            
                // Initialize min and max values
                for (int i = 0; i < numObjectives; i++) {    
                    minValues[i] = 1e-100;
                    maxValues[i] = 1e-100;
                }
            
                // Find the minimum and maximum values for each objective
                for (int i = 0; i < numObjectives; i++) {
                    for (int j = 0; j < numSolutions; j++) {
                        double value = frontPareto[j][i];
                        if (value < minValues[i]) {
                            minValues[i] = value;
                        }
                        if (value > maxValues[i]) {
                            maxValues[i] = value;
                        }
                    }
                }
            
                // Normalize the frontPareto values using the min-max normalization formula
                for (int i = 0; i < numSolutions; i++) {
                    for (int j = 0; j < numObjectives; j++) {
                        double value = frontPareto[i][j];
                        if (maxValues[j] == minValues[j]) {
                            normalizedFrontPareto[i][j] = 0.0; // Handle the case where max and min values are equal to avoid division by zero
                        } else {
                            double normalizedValue = (value - minValues[j]) / (maxValues[j] - minValues[j]);
                            normalizedFrontPareto[i][j] = Math.min(Math.max(normalizedValue, 0.0), 1.0); // Ensure the value is within [0, 1]
                        }
                    }
                }
            
                return normalizedFrontPareto;
            }

        public static void evaluateMetrics(List<DoubleSolution> population) {
                ParetoFrontGenerator paretoFrontGenerator = new ParetoFrontGenerator();

                double[][] referenceParetoFront = paretoFrontGenerator.generateTrueParetoFront(population);

                // Convert the population to a double array for evaluation
                double[][] frontPareto = paretoFrontGenerator.convertSolutionsToObjectivesArray(population);

                // Hypervolume
                // Normalize the frontPareto values to be subunitary
                double[][] normalizedFrontPareto = normalizeFrontPareto(frontPareto);

                // Calculate the hypervolume
                PISAHypervolume hypervolume = new PISAHypervolume();
                double hypervolumeValue = hypervolume.compute(normalizedFrontPareto);
                // Normalize the hypervolume value to be between [0, 1]
        double minHypervolume = 0.0; // Minimum possible hypervolume value
        double maxHypervolume = 1.0; // Maximum possible hypervolume value
        double normalizedHypervolume = Math.max(0.0, Math.min(1.0, (hypervolumeValue - minHypervolume) / (double)(maxHypervolume - minHypervolume)));
        JMetalLogger.logger.info("Hypervolume: " + hypervolumeValue + " (normalized: " + normalizedHypervolume + ")");

                
        }

        public static void main(String[] args) throws IOException, ParserConfigurationException, SAXException, TransformerException {
                AbstractDoubleProblem problem = new GapProblem(6, 2);
                //AbstractDoubleProblem problem = new DTLZ1(6, 2); //3 2

                // PARAMETERS for NSGAII
                double crossoverProbability1 = 0.9;
                double crossoverDistributionIndex1 = 20.0;
                CrossoverOperator<DoubleSolution> crossover1 = new SBXCrossover(crossoverProbability1, crossoverDistributionIndex1);

                double mutationProbability1 = 1.0 / problem.numberOfVariables();
                double mutationDistributionIndex1 = 20.0;
                MutationOperator<DoubleSolution> mutation1 = new PolynomialMutation(mutationProbability1, mutationDistributionIndex1);

                NSGAIIBuilder<DoubleSolution> builder = new NSGAIIBuilder<DoubleSolution>(problem, crossover1, mutation1, 10);
                builder.setMaxEvaluations(30)
                                .setMatingPoolSize(2);


                // Build and run NSGA-II algorithm
                NSGAII<DoubleSolution> algorithm = builder.build();

                // PARAMETERS + BUILD for SMPSO
                BoundedArchive<DoubleSolution> archive = new CrowdingDistanceArchive<>(2); //100

                double mutationProbability = 1.0 / problem.numberOfVariables();
                double mutationDistributionIndex = 20.0;
                var mutation = new PolynomialMutation(mutationProbability, mutationDistributionIndex);

                SMPSO smpso = new SMPSOBuilder((DoubleProblem) problem, archive)
                                .setMutation(mutation)
                                .setMaxIterations(30)
                                .setSwarmSize(10) //100
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

                        int noOfObj = problem.numberOfObjectives();

                        System.out.println("Super-Position result (objectives) population:");

                        for (int solIndex = 0; solIndex <= result.size() - 1; solIndex++) {
                                System.out.print("Individual " + solIndex + ": ");

                                for (int objIndex = 0; objIndex <= noOfObj - 1; objIndex++) {
                                        System.out.print(result.get(solIndex).objectives()[objIndex] + " ");
                                }

                                System.out.println();
                        }

                        smpso.run();
                        var nsgaIIPopulation = smpso.getSwarm();
                        for (int solIndex = 0; solIndex <= nsgaIIPopulation.size() - 1; solIndex++) {
                            System.out.print("Individual nsgaii " + solIndex + ": ");

                            for (int objIndex = 0; objIndex <= noOfObj - 1; objIndex++) {
                                    System.out.print(result.get(solIndex).objectives()[objIndex] + " ");
                            }

                            System.out.println();
                    }


                    // Set Coverage A->B
                SetCoverage setCoverage = new SetCoverage();
                var result1 = convertSolutionsToObjectivesArray(result);
                var result2 = convertSolutionsToObjectivesArray(nsgaIIPopulation);
                double setCoverageValueAB = setCoverage.compute(result1, result2);
                JMetalLogger.logger.info("Set Coverage (A -> B): " + setCoverageValueAB);

                // Set Coverage B->A
                double setCoverageValueBA = setCoverage.compute(result2, result1);
                JMetalLogger.logger.info("Set Coverage (B -> A): " + setCoverageValueBA);

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

        public static double[][] convertSolutionsToObjectivesArray(List<DoubleSolution> solutions) {
            double[][] objectivesArray = new double[solutions.size()][];
            for (int i = 0; i < solutions.size(); i++) {
                objectivesArray[i] = solutions.get(i).objectives();
            }
            return objectivesArray;
        }
}
