package jmetal.core.problem.doubleproblem.impl;

import java.io.File;
import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.StandardCopyOption;
import java.util.ArrayList;
import java.util.LinkedList;
import java.util.List;
import java.util.Random;

import javax.xml.parsers.DocumentBuilder;
import javax.xml.parsers.DocumentBuilderFactory;
import javax.xml.parsers.ParserConfigurationException;

import org.w3c.dom.Document;
import org.w3c.dom.Element;
import org.w3c.dom.Node;
import org.w3c.dom.NodeList;
import org.xml.sax.SAXException;

import jmetal.core.problem.integerproblem.IntegerProblem;
import jmetal.core.solution.doublesolution.DoubleSolution;
import jmetal.lab.visualization.html.impl.htmlTable.impl.MedianValuesTable.Objective;

/**
 * Fake implementation of {@link IntegerProblem}. Intended to be used in unit tests.
 *
 *         <parameter name="n_lines" description="param1" type="integer" min="4" max="32"/>
 *         <parameter name="n_columns" description="param2" type="integer" min="4" max="32"/>
 *         <parameter name="n_layers" description="param3" type="exp2" min="1" max="64"/>
 *         <parameter name="c_chunk" description="c_chunk" type="exp2" min="1" max="32"/> <!-- default: 8 (1/32) -->
 *         <parameter name="c_sets" description="c_sets" type="exp2" min="32" max="8192"/> <!-- default: 128 (32/8192) -->
 *         <parameter name="c_lines" description="c_lines" type="exp2" min="1" max="128"/> <!-- default 1 (1/128) -->
 *
 */


@SuppressWarnings("serial")
public class GapProblem extends AbstractDoubleProblem {

    static String simulator_executable = "";
    /** Temporary objectives */
    public static final String NUMBER_OF_COLUMNS = "Number of Columns";
    public static final String NUMBER_OF_LAYERS = "Number of Layers";
    public static final String NUMBER_OF_LINES = "Number of Lines";
    public static final String CACHE_CHUNK_SIZE = "Cache Configuration - Chunk size";
    public static final String CACHE_SETS = "Cache Configuration - Sets";
    public static final String CACHE_LINES_PER_SET = "Cache Configuration - Lines";
    public static final String OBJECTIVE_CLOCK_CYCLES = "number of clock cycles";
    public static final String OBJECTIVE_INSTRUCTIONS_PER_CLOCK_CYCLE = "instruction per clock cycle IPC";
    // Objectives for HW
    public static final String OBJECTIVE_CLOCKS_PER_INSTRUCTION = "Clocks per instruction CPI";
    public static final String OBJECTIVE_HARDWARE_COMPLEXITY = "Hardware complexity";
    // Objectives for SW
    public static final String OBJECTIVE_CPRI = "clock cycles per reference instruction CPRI";
    public static final String OBJECTIVE_OMSPRI = "optimization milli-seconds per reference instructions OmsPRI";
    public static final String OBJECTIVE_OPTIMIZATION_TIME = "Optimization time (ms)";

    public GapProblem(int numberOfVariables, int numberOfObjectives) {
        numberOfObjectives(numberOfObjectives);
        numberOfConstraints(numberOfConstraints);

        List<Double> lowerLimit = new ArrayList<>(numberOfVariables);
        List<Double> upperLimit = new ArrayList<>(numberOfVariables);

        // <parameter name="n_lines" description="param1" type="integer" min="4" max="32"/>
        lowerLimit.add(0,4.0);
        upperLimit.add(0,32.99);

        // <parameter name="n_columns" description="param2" type="integer" min="4" max="32"/>
        lowerLimit.add(1,4.0);
        upperLimit.add(1,32.99);

        // <parameter name="n_layers" description="param3" type="exp2" min="1" max="64"/>
        lowerLimit.add(2,0.0);
        upperLimit.add(2,6.99);

        // <parameter name="c_chunk" description="c_chunk" type="exp2" min="1" max="32"/>
        lowerLimit.add(3,0.0);
        upperLimit.add(3,5.99);

        // <parameter name="c_sets" description="c_sets" type="exp2" min="32" max="8192"/>
        lowerLimit.add(4,5.0);
        upperLimit.add(4,13.99);

        // <parameter name="c_lines" description="c_lines" type="exp2" min="1" max="128"/>
        lowerLimit.add(5,0.0);
        upperLimit.add(5,7.99);

        variableBounds(lowerLimit, upperLimit);
    }

    public GapProblem() {
        this(2, 2);
    }

    @Override
    public DoubleSolution evaluate(DoubleSolution solution) {
        String commandLineToExecute = "";
        double[] result;
        try {
             commandLineToExecute = this.getCommandLine(solution);
        } catch (IOException e) {
            throw new RuntimeException(e);
        }
    
        // Create Command Line
        System.out.println(commandLineToExecute);
        runCommandLine();
    
        //Wait for command line to finish
        try {
            Thread.sleep(1000);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
    
        // ReadResults
        // Read results from the simulator output
        //result = getResults();
    
        // Set objectives from the read results
        Random rand = new Random();
    
        //solution.objectives()[0] = result[0];
        //solution.objectives()[1] = result[1];
    
        return solution;
    }

    private String getCommandLine(DoubleSolution solution) throws IOException {
         String arguments = "";
        for (int i= 0; i<solution.variables().size(); i++) {
            arguments += " " + getActualValueForParameter(solution.variables().get(i), i);
        }

        return getMySimulator() + " " + arguments;
    }

    private String getMySimulator() throws IOException {
        var simulator_id = (int) ((Math.random() * 900) + 100);

        String my_sim = simulator_executable.replace(".exe", simulator_id + ".exe");

        if (!(new File(my_sim)).canRead()) {
            Files.copy(new File(simulator_executable).toPath(), new File(my_sim).toPath(), StandardCopyOption.REPLACE_EXISTING);
        }

        return my_sim;
    }

    private int getActualValueForParameter(double value, int index) {
        var integerValue = (int)value;
        if (index < 2) {
            return integerValue;
        }
        return (int)(Math.pow(2, integerValue));
    }

    private void runCommandLine() {
        String configFilePath = "configs/designSpace/config.xml";
        String benchmarksPath = "C:/Users/Ana/Downloads/JMetalSP-dissertation-NSGAII-SMPSO-Radu/JMetalSP-dissertation-NSGAII-SMPSO/simulator/gap_dump_1717571900584_default-mibench-netw-dijkstra";
        String simulatorPath = "C:/Users/Ana/Downloads/JMetalSP-dissertation-NSGAII-SMPSO-Radu/JMetalSP-dissertation-NSGAII-SMPSO/simulator/SimALU.exe";

        try {
            // Load the config.xml file
            File configFile = new File(configFilePath);
            DocumentBuilderFactory dbFactory = DocumentBuilderFactory.newInstance();
            DocumentBuilder dBuilder = dbFactory.newDocumentBuilder();
            Document doc = dBuilder.parse(configFile);
            doc.getDocumentElement().normalize();

            // Extract the parameter values
            NodeList parameterList = doc.getElementsByTagName("parameter");
            StringBuilder commandLine = new StringBuilder(simulatorPath);
            StringBuilder benchBuilder = new StringBuilder(benchmarksPath);
            commandLine.append(" ").append(benchBuilder.toString());
            for (int i = 0; i < parameterList.getLength(); i++) {
                Node parameterNode = parameterList.item(i);
                if (parameterNode.getNodeType() == Node.ELEMENT_NODE) {
                    Element parameterElement = (Element) parameterNode;
                    //String parameterName = parameterElement.getAttribute("name");
                    String parameterValue = parameterElement.getAttribute("value");
                    commandLine.append(" -").append("").append(parameterValue);
                }
            }
            commandLine.append(" /lb");
            try {
                // Split the command line into an array of Strings
                String[] commandArray = commandLine.toString().split(" ");
                // Print the command line
                System.out.println("Command Line: " + String.join(" ", commandArray));
                // Specify the working directory
                File workingDirectory = new File("C:/Users/Ana/Downloads/JMetalSP-dissertation-NSGAII-SMPSO-Radu/JMetalSP-dissertation-NSGAII-SMPSO/simulator");

                // Run the simulator with the generated command line
                Process process = Runtime.getRuntime().exec(commandArray, null, workingDirectory);

                try (java.io.BufferedReader reader = new java.io.BufferedReader(
                    new java.io.InputStreamReader(process.getInputStream()))) {
                String line;
                while ((line = reader.readLine()) != null) {
                    System.out.println(line);
                }
            }
                
            } catch (IOException e) {
                e.printStackTrace();
            }

            // Handle the output or errors if needed

        } catch (ParserConfigurationException | SAXException | IOException e) {
            e.printStackTrace();
        }
    }

    //     public LinkedList<Objective> getResults() {
    //     File file = new File(this.simulator.getSimulatorOutputFile());

    //     // Check file it is dir or not...
    //     if (file.isDirectory()) {
    //         // Somebody specified a directory... browse through it.
    //         for (File item : file.listFiles()) {
    //             if (item.getName().endsWith("results.txt")) {
    //                 file = item;
    //                 break;
    //             }
    //         }
    //     }
    //     System.out.println("Found result file: " + file);

    //     // Add some more keys which are needed to calculate combined objectives
    //     this.results.put(NUMBER_OF_LINES, Double.NaN);
    //     this.results.put(NUMBER_OF_LAYERS, Double.NaN);
    //     this.results.put(NUMBER_OF_COLUMNS, Double.NaN);

    //     this.results.put(CACHE_CHUNK_SIZE, Double.NaN);
    //     this.results.put(CACHE_LINES_PER_SET, Double.NaN);
    //     this.results.put(CACHE_SETS, Double.NaN);

    //     this.results.put(OBJECTIVE_CLOCK_CYCLES, Double.NaN);
    //     this.results.put(OBJECTIVE_INSTRUCTIONS_PER_CLOCK_CYCLE, Double.NaN);

    //     this.results.put(OBJECTIVE_OPTIMIZATION_TIME, Double.NaN);

    //     // Process the file and find some objectives => can be found in this.results
    //     this.processFile(individual);

    //     // Remove them again and remember the values
    //     double lines = this.results.remove(NUMBER_OF_LINES);
    //     double layers = this.results.remove(NUMBER_OF_LAYERS);
    //     double columns = this.results.remove(NUMBER_OF_COLUMNS);

    //     double cache_chunk_size = this.results.remove(CACHE_CHUNK_SIZE);
    //     double cache_lines_per_set = this.results.remove(CACHE_LINES_PER_SET);
    //     double cache_sets = this.results.remove(CACHE_SETS);

    //     double clock_cycles = this.results.remove(OBJECTIVE_CLOCK_CYCLES);
    //     double ipc = this.results.remove(OBJECTIVE_INSTRUCTIONS_PER_CLOCK_CYCLE);
    //     double optimization_time = this.results.remove(OBJECTIVE_OPTIMIZATION_TIME);

    //     // The return object
    //     LinkedList<Objective> finalResults = new LinkedList<Objective>();

    //     // Go through all the objectives and copy them to the return-object finalResults
    //     try {
    //         for (Objective obj : this.currentObjectives) {
    //             String key = obj.getName();

    //             // First handle the complex objectives
    //             if (key.equals(OBJECTIVE_HARDWARE_COMPLEXITY)) {
    //                 double complexity;

    //                 complexity = this.getHardwareComplexity(lines, columns, layers, cache_chunk_size, cache_lines_per_set, cache_sets);

    //                 System.out.println("Calculated Value for Hardware complexity: " + complexity);
    //                 obj.setValue(complexity);
    //             } else if (key.equals(OBJECTIVE_CPRI)) {
    //                 System.out.println(
    //                         "Calculating CPRI: clock_cycles=" + clock_cycles
    //                         + ", getMyReferenceInstructionCount(individual)=" + getMyReferenceInstructionCount(individual));

    //                 double ripc = (double) clock_cycles / (double) getMyReferenceInstructionCount(individual);

    //                 obj.setValue(ripc);
    //             } else if (key.equals(OBJECTIVE_CLOCKS_PER_INSTRUCTION)) {
    //                 System.out.println("Calculating CPI: IPC=" + ipc);
    //                 double cpi = 1 / ipc;

    //                 obj.setValue(cpi);
    //             } else if (key.equals(OBJECTIVE_OMSPRI)) {
    //                 System.out.println("Au weh zwick II => OMSPRI");

    //                 int ref_insn_count = this.getMyReferenceInstructionCount(individual);
    //                 System.out.println("  ref_insn_count: " + ref_insn_count);
    //                 System.out.println("  optimization_time: " + optimization_time);

    //                 double ripos = (double) optimization_time / (double) ref_insn_count;

    //                 obj.setValue(ripos);
    //             } else {
    //                 // It is a default/simple objective corresponding to a single line
    //                 if (this.results.containsKey(key) && this.results.get(key) != null) {
    //                     obj.setValue(this.results.get(key));
    //                     System.out.println("Found value for " + key + ": " + this.results.get(key));
    //                 } else {
    //                     individual.markAsInfeasibleAndSetBadValuesForObjectives("Objective " + key + " cannot be found (not existent or null): " + this.results);
    //                     setWorstObjectives(finalResults);
    //                     break;
    //                 }
    //             }

    //             finalResults.add(obj);
    //             System.out.println("Final Results after adding " + obj.getValue() + " for " + obj.getName() + ": " + finalResults);
    //         }
    //     } catch (Exception ex) {
    //         System.out.println("Error while calculating Objective: " + ex.getMessage());
    //         individual.markAsInfeasibleAndSetBadValuesForObjectives("Error calculating objective: " + ex.getMessage());
    //         setWorstObjectives(finalResults);
    //     }

    //     // Check if one of the values if MAX, then set as infeasible
    //     for (Objective item : finalResults) {
    //         if (item.getValue() == Double.MAX_VALUE) {
    //             individual.markAsInfeasibleAndSetBadValuesForObjectives("one of the objectives is Double.MAX_VALUE: " + finalResults);
    //             setWorstObjectives(finalResults);
    //             break;
    //         }
    //     }

    //     // If infeasible, then set all values to max.
    //     if (!individual.isFeasible()) {
    //         // Set all the objectives to the max available value...
    //         System.out.println("Individual is infeasible - clear objectives.");
    //         setWorstObjectives(finalResults);
    //     }

    //     System.out.println("I calculated as results: " + finalResults);

    //     return finalResults;
    // }
}
