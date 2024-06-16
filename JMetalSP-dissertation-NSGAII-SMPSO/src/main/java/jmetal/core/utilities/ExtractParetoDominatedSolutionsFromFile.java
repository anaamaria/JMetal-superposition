package jmetal.core.utilities;

import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Paths;
import jmetal.core.solution.pointsolution.PointSolution;
import jmetal.core.util.StoredSolutionsUtils;
import jmetal.core.util.archive.impl.NonDominatedSolutionListArchive;
import jmetal.core.util.errorchecking.Check;
import jmetal.core.util.errorchecking.JMetalException;
import jmetal.core.util.fileoutput.impl.DefaultFileOutputContext;

/**
 * This utility takes an input file and produces an output file containing only the non-dominated
 * solutions in the input file
 *
 * Each line of the file represents a solution. The format of each of these solutions is rather
 * flexible and the only requirement is that for an n-objective problem, at least the first n
 * columns must be numerical values representing these objectives values. By default, columns can be
 * separated by blank spaces, tabs, or commas (csv)
 *
 * The program receives three parameters: 1. the name of the file containing the data, 2. the
 * output file name which will contain the generated front, and 3. the number of objectives
 */
public class ExtractParetoDominatedSolutionsFromFile {

  public static void main(String[] args) throws IOException {
    Check.that(args.length == 3,
          "Wrong number of arguments: "
              + args.length
              + "\nThis program should be called with three arguments:"
              + "\nThe first argument is the name of the file containing the input solutions."
              + "\nThe second argument is the name of the file containing the computed output."
              + "\nThe third argument is the number of objectives of the problem whose front is to be extracted.");

    String inputFileName = args[0];
    String outputFileName = args[1];
    int numberOfObjectives = Integer.parseInt(args[2]);

    NonDominatedSolutionListArchive<PointSolution> archive =
        new NonDominatedSolutionListArchive<>();

    if (Files.isRegularFile(Paths.get(inputFileName))) {
      archive.addAll(StoredSolutionsUtils.readSolutionsFromFile(inputFileName, numberOfObjectives));
    } else {
      throw new JMetalException("Error opening file " + inputFileName);
    }

    StoredSolutionsUtils.writeToOutput(archive.solutions(), new DefaultFileOutputContext(outputFileName));
  }
}