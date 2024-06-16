package jmetal.auto.irace.parameterfilegeneration;

import jmetal.auto.autoconfigurablealgorithm.AutoMOEAD;
import jmetal.auto.autoconfigurablealgorithm.AutoNSGAII;

/**
 * Program to generate the irace configuration file for class {@link AutoNSGAII}
 *
 * @author Antonio J. Nebro (ajnebro@uma.es)
 */
public class AutoMOEADIraceParameterFileGenerator {
  public static void main(String[] args) {
    IraceParameterFileGenerator parameterFileGenerator = new IraceParameterFileGenerator() ;
    parameterFileGenerator.generateConfigurationFile(new AutoMOEAD()) ;
  }
}