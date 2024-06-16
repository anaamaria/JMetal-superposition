package jmetal.core.qualityindicator.impl;

import java.io.FileNotFoundException;
import jmetal.core.qualityindicator.QualityIndicator;
import jmetal.core.util.VectorUtils;
import jmetal.core.util.errorchecking.Check;

/**
 * Set coverage metric
 *
 * @author Antonio J. Nebro
 * @version 1.0
 */
@SuppressWarnings("serial")
public class SetCoverage extends QualityIndicator {

  /**
   * Constructor
   */
  public SetCoverage() {
  }

  /**
   * Constructor
   *
   * @param referenceFront
   * @throws FileNotFoundException
   */
  public SetCoverage(double[][] referenceFront) {
    super(referenceFront) ;
  }

  @Override
  public double compute(double[][] front) {
    Check.notNull(front);

    return compute(front, referenceFront);
  }

  @Override
  public boolean isTheLowerTheIndicatorValueTheBetter() {
    return false;
  }

  /**
   * Calculates the set coverage of a front over a reference front
   * @param front
   * @param referenceFront
   * @return The value of the set coverage
   */
  public double compute(double[][] front, double[][] referenceFront) {
    Check.notNull(front);
    Check.notNull(referenceFront);

    double result ;
    int sum = 0 ;

    if (referenceFront.length == 0) {
      if (front.length ==0) {
        result = 0.0 ;
      } else {
        result = 1.0 ;
      }
    } else {
      for (double[] vector : referenceFront) {
        if (VectorUtils.isVectorDominatedByAFront(vector, front)) {
          sum++;
        }
      }
      result = (double)sum/referenceFront.length ;
    }
    return result ;
  }

  @Override public String name() {
    return "SC";
  }

  @Override public String description() {
    return "Set coverage";
  }
}