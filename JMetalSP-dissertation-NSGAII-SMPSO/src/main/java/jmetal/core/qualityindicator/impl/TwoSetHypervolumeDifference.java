package jmetal.core.qualityindicator.impl;

import jmetal.core.qualityindicator.QualityIndicator;
import jmetal.core.qualityindicator.impl.hypervolume.Hypervolume;
import jmetal.core.util.legacy.front.Front;
import jmetal.core.util.legacy.front.impl.ArrayFront;

public class TwoSetHypervolumeDifference extends QualityIndicator {

    //private Hypervolume hypervolumeCalculator;

   // public TwoSetHypervolumeDifference() {
    //    hypervolumeCalculator = new Hypervolume();
   // }

   // public TSHD(double[][] referenceFront) {
    //    super(referenceFront);
    //    hypervolumeCalculator = new Hypervolume(referenceFront);
   // }

    @Override
    public double compute(double[][] solutionSet) {
        throw new UnsupportedOperationException("TSHD requires two solution sets.");
    }

    /**
     * Computes the TSHD value for two sets of solutions.
     *
     * @param set1 the first set of solutions
     * @param set2 the second set of solutions
     * @return the TSHD value
     */
    public double compute(double[][] set1, double[][] set2) {
       // Front front1 = new ArrayFront(set1);
       // Front front2 = new ArrayFront(set2);

        //double hvSet1 = hypervolumeCalculator.evaluate(front1);
        //double hvSet2 = hypervolumeCalculator.evaluate(front2);

       // return Math.abs(hvSet1 - hvSet2);
        return set1[2][2];
    }

    @Override
    public boolean isTheLowerTheIndicatorValueTheBetter() {
        return true;
    }

    @Override
    public String name() {
        return "TSHD";
    }

    @Override
    public String description() {
        return "Two set hypervolume difference";
    }
}
