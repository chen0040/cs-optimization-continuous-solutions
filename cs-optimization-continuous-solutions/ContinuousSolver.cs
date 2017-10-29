using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;

namespace ContinuousOptimization
{
    public class ContinuousSolver : BaseSolver<double>
    {
        protected double[] mLowerBounds;
        protected double[] mUpperBounds;

        public double[] LowerBounds
        {
            get { return mLowerBounds; }
            set { mLowerBounds = value; }
        }

        public double[] UpperBounds
        {
            get { return mUpperBounds; }
            set { mUpperBounds = value; }
        }

        public delegate double CostEvaluationMethod(double[] x, double[] lower_bounds, double[] upper_bounds, object constraints);
        public delegate void GradientEvaluationMethod(double[] x, double[] gradx, double[] lower_bounds, double[] upper_bounds, object constraints);





    }
}
