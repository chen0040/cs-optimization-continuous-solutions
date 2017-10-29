using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using ContinuousOptimization.ProblemModels;

namespace ContinuousOptimization.LocalSearch
{
    public class GradientDescent : SingleTrajectoryContinuousSolver
    {
        protected double mAlpha = 0.0001;


        public GradientDescent()
        {

        }

        public GradientDescent(double alpha)
        {
            mAlpha = alpha;
        }

        public double Alpha
        {
            get { return mAlpha; }
            set { mAlpha = value; }
        }

        public override ContinuousSolution Minimize(double[] x_0, CostEvaluationMethod evaluate, GradientEvaluationMethod calc_gradient, TerminationEvaluationMethod should_terminate, object constraints = null)
        {
            int dimension = x_0.Length;
            double[] x = (double[])x_0.Clone();
            double fx = evaluate(x, mLowerBounds, mUpperBounds, constraints);
            double[] Vfx = new double[dimension];

            ContinuousSolution best_solution = new ContinuousSolution(x, fx);

            double? improvement = null;
            int iteration = 0;
            while (!should_terminate(improvement, iteration))
            {
                calc_gradient(x, Vfx, mLowerBounds, mUpperBounds, constraints);

                for (int d = 0; d < dimension; ++d)
                {
                    x[d] = x[d] - mAlpha * Vfx[d];
                }

                fx = evaluate(x, mLowerBounds, mUpperBounds, constraints);

                if (best_solution.TryUpdateSolution(x, fx, out improvement))
                {
                    //Console.WriteLine("called");
                    OnSolutionUpdated(best_solution, iteration);
                }

                OnStepped(new ContinuousSolution(x, fx), iteration);
                iteration++;
            }

            return best_solution;
        }
    }
}
