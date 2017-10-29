using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using ContinuousOptimization.ProblemModels;

namespace ContinuousOptimization.LocalSearch
{
    public class SLOP : SingleTrajectoryContinuousSolver
    {
        protected double mDeltaX = 1;
        public double DeltaX
        {
            get { return mDeltaX; }
            set { mDeltaX = value; }
        }

        public override ContinuousSolution Minimize(double[] x_0, CostEvaluationMethod evaluate, GradientEvaluationMethod calc_gradient, TerminationEvaluationMethod should_terminate, object constraints = null)
        {
            ContinuousSolution best_solution = new ContinuousSolution();

            int dimension = x_0.Length;
            double[] x = (double[])x_0.Clone();
            double fx = evaluate(x, mLowerBounds, mUpperBounds, constraints);

            double? improvement = null;
            int iteration = 0;
            while (!should_terminate(improvement, iteration))
            {
                double[] fx_candidate = new double[dimension];
                double[] h = new double[dimension];
                for (int d = 0; d < dimension; ++d)
                {
                    x[d] += mDeltaX;
                    fx_candidate[d] = evaluate(x, mLowerBounds, mUpperBounds, constraints);
                    h[d] = fx - fx_candidate[d];
                    x[d] -= mDeltaX;
                }

                int max_d = -1;
                double h_max = 0;
                for (int d = 0; d < dimension; ++d)
                {
                    if (h_max < h[d])
                    {
                        h_max = h[d];
                        max_d = d;
                    }
                }

                if (max_d != -1)
                {
                    fx = fx_candidate[max_d];
                    x[max_d] += mDeltaX;
                }
                else
                {
                    if (mDeltaX > ZERO)
                    {
                        mDeltaX *= (-BETA);
                    }
                    else
                    {
                        break;
                    }
                }

                if (best_solution.TryUpdateSolution(x, fx, out improvement))
                {
                    OnSolutionUpdated(best_solution, iteration);
                }
                OnStepped(new ContinuousSolution(x, fx), iteration);
                iteration++;
            }

            return best_solution;
        }
    }
}
