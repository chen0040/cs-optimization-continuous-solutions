using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using ContinuousOptimization.ProblemModels;

namespace ContinuousOptimization.LocalSearch
{
    public class PowellMethod : SingleTrajectoryContinuousSolver
    {
        public override ContinuousSolution Minimize(double[] x_0, CostEvaluationMethod evaluate, GradientEvaluationMethod calc_gradient, TerminationEvaluationMethod should_terminate, object constraints = null)
        {
            int dimension = x_0.Length;

            double[][] h = new double[dimension][];
            double[] lambda = new double[dimension];
            for (int i = 0; i < dimension; ++i)
            {
                h[i] = new double[dimension];
                for (int j = 0; j < dimension; ++j)
                {
                    h[i][j] = i == j ? 1.0 : 0.0;
                }
            }

            double[][] x = new double[dimension + 1][];
            double[] fx = new double[dimension + 1];

            for (int i = 0; i <= dimension; ++i)
            {
                x[i] = new double[dimension];
            }
            for (int d = 0; d < dimension; ++d)
            {
                x[0][d] = x_0[d];
                fx[0] = evaluate(x[0], mLowerBounds, mUpperBounds, constraints);
            }

            ContinuousSolution best_solution = new ContinuousSolution(x[0], fx[0]);

            double? improvement = null;
            int iteration = 0;
            while (!should_terminate(improvement, iteration))
            {
                for (int i = 1; i <= dimension; ++i)
                {
                    LineSearch(x[i - 1], fx[i - 1], h[i - 1], out x[i], out fx[i], out lambda[i - 1], evaluate, calc_gradient, mLowerBounds, mUpperBounds, constraints);
                    for (int j = 0; j < dimension - 1; ++j)
                    {
                        for (int d = 0; d < dimension; ++d)
                        {
                            h[j][d] = h[j + 1][d];
                        }
                    }
                    for (int d = 0; d < dimension; ++d)
                    {
                        h[dimension - 1][d] = x[dimension][d] - x[0][d];
                    }
                    LineSearch(x[0], fx[0], h[dimension - 1], out x[0], out fx[0], out lambda[dimension - 1], evaluate, calc_gradient, mLowerBounds, mUpperBounds, constraints);
                    if (best_solution.TryUpdateSolution(x[0], fx[0], out improvement))
                    {
                        OnSolutionUpdated(best_solution, iteration);
                    }
                }
                OnStepped(new ContinuousSolution(x[0], fx[0]), iteration);
                iteration++;
            }

            return best_solution;
        }
    }
}
