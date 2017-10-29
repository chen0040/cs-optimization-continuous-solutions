using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using ContinuousOptimization.ProblemModels;

namespace ContinuousOptimization.LocalSearch
{
    public class NewtonMethod : SingleTrajectoryContinuousSolver
    {
        public override ContinuousSolution Minimize(double[] x_0, CostEvaluationMethod evaluate, GradientEvaluationMethod calc_gradient, TerminationEvaluationMethod should_terminate, object constraints = null)
        {
            throw new NotImplementedException();
        }

        public void Minimize(double x_0, CostFunction f, int max_iterations)
        {
            double x = x_0;
            double fx = f.Evaluate(x);

            double newtonX = 0;

            double denominator = 0;

            ContinuousSolution best_solution = new ContinuousSolution(new double[] { x }, fx);

            double? improvement = null;
            for (int k = 0; k < max_iterations; ++k)
            {
                f.CalcGradient(x, out denominator);
                if (Math.Abs(denominator) < ZERO)
                {
                    break;
                }

                newtonX = x - fx / denominator;

                if (Math.Abs(newtonX - x) < SIGMA)
                {
                    break;
                }
                x = newtonX;

                fx = f.Evaluate(x);

                if (best_solution.TryUpdateSolution(new double[] { x }, fx, out improvement))
                {
                    OnSolutionUpdated(best_solution, k);
                }

                OnStepped(new ContinuousSolution(new double[] { x }, fx), k);
            }
        }


    }
}
