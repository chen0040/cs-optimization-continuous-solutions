using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using ContinuousOptimization.ProblemModels;

namespace ContinuousOptimization.LocalSearch
{
    public class GoldenSectionSearch : SingleTrajectoryContinuousSolver
    {
        public override ContinuousSolution Minimize(double[] x_0, CostEvaluationMethod evaluate, GradientEvaluationMethod calc_gradient, TerminationEvaluationMethod should_terminate, object constraints = null)
        {
            throw new NotImplementedException();
        }

        public void Minimize(double x_0, CostFunction f, int max_iterations, double lower_bound, double upper_bound)
        {
            double lambda = (Math.Sqrt(5) - 1) / 2;

            double x1 = lower_bound + (1 - lambda) * (upper_bound - lower_bound);
            double x2 = lower_bound + lambda * (upper_bound - lower_bound);

            double f_x1 = f.Evaluate(x1);
            double f_x2 = f.Evaluate(x2);

            double a = lower_bound;
            double b = upper_bound;

            double x = 0;
            double fx = 0;
            if (f_x1 < f_x2)
            {
                fx = f_x1;
                x = x1;
            }
            else
            {
                fx = f_x2;
                x = x2;
            }
            ContinuousSolution best_solution = new ContinuousSolution(new double[] { x }, fx);

            double? improvement = null;
            for (int k = 0; k < max_iterations; ++k)
            {
                if (Math.Abs(b - a) <= ZERO)
                {
                    break;
                }

                if (f_x1 < f_x2)
                {
                    b = x2;
                    x2 = x1;
                    x1 = a + (1 - lambda) * (b - a);

                    f_x1 = f.Evaluate(x1);
                    f_x2 = f.Evaluate(x2);

                    x = x1;
                    fx = f_x1;
                }
                else
                {
                    a = x1;
                    x1 = x2;
                    x2 = a + lambda * (b - a);

                    f_x1 = f.Evaluate(x1);
                    f_x2 = f.Evaluate(x2);

                    x = x2;
                    fx = f_x2;
                }


                if (f_x1 < f_x2)
                {
                    fx = f_x1;
                    x = x1;
                }
                else
                {
                    fx = f_x2;
                    x = x2;
                }
                if (best_solution.TryUpdateSolution(new double[] { x }, fx, out improvement))
                {
                    OnSolutionUpdated(best_solution, k);
                }
                OnStepped(new ContinuousSolution(new double[] { x }, fx), k);
            }



            if (best_solution.TryUpdateSolution(new double[] { x }, fx, out improvement))
            {
                OnSolutionUpdated(best_solution, max_iterations);
            }


        }
    }
}
