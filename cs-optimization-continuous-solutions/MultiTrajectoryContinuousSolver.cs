using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using ContinuousOptimization.ProblemModels;

namespace ContinuousOptimization
{
    public abstract class MultiTrajectoryContinuousSolver : ContinuousSolver
    {
        public virtual ContinuousSolution Minimize(CostFunction f, int max_iterations)
        {
            return Minimize((x, lower_bounds, upper_bounds, constraints) =>
            {
                return f.Evaluate(x);
            },
                (x, gradX, lower_bounds, upper_bounds, constraints) =>
                {
                    f.CalcGradient(x, gradX);
                },
                (improvement, iterations) =>
                {
                    return iterations > max_iterations;
                });
        }

        public abstract ContinuousSolution Minimize(CostEvaluationMethod evaluate, GradientEvaluationMethod calc_gradient, TerminationEvaluationMethod should_terminate, object constraints = null);

    }
}
