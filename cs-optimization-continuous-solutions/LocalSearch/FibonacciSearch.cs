using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using ContinuousOptimization.ProblemModels;

namespace ContinuousOptimization.LocalSearch
{
    public class FibonacciSearch : SingleTrajectoryContinuousSolver
    {
        public void Minimize(double x_0, CostFunction f, int max_iterations, double lower_bound, double upper_bound)
        {

        }

        public override ContinuousSolution Minimize(double[] x_0, CostEvaluationMethod evaluate, GradientEvaluationMethod calc_gradient, TerminationEvaluationMethod should_terminate, object constraints = null)
        {
            throw new NotImplementedException();
        }
    }
}
