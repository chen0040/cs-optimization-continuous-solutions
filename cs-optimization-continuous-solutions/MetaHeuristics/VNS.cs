using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;

namespace ContinuousOptimization.MetaHeuristics
{
    /// <summary>
    /// VNS implements Variable Neighborhood Search
    /// </summary>
    public class VNS : SingleTrajectoryContinuousSolver
    {
        protected LocalSearchChain mNeighborhoods = new LocalSearchChain();

        public void AddNeighborhood(SingleTrajectoryContinuousSolver local_search, SingleTrajectoryContinuousSolver.TerminationEvaluationMethod termination_condition)
        {
            local_search.Stepped += (solution, iteration) =>
                {
                    OnStepped(solution, iteration);
                };
            local_search.SolutionUpdated += (solution, iteration) =>
                {
                    OnSolutionUpdated(solution, iteration);
                };
            mNeighborhoods.Add(local_search, termination_condition);
        }

        public override ContinuousSolution Minimize(double[] x_0, CostEvaluationMethod evaluate, GradientEvaluationMethod calc_grad, TerminationEvaluationMethod should_terminate, object constraints = null)
        {
            double? improvement = null;
            int iteration = 0;

            if (mNeighborhoods.Count == 0)
            {
                throw new ArgumentNullException();
            }

            mNeighborhoods.LowerBounds = mLowerBounds;
            mNeighborhoods.UpperBounds = mUpperBounds;

            double[] x = (double[])x_0.Clone();
            double fx = evaluate(x, mLowerBounds, mUpperBounds, constraints);

            ContinuousSolution best_solution = new ContinuousSolution(x, fx);

            int neighborhood_count = mNeighborhoods.Count;

            ContinuousSolution current_best = null;

            while (!should_terminate(improvement, iteration))
            {
                for (int l = 0; l < neighborhood_count; ++l)
                {
                    SingleTrajectoryContinuousSolver local_search = mNeighborhoods.GetLocalSearchAt(l);
                    SingleTrajectoryContinuousSolver.TerminationEvaluationMethod termination_condition = mNeighborhoods.GetTerminationConditionAt(l);

                    current_best = local_search.Minimize(x, evaluate, calc_grad, termination_condition, constraints);

                    x = current_best.Values;
                    fx = current_best.Cost;

                    if (best_solution.TryUpdateSolution(x, fx, out improvement))
                    {
                        OnSolutionUpdated(best_solution, iteration);
                    }

                    OnStepped(best_solution, iteration);
                    iteration++;
                }
            }

            return best_solution;

        }
    }
}
