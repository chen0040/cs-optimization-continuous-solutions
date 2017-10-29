using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;

namespace ContinuousOptimization.LocalSearch
{
    public class RandomSearch : SingleTrajectoryContinuousSolver
    {
        public delegate double[] CreateRandomSolutionMethod(object constraints);
        protected CreateRandomSolutionMethod mSolutionGenerator;
        protected int mSearchSpaceSize = -1;

        public RandomSearch(CreateRandomSolutionMethod generator, int search_space_size = -1)
        {
            mSearchSpaceSize = search_space_size;
            mSolutionGenerator = generator;
        }

        public double[] CreateRandomSolution(CreateRandomSolutionMethod generator, int index, object constraints)
        {
            return generator(constraints);
        }

        public override ContinuousSolution Minimize(double[] x_0, CostEvaluationMethod evaluate, GradientEvaluationMethod calc_gradient, TerminationEvaluationMethod should_terminate, object constraints = null)
        {
            double? improvement = null;
            int iteration = 0;

            double fx_0 = evaluate(x_0, mLowerBounds, mUpperBounds, constraints);
            ContinuousSolution best_solution = new ContinuousSolution(x_0, fx_0);

            if (mSearchSpaceSize == -1)
            {
                mSearchSpaceSize = x_0.Length;
            }

            while (!should_terminate(improvement, iteration))
            {
                double[] best_x_in_neighborhood = null;
                double best_x_in_neighborhood_fx = double.MaxValue;
                for (int i = 0; i < mSearchSpaceSize; ++i)
                {
                    double[] x_pi = CreateRandomSolution(mSolutionGenerator, i, constraints);
                    double fx_pi = evaluate(x_pi, mLowerBounds, mUpperBounds, constraints);

                    if (fx_pi < best_x_in_neighborhood_fx)
                    {
                        best_x_in_neighborhood = x_pi;
                        best_x_in_neighborhood_fx = fx_pi;
                    }
                }

                if (best_x_in_neighborhood != null)
                {
                    if (best_solution.TryUpdateSolution(best_x_in_neighborhood, best_x_in_neighborhood_fx, out improvement))
                    {
                        OnSolutionUpdated(best_solution, iteration);
                    }
                }

                OnStepped(best_solution, iteration);
                iteration++;
            }

            return best_solution;
        }
    }
}
