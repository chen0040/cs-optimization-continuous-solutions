using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;

namespace ContinuousOptimization.MetaHeuristics
{
    /// <summary>
    /// GRASP implements Greedy Randomized Adaptive Search Program
    /// </summary>
    public class GRASP : MultiTrajectoryContinuousSolver
    {
        protected TerminationEvaluationMethod mLocalSearchShouldTerminate;

        public delegate double[] GreedyConstructSolutionMethod(CostEvaluationMethod evaluate, object constraints, int problem_size);
        public GreedyConstructSolutionMethod mGreedySolutionConstructor;

        protected SingleTrajectoryContinuousSolver mLocalSearch;
        public SingleTrajectoryContinuousSolver LocalSearch
        {
            get { return mLocalSearch; }
            set { mLocalSearch = value; }
        }

        protected int mDimension;
        protected int[] mMasks;
        public GRASP(int[] masks, int dimension, TerminationEvaluationMethod local_search_should_terminate, GreedyConstructSolutionMethod solution_constructor)
        {
            mDimension = dimension;

            mLocalSearchShouldTerminate = local_search_should_terminate;

            mMasks = (int[])masks.Clone();
            mGreedySolutionConstructor = solution_constructor;
            if (mGreedySolutionConstructor == null)
            {
                mGreedySolutionConstructor = (evaluate, constraints, problem_size) =>
                    {
                        double[] solution = new double[problem_size];
                        for (int i = 0; i < solution.Length; ++i)
                        {
                            solution[i] = -1;
                        }
                        for (int i = 0; i < problem_size; ++i)
                        {
                            int best_feature_value = -1;
                            double min_cost = double.MaxValue;
                            for (int bit_value = 0; bit_value < 2; ++bit_value)
                            {
                                solution[i] = bit_value;

                                double cost = evaluate(solution, mLowerBounds, mUpperBounds, constraints); //evaluate partial solution
                                if (min_cost > cost)
                                {
                                    min_cost = cost;
                                    best_feature_value = bit_value;
                                }
                            }
                        }
                        return solution;
                    };
            }
        }


        protected virtual double[] GreedyConstructRandomSolution(CostEvaluationMethod evaluate, object constraints, int problem_size)
        {
            return mGreedySolutionConstructor(evaluate, constraints, problem_size);
        }

        public override ContinuousSolution Minimize(CostEvaluationMethod evaluate, GradientEvaluationMethod calc_grad, TerminationEvaluationMethod should_terminate, object constraints = null)
        {
            double? improvement = null;
            int iteration = 0;

            if (mLocalSearch == null)
            {
                throw new ArgumentNullException();
            }

            mLocalSearch.LowerBounds = mLowerBounds;
            mLocalSearch.UpperBounds = mUpperBounds;

            int problem_size = mDimension;
            ContinuousSolution best_solution = new ContinuousSolution();

            while (!should_terminate(improvement, iteration))
            {
                double[] x = GreedyConstructRandomSolution(evaluate, constraints, problem_size);

                ContinuousSolution x_pi_refined = mLocalSearch.Minimize(x, evaluate, calc_grad, mLocalSearchShouldTerminate, constraints);

                if (best_solution.TryUpdateSolution(x_pi_refined.Values, x_pi_refined.Cost, out improvement))
                {
                    OnSolutionUpdated(best_solution, iteration);
                }

                OnStepped(best_solution, iteration);
                iteration++;
            }

            return best_solution;
        }
    }
}
