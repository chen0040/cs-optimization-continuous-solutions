using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;


namespace ContinuousOptimization.MetaHeuristics
{
    public class IteratedLocalSearch : SingleTrajectoryContinuousSolver
    {
        public delegate double[] CreateRandomNeighborhoodMethod(double[] x, int index, object constraints);
        public CreateRandomNeighborhoodMethod mNeighborSolutionProvider;

        protected TerminationEvaluationMethod mLocalSearchTerminationCondition;

        protected SingleTrajectoryContinuousSolver mLocalSearch;
        public SingleTrajectoryContinuousSolver LocalSearch
        {
            get { return mLocalSearch; }
            set { mLocalSearch = value; }
        }



        protected double[] mMasks;
        public IteratedLocalSearch(double[] masks, SingleTrajectoryContinuousSolver local_search, TerminationEvaluationMethod local_search_termination_condition, CreateRandomNeighborhoodMethod generator = null)
        {
            mLocalSearch = local_search;
            mLocalSearchTerminationCondition = local_search_termination_condition;


            mMasks = (double[])masks.Clone();
            mNeighborSolutionProvider = generator;
            if (mNeighborSolutionProvider == null)
            {
                mNeighborSolutionProvider = (x, index, constraints) =>
                    {
                        double[] lower_bounds = null;
                        double[] upper_bounds = null;

                        if (mLowerBounds == null || mUpperBounds == null)
                        {
                            Tuple<double[], double[]> bounds = null;
                            if (constraints is Tuple<double[], double[]>)
                            {
                                bounds = constraints as Tuple<double[], double[]>;
                            }
                            else
                            {
                                throw new InvalidCastException();
                            }

                            lower_bounds = bounds.Item1;
                            upper_bounds = bounds.Item2;
                        }
                        else
                        {
                            lower_bounds = mLowerBounds;
                            upper_bounds = mUpperBounds;
                        }

                        if (lower_bounds.Length < x.Length)
                        {
                            throw new ArgumentOutOfRangeException();
                        }
                        if (upper_bounds.Length < x.Length)
                        {
                            throw new ArgumentOutOfRangeException();
                        }

                        double[] x_p = (double[])x.Clone();

                        for (int i = 0; i < mMasks.Length; ++i)
                        {
                            int mindex = (index + i) % x_p.Length;
                            double new_val = mMasks[i] > 0 ? x[(index + i)] + RandomEngine.Gauss(0, mMasks[i]) : x[(index + i) % x.Length];
                            new_val = System.Math.Max(lower_bounds[mindex], new_val);
                            new_val = System.Math.Min(upper_bounds[mindex], new_val);

                            x_p[mindex] = new_val;
                        }
                        return x_p;
                    };
            }
        }

        public double[] GetNeighbor(double[] x, int index, object constraints)
        {
            return mNeighborSolutionProvider(x, index, constraints);
        }

        public override ContinuousSolution Minimize(double[] x_0, CostEvaluationMethod evaluate, GradientEvaluationMethod calc_gradient, TerminationEvaluationMethod should_terminate, object constraints = null)
        {
            double? improvement = null;
            int iteration = 0;

            if (mLocalSearch == null)
            {
                throw new ArgumentNullException();
            }

            mLocalSearch.LowerBounds = mLowerBounds;
            mLocalSearch.UpperBounds = mUpperBounds;

            double[] x = (double[])x_0.Clone();

            double fx = evaluate(x, mLowerBounds, mUpperBounds, constraints);

            ContinuousSolution best_solution = new ContinuousSolution(x, fx);

            while (!should_terminate(improvement, iteration))
            {
                int r_index = (int)(RandomEngine.NextDouble() * x.Length);
                double[] x_pi = GetNeighbor(x, r_index, constraints);
                double fx_pi = evaluate(x_pi, mLowerBounds, mUpperBounds, constraints);

                ContinuousSolution x_pi_refined = mLocalSearch.Minimize(x_pi, evaluate, calc_gradient, mLocalSearchTerminationCondition, constraints);

                if (best_solution.TryUpdateSolution(x_pi_refined.Values, x_pi_refined.Cost, out improvement))
                {
                    OnSolutionUpdated(best_solution, iteration);
                }

                x = best_solution.Values;
                fx = best_solution.Cost;

                OnStepped(best_solution, iteration);
                iteration++;
            }

            return best_solution;
        }
    }
}
