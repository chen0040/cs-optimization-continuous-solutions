using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;


namespace ContinuousOptimization.MetaHeuristics
{
    public class StochasticHillClimber : SingleTrajectoryContinuousSolver
    {
        public delegate double[] CreateRandomNeighborhoodMethod(double[] x, int index, object constraints);
        public CreateRandomNeighborhoodMethod mSolutionGenerator;

        protected double[] mMasks;
        public StochasticHillClimber(double[] masks, CreateRandomNeighborhoodMethod generator = null)
        {
            mMasks = (double[])masks.Clone();
            mSolutionGenerator = generator;
            if (mSolutionGenerator == null)
            {
                mSolutionGenerator = (x, index, constraints) =>
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
            return mSolutionGenerator(x, index, constraints);
        }

        public override ContinuousSolution Minimize(double[] x_0, CostEvaluationMethod evaluate, GradientEvaluationMethod calc_gradient, TerminationEvaluationMethod should_terminate, object constraints = null)
        {
            double? improvement = null;
            int iteration = 0;

            double fx_0 = evaluate(x_0, mLowerBounds, mUpperBounds, constraints);
            ContinuousSolution best_solution = new ContinuousSolution(x_0, fx_0);

            int dimension = x_0.Length;

            while (!should_terminate(improvement, iteration))
            {
                double[] best_x_in_neighborhood = null;
                double best_x_in_neighborhood_fx = double.MaxValue;
                for (int i = 0; i < dimension; ++i)
                {
                    double[] x_pi = GetNeighbor(best_solution.Values, i, constraints);
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
