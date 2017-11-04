using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using ContinuousOptimization.ProblemModels;

namespace ContinuousOptimization.MetaHeuristics
{
    public class DifferentialEvolution : MultiTrajectoryContinuousSolver
    {
        public delegate double[] CreateSolutionMethod(int dimension, object constraints);
        protected CreateSolutionMethod mSolutionGenerator;
        protected int mDimension;
        protected double mWeightingFactor = 0.8;
        protected double mCrossoverRate = 0.9;
        protected int mPopSize;

        public int Dimension
        {
            get { return mDimension; }
            set { mDimension = value; }
        }

        public double WeightingFactor
        {
            get { return mWeightingFactor; }
            set { mWeightingFactor = value; }
        }

        public double CrossoverRate
        {
            get { return mCrossoverRate; }
            set { mCrossoverRate = value; }
        }

        public int PopSize
        {
            get { return mPopSize; }
            set { mPopSize = value; }
        }

        public DifferentialEvolution(int dimension, CreateSolutionMethod solution_generator)
        {
            mPopSize = dimension * 10;
        }

        public DifferentialEvolution(CostFunction f)
        {
            mSolutionGenerator = (dimension, constraints) =>
            {
                double[] x_0 = f.CreateRandomSolution();
                return x_0;
            };

            mPopSize = f.DimensionCount * 10;
            mDimension = f.DimensionCount;

            mUpperBounds = f.UpperBounds;
            mLowerBounds = f.LowerBounds;

        }

        protected ContinuousSolution Reproduce(ContinuousSolution[] pop, ContinuousSolution p0, double[] lower_bounds, double[] upper_bounds)
        {
            ContinuousSolution[] parents = SelectParents(pop, p0);
            ContinuousSolution p1 = parents[0];
            ContinuousSolution p2 = parents[1];
            ContinuousSolution p3 = parents[2];

            int cut_point = RandomEngine.NextInt(mDimension);
            ContinuousSolution child = new ContinuousSolution(mDimension);
            for (int i = 0; i < mDimension; ++i)
            {
                child[i] = p0[i];
                if (i == cut_point || RandomEngine.NextDouble() < mCrossoverRate)
                {
                    child[i] = p1[i] + mWeightingFactor * (p2[i] - p3[i]);
                    child[i] = System.Math.Max(lower_bounds[i], child[i]);
                    child[i] = System.Math.Min(upper_bounds[i], child[i]);
                }
            }

            return child;
        }

        protected ContinuousSolution[] SelectParents(ContinuousSolution[] pop, ContinuousSolution p0)
        {
            ContinuousSolution p1, p2, p3;
            do
            {
                p1 = pop[RandomEngine.NextInt(pop.Length)];
            } while (p1 == p0);
            do
            {
                p2 = pop[RandomEngine.NextInt(pop.Length)];
            } while (p2 == p1 || p2 == p0);
            do
            {
                p3 = pop[RandomEngine.NextInt(pop.Length)];
            } while (p3 == p2 || p3 == p1 || p3 == p0);

            return new ContinuousSolution[] { p1, p2, p3 };
        }

        public override ContinuousSolution Minimize(CostEvaluationMethod evaluate, GradientEvaluationMethod calc_gradient, TerminationEvaluationMethod should_terminate, object constraints = null)
        {
            double? improvement = null;
            int iteration = 0;

            ContinuousSolution[] pop = new ContinuousSolution[mPopSize];
            ContinuousSolution[] children = new ContinuousSolution[mPopSize];

            double[] lower_bounds = null;
            double[] upper_bounds = null;

            if (mLowerBounds == null || mUpperBounds == null)
            {
                if (constraints is Tuple<double[], double[]>)
                {
                    Tuple<double[], double[]> bounds = constraints as Tuple<double[], double[]>;
                    lower_bounds = bounds.Item1;
                    upper_bounds = bounds.Item2;
                }
                else
                {
                    throw new InvalidCastException();
                }
            }
            else
            {
                lower_bounds = mLowerBounds;
                upper_bounds = mUpperBounds;
            }

            if (lower_bounds.Length < mDimension)
            {
                throw new IndexOutOfRangeException();
            }
            if (upper_bounds.Length < mDimension)
            {
                throw new IndexOutOfRangeException();
            }

            double[] best_x0 = null;
            double best_fx0 = double.MaxValue;
            for (int i = 0; i < mPopSize; ++i)
            {
                double[] x = mSolutionGenerator(mDimension, constraints);
                double fx = evaluate(x, mLowerBounds, mUpperBounds, constraints);
                ContinuousSolution s = new ContinuousSolution(x, fx);
                pop[i] = s;
                if (fx < best_fx0)
                {
                    best_fx0 = fx;
                    best_x0 = x;
                }
            }

            pop = pop.OrderBy(s => s.Cost).ToArray();
            ContinuousSolution best_solution = new ContinuousSolution(best_x0, best_fx0);

            while (!should_terminate(improvement, iteration))
            {
                for (int i = 0; i < mPopSize; ++i)
                {
                    ContinuousSolution child = Reproduce(pop, pop[i], lower_bounds, upper_bounds);
                    child.Cost = evaluate(child.Values, mLowerBounds, mUpperBounds, constraints);
                    children[i] = child;
                }

                for (int i = 0; i < mPopSize; ++i)
                {
                    if (pop[i].Cost > children[i].Cost)
                    {
                        pop[i] = children[i];
                    }
                }

                pop = pop.OrderBy(s => s.Cost).ToArray();

                if (best_solution.TryUpdateSolution(pop[0].Values, pop[0].Cost, out improvement))
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
