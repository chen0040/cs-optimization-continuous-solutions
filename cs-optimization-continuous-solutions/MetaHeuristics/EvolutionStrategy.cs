using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;

using ContinuousOptimization.MetaHeuristics.ExtensionMethods;

namespace ContinuousOptimization.MetaHeuristics
{
    public class EvolutionStrategy : MultiTrajectoryContinuousSolver
    {
        public delegate double[] CreateSolutionMethod(int dimension, object constraints);
        protected CreateSolutionMethod mSolutionGenerator;
        protected int mPopSize_mu;
        protected int mOffspringSize_lambda;
        protected int mDimension = 100;
        protected int mSelectedParentCount_rho = 1;

        public EvolutionStrategy(CreateSolutionMethod solution_generator, int dimension, int mu = 30, int lambda = 20)
        {
            mPopSize_mu = mu;
            mOffspringSize_lambda = lambda;
            mDimension = dimension;

            mSolutionGenerator = solution_generator;
            if (mSolutionGenerator == null)
            {
                throw new NullReferenceException();
            }
        }

        protected ContinuousSolution Mutate(ContinuousSolution solution, double[] lower_bounds, double[] upper_bounds, object constraints)
        {
            ContinuousSolution child = MutateVector(solution, lower_bounds, upper_bounds, constraints);
            double[] child_strategy = MutateStrategy(solution.GetMutationStrategy());
            child.SetMutationStrategy(child_strategy);
            return child;
        }

        protected virtual double[] MutateStrategy(double[] stddevs)
        {
            double tau = 1.0 / System.Math.Sqrt(2.0 * stddevs.Length);
            double tau_p = 1.0 / System.Math.Sqrt(2.0 * System.Math.Sqrt(stddevs.Length));
            double[] child_stddevs = new double[stddevs.Length];
            for (int i = 0; i < stddevs.Length; ++i)
            {
                child_stddevs[i] = stddevs[i] * System.Math.Exp(RandomEngine.Gauss(0, tau_p) + RandomEngine.Gauss(0, tau));
            }
            return child_stddevs;
        }

        protected virtual ContinuousSolution MutateVector(ContinuousSolution solution, double[] lower_bounds, double[] upper_bounds, object constraints)
        {
            double[] x = new double[mDimension];
            double[] strategy = solution.GetMutationStrategy();
            for (int i = 0; i < mDimension; ++i)
            {
                x[i] = solution[i] + RandomEngine.Gauss(0, strategy[i]);
                x[i] = System.Math.Max(lower_bounds[i], x[i]);
                x[i] = System.Math.Min(upper_bounds[i], x[i]);
            }

            return new ContinuousSolution(x, double.MaxValue);
        }

        protected ContinuousSolution BinaryTournamentSelection(ContinuousSolution[] pop)
        {
            int index1 = RandomEngine.NextInt(pop.Length);
            int index2 = index1;
            do
            {
                index2 = RandomEngine.NextInt(pop.Length);
            } while (index1 == index2);

            if (pop[index1].Cost < pop[index2].Cost)
            {
                return pop[index1];
            }
            return pop[index2];
        }

        protected virtual ContinuousSolution Recombine(ContinuousSolution[] pop, double[] lower_bounds, double[] upper_bounds, object constraints)
        {
            ContinuousSolution[] parents = new ContinuousSolution[mSelectedParentCount_rho];
            for (int i = 0; i < mSelectedParentCount_rho; ++i)
            {
                parents[i] = BinaryTournamentSelection(pop);
            }

            HashSet<int> intersection_points = new HashSet<int>();
            for (int i = 0; i < mSelectedParentCount_rho - 1; ++i)
            {
                int index = RandomEngine.NextInt(mDimension);
                while (intersection_points.Contains(index))
                {
                    index = RandomEngine.NextInt(mDimension);
                }
                intersection_points.Add(index);
            }

            int[] intersect_pts = intersection_points.OrderBy(s => s).ToArray();

            int start_pt = 0;
            double[] x = new double[mDimension];
            double[] child_strategy = new double[mDimension];
            double[] parent_strategy = null;
            for (int i = 0; i < intersect_pts.Length; ++i)
            {
                int end_pt = intersect_pts[i];
                parent_strategy = parents[i].GetMutationStrategy();
                for (int j = start_pt; j < end_pt; ++j)
                {
                    x[j] = parents[i][j];
                    child_strategy[j] = parent_strategy[j];
                }
            }

            parent_strategy = parents[mSelectedParentCount_rho - 1].GetMutationStrategy();
            for (int i = start_pt; i < mDimension; ++i)
            {
                x[i] = parents[mSelectedParentCount_rho - 1][i];
                child_strategy[i] = parent_strategy[i];
            }

            ContinuousSolution child = new ContinuousSolution(x, double.MaxValue);
            child.SetMutationStrategy(child_strategy);

            return child;
        }

        public override ContinuousSolution Minimize(CostEvaluationMethod evaluate, GradientEvaluationMethod calc_gradient, TerminationEvaluationMethod should_terminate, object constraints = null)
        {
            double? improvement = null;
            int iteration = 0;

            ContinuousSolution[] pop = new ContinuousSolution[mPopSize_mu];

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

            double[,] init_strategy_bounds = new double[mDimension, 2];
            for (int j = 0; j < mDimension; ++j)
            {
                init_strategy_bounds[j, 0] = 0;
                init_strategy_bounds[j, 1] = (upper_bounds[j] - lower_bounds[j]) * 0.05;
            }


            ContinuousSolution best_solution = null;

            for (int i = 0; i < mPopSize_mu; ++i)
            {
                double[] x = mSolutionGenerator(mDimension, constraints);
                double fx = evaluate(x, mLowerBounds, mUpperBounds, constraints);
                ContinuousSolution s = new ContinuousSolution(x, fx);
                double[] strategy = new double[mDimension];
                for (int j = 0; j < mDimension; ++j)
                {
                    strategy[j] = init_strategy_bounds[j, 0] + (init_strategy_bounds[j, 1] - init_strategy_bounds[j, 0]) * RandomEngine.NextDouble();
                }
                s.SetMutationStrategy(strategy);
                pop[i] = s;
            }

            pop = pop.OrderBy(s => s.Cost).ToArray();

            best_solution = pop[0].Clone() as ContinuousSolution;

            ContinuousSolution[] children = new ContinuousSolution[mOffspringSize_lambda];
            ContinuousSolution[] generation = new ContinuousSolution[mPopSize_mu + mOffspringSize_lambda];

            while (!should_terminate(improvement, iteration))
            {
                for (int i = 0; i < mOffspringSize_lambda; ++i)
                {
                    if (mSelectedParentCount_rho == 1)
                    {
                        children[i] = Mutate(pop[i], lower_bounds, upper_bounds, constraints);
                    }
                    else
                    {
                        children[i] = Recombine(pop, lower_bounds, upper_bounds, constraints);
                        children[i] = Mutate(children[i], lower_bounds, upper_bounds, constraints);
                    }
                    children[i].Cost = evaluate(children[i].Values, mLowerBounds, mUpperBounds, constraints);
                }

                for (int i = 0; i < mPopSize_mu; ++i)
                {
                    generation[i] = pop[i];
                }
                for (int i = 0; i < mOffspringSize_lambda; ++i)
                {
                    generation[i + mPopSize_mu] = children[i];
                }

                generation = generation.OrderBy(s => s.Cost).ToArray();

                for (int i = 0; i < mPopSize_mu; ++i)
                {
                    pop[i] = generation[i];
                }

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
