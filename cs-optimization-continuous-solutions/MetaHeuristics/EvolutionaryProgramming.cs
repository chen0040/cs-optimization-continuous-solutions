using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;

using ContinuousOptimization.MetaHeuristics.ExtensionMethods;
using ContinuousOptimization.ProblemModels;

namespace ContinuousOptimization.MetaHeuristics
{
    public class EvolutionaryProgramming : MultiTrajectoryContinuousSolver
    {
        public delegate double[] CreateSolutionMethod(int dimension, object constraints);
        protected CreateSolutionMethod mSolutionGenerator;
        protected int mPopSize;
        protected int mDimension = 100;
        protected int mBoutSize = 5;

        public EvolutionaryProgramming(CostFunction f, int popSize = 100, int boutSize = 5)
        {
            mSolutionGenerator = (dimension, constraints) =>
            {
                double[] x_0 = f.CreateRandomSolution();
                return x_0;
            };

            mPopSize = popSize;
            mDimension = f.DimensionCount;
            mBoutSize = boutSize;

            mUpperBounds = f.UpperBounds;
            mLowerBounds = f.LowerBounds;
            
        }

        public EvolutionaryProgramming(CreateSolutionMethod solution_generator, int dimension, int pop_size = 100, int bout_size = 5)
        {
            mPopSize = pop_size;
            mDimension = dimension;
            mBoutSize = bout_size;

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
            double[] child_stddevs = new double[stddevs.Length];
            for (int i = 0; i < stddevs.Length; ++i)
            {
                child_stddevs[i] = stddevs[i] + RandomEngine.Gauss(0, System.Math.Sqrt(System.Math.Abs(stddevs[i])));
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



        public override ContinuousSolution Minimize(CostEvaluationMethod evaluate, GradientEvaluationMethod calc_gradient, TerminationEvaluationMethod should_terminate, object constraints = null)
        {
            double? improvement = null;
            int iteration = 0;

            ContinuousSolution[] pop = new ContinuousSolution[mPopSize];

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

            for (int i = 0; i < mPopSize; ++i)
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

            ContinuousSolution[] children = new ContinuousSolution[mPopSize];
            ContinuousSolution[] generation = new ContinuousSolution[mPopSize * 2];

            while (!should_terminate(improvement, iteration))
            {
                for (int i = 0; i < mPopSize; ++i)
                {
                    children[i] = Mutate(pop[i], lower_bounds, upper_bounds, constraints);
                    children[i].Cost = evaluate(children[i].Values, mLowerBounds, mUpperBounds, constraints);
                }

                children = children.OrderBy(s => s.Cost).ToArray();
                if (best_solution.TryUpdateSolution(children[0].Values, children[0].Cost, out improvement))
                {
                    OnSolutionUpdated(best_solution, iteration);
                }


                for (int i = 0; i < mPopSize; ++i)
                {
                    generation[i] = pop[i];
                }
                for (int i = 0; i < mPopSize; ++i)
                {
                    generation[i + mPopSize] = children[i];
                }


                for (int i = 0; i < generation.Length; ++i)
                {
                    int wins = 0;
                    ContinuousSolution si = generation[i];
                    for (int j = 0; j < mBoutSize; ++j)
                    {
                        ContinuousSolution sj = generation[RandomEngine.NextInt(generation.Length)];
                        if (si.Cost < sj.Cost)
                        {
                            wins++;
                        }
                    }
                    si.SetWins(wins);
                }

                generation = generation.OrderByDescending(s => s.GetWins()).ToArray();

                for (int i = 0; i < mPopSize; ++i)
                {
                    pop[i] = generation[i];
                }


                OnStepped(best_solution, iteration);
                iteration++;
            }

            return best_solution;
        }
    }
}
