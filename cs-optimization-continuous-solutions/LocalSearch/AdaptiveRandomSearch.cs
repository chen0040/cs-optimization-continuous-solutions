using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;

namespace ContinuousOptimization.LocalSearch
{
    public class AdaptiveRandomSearch : SingleTrajectoryContinuousSolver
    {
        public delegate double[] CreateNeighborMethod(double[] x, double step_size, object constraints);
        protected CreateNeighborMethod mSolutionGenerator;
        protected int mSearchSpaceSize;


        protected double mStepSizeFactor_Large = 3.0;
        protected double mStepSizeFactor_Small = 1.5;

        protected int mStepSizeIterationFactor = 10;

        protected int mMaxNoChangeIterationCount = 30;

        public double StepSizeFactor_Large
        {
            get { return mStepSizeFactor_Large; }
            set { mStepSizeFactor_Large = value; }
        }

        public double StepSizeFactor_Small
        {
            get { return mStepSizeFactor_Small; }
            set { mStepSizeFactor_Small = value; }
        }

        public int StepSizeIterationFactor
        {
            get { return mStepSizeIterationFactor; }
            set { mStepSizeIterationFactor = value; }
        }

        public int MaxNoChangeIterationCount
        {
            get { return mMaxNoChangeIterationCount; }
            set { mMaxNoChangeIterationCount = value; }
        }

        public double CalcStepSizeLarge(double current_step_size, int iteration)
        {
            if (iteration % mStepSizeIterationFactor == 0)
            {
                return current_step_size * mStepSizeFactor_Large;
            }
            else
            {
                return current_step_size * mStepSizeFactor_Small;
            }
        }

        public AdaptiveRandomSearch(int search_space_size, CreateNeighborMethod generator)
        {
            mSearchSpaceSize = search_space_size;
            mSolutionGenerator = generator;
            if (mSolutionGenerator == null)
            {
                mSolutionGenerator = (x, step_size, constraints) =>
                      {
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

                          if (lower_bounds.Length < x.Length)
                          {
                              throw new ArgumentOutOfRangeException();
                          }
                          if (upper_bounds.Length < x.Length)
                          {
                              throw new ArgumentOutOfRangeException();
                          }

                          for (int i = 0; i < x.Length; ++i)
                          {
                              // must check boundary ...
                              double min = System.Math.Max(lower_bounds[i], x[i] - step_size);
                              double max = System.Math.Min(upper_bounds[i], x[i] + step_size);
                              double new_val = min + (max - min) * RandomEngine.NextDouble();
                              x[i] = new_val;
                          }

                          return x;
                      };
            }
        }

        public double[] TakeStep(CreateNeighborMethod generator, double[] x, double step_size, object constraints)
        {
            return generator(x, step_size, constraints);
        }

        public override ContinuousSolution Minimize(double[] x_0, CostEvaluationMethod evaluate, GradientEvaluationMethod calc_gradient, TerminationEvaluationMethod should_terminate, object constraints = null)
        {
            double? improvement = null;
            int iteration = 0;
            double[] x = (double[])x_0.Clone();

            double fx = evaluate(x, mLowerBounds, mUpperBounds, constraints);
            ContinuousSolution best_solution = new ContinuousSolution(x, fx);

            double current_step_size = 0;
            double step_size_large = 0;

            int no_change_count = 0;

            while (!should_terminate(improvement, iteration))
            {
                double[] x1 = TakeStep(mSolutionGenerator, x, current_step_size, constraints);
                double fx1 = evaluate(x1, mLowerBounds, mUpperBounds, constraints);

                step_size_large = CalcStepSizeLarge(current_step_size, iteration);

                double[] x2 = TakeStep(mSolutionGenerator, x, current_step_size, constraints);
                double fx2 = evaluate(x1, mLowerBounds, mUpperBounds, constraints);

                if (fx1 < fx || fx2 < fx)
                {
                    if (fx1 < fx2)
                    {
                        x = x1;
                        fx = fx1;
                        current_step_size = step_size_large;
                    }
                    else
                    {
                        x = x2;
                        fx = fx2;
                    }

                    no_change_count = 0;

                    if (best_solution.TryUpdateSolution(x, fx, out improvement))
                    {
                        OnSolutionUpdated(best_solution, iteration);
                    }
                }
                else
                {
                    no_change_count++;

                    if (no_change_count > mMaxNoChangeIterationCount)
                    {
                        no_change_count = 0;
                        current_step_size = current_step_size / mStepSizeFactor_Small;
                    }
                }

                OnStepped(best_solution, iteration);
                iteration++;
            }

            return best_solution;
        }
    }
}
