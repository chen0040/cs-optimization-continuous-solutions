using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using ContinuousOptimization.ProblemModels;

namespace ContinuousOptimization.MetaHeuristics
{
    public class NelderMeadMethod : SingleTrajectoryContinuousSolver
    {
        /// <summary>
        /// Reflection parameter
        /// </summary>
        protected double mAlpha = 1;
        public double Alpha
        {
            get { return mAlpha; }
            set { mAlpha = value; }
        }

        /// <summary>
        /// Expansion parameter
        /// </summary>
        protected double mGamma = 2;
        public double Gamma
        {
            get { return mGamma; }
            set { mGamma = value; }
        }

        /// <summary>
        /// Contraction parameter
        /// </summary>
        protected double mRho = -0.5;
        public double Rho
        {
            get { return mRho; }
            set { mRho = value; }
        }

        /// <summary>
        /// Reduction parameter
        /// </summary>
        protected double mSigma = 0.5;
        public double Sigma
        {
            get { return mSigma; }
            set { mSigma = value; }
        }

        public override ContinuousSolution Minimize(double[] x_0, CostEvaluationMethod evaluate, GradientEvaluationMethod calc_gradient, TerminationEvaluationMethod should_terminate, object constraints = null)
        {
            ContinuousSolution best_solution = new ContinuousSolution();

            int dimension = x_0.Length;
            int vertex_count = dimension + 1;

            double[] x = (double[])x_0.Clone();


            List<ContinuousSolution> solutions = new List<ContinuousSolution>();
            for (int i = 0; i < vertex_count; ++i)
            {
                double[] temp = (double[])x_0.Clone();
                if (i == vertex_count - 1)
                {
                    temp[0] -= 1.0;
                }
                else
                {
                    temp[i] += 1.0;
                }

                solutions.Add(new ContinuousSolution(temp, evaluate(temp, mLowerBounds, mUpperBounds, constraints)));
            }

            ContinuousSolution centroid = new ContinuousSolution();

            double? improvement = null;
            int iteration = 0;
            while (!should_terminate(improvement, iteration))
            {
                Sort(solutions);
                CalculateCentroid(solutions, centroid);

                double[] x_r;
                Reflect(solutions, centroid, mAlpha, out x_r);
                double fx_r = evaluate(x_r, mLowerBounds, mUpperBounds, constraints);
                if (fx_r >= solutions[0].Cost && fx_r < solutions[solutions.Count - 2].Cost)
                {
                    ContinuousSolution last_solution = solutions[solutions.Count - 1];
                    last_solution.Values = x_r;
                    last_solution.Cost = fx_r;
                }
                else if (fx_r < solutions[0].Cost)
                {
                    double[] x_e;
                    Expand(solutions, centroid, mGamma, out x_e);
                    double fx_e = evaluate(x_e, mLowerBounds, mUpperBounds, constraints);
                    if (fx_e < fx_r)
                    {
                        ContinuousSolution last_solution = solutions[solutions.Count - 1];
                        last_solution.Values = x_e;
                        last_solution.Cost = fx_e;
                    }
                    else
                    {
                        ContinuousSolution last_solution = solutions[solutions.Count - 1];
                        last_solution.Values = x_r;
                        last_solution.Cost = fx_r;
                    }
                }
                else
                {
                    double[] x_c;
                    Contract(solutions, centroid, mRho, out x_c);
                    double fx_c = evaluate(x_c, mLowerBounds, mUpperBounds, constraints);
                    if (fx_c < solutions[solutions.Count - 1].Cost)
                    {
                        ContinuousSolution last_solution = solutions[solutions.Count - 1];
                        last_solution.Values = x_c;
                        last_solution.Cost = fx_c;
                    }
                    else
                    {
                        Reduce(solutions, mSigma);
                        int solution_count = solutions.Count;
                        for (int i = 1; i < solution_count; ++i)
                        {
                            ContinuousSolution s = solutions[i];
                            s.Cost = evaluate(s.Values, mLowerBounds, mUpperBounds, constraints);
                        }
                    }
                }

                if (best_solution.TryUpdateSolution(solutions[0].Values, solutions[0].Cost, out improvement))
                {
                    OnSolutionUpdated(best_solution, iteration);
                }
                OnStepped(solutions[0], iteration);

                iteration++;
            }

            return best_solution;
        }

        public void Reduce(List<ContinuousSolution> solutions, double sigma)
        {
            int solution_count = solutions.Count;
            ContinuousSolution best_solution = solutions[0];
            double[] x_0 = best_solution.Values;
            int dimension = x_0.Length;
            for (int i = 1; i < solution_count; ++i)
            {
                ContinuousSolution s = solutions[i];
                double[] x = s.Values;
                for (int d = 0; d < dimension; ++d)
                {
                    x[d] = x_0[d] + sigma * (x[d] - x_0[d]);
                }
                s.Values = x;
            }
        }

        public void Contract(List<ContinuousSolution> solutions, ContinuousSolution centroid, double rho, out double[] x_c)
        {
            int dimension = centroid.Values.Length;
            double[] x_centroid = centroid.Values;

            int solution_count = solutions.Count - 1;
            ContinuousSolution last_solution = solutions[solution_count];
            double[] x_last = last_solution.Values;
            x_c = new double[dimension];
            for (int d = 0; d < dimension; ++d)
            {
                x_c[d] = x_centroid[d] + rho * (x_centroid[d] - x_last[d]);
            }
        }

        public void Expand(List<ContinuousSolution> solutions, ContinuousSolution centroid, double gamma, out double[] x_e)
        {
            int dimension = centroid.Values.Length;
            double[] x_centroid = centroid.Values;

            int solution_count = solutions.Count - 1;
            ContinuousSolution last_solution = solutions[solution_count];
            double[] x_last = last_solution.Values;
            x_e = new double[dimension];
            for (int d = 0; d < dimension; ++d)
            {
                x_e[d] = x_centroid[d] + gamma * (x_centroid[d] - x_last[d]);
            }
        }

        public void Reflect(List<ContinuousSolution> solutions, ContinuousSolution centroid, double alpha, out double[] x_r)
        {
            int dimension = centroid.Values.Length;
            double[] x_centroid = centroid.Values;

            int solution_count = solutions.Count - 1;
            ContinuousSolution last_solution = solutions[solution_count];
            double[] x_last = last_solution.Values;
            x_r = new double[dimension];
            for (int d = 0; d < dimension; ++d)
            {
                x_r[d] = x_centroid[d] + alpha * (x_centroid[d] - x_last[d]);
            }
        }

        public void Sort(List<ContinuousSolution> solutions)
        {
            solutions.Sort((s1, s2) =>
            {
                return s1.Cost.CompareTo(s2.Cost);
            });

            //for (int i = 0; i < solutions.Count; ++i)
            //{
            //    Console.WriteLine("Fitness: {0}", solutions[i].Fitness);
            //}
        }

        public void CalculateCentroid(List<ContinuousSolution> solutions, ContinuousSolution centroid)
        {
            int solution_count = solutions.Count - 1;
            int dimension = solutions[0].Values.Length;
            double[] x_centroid = new double[dimension];
            for (int d = 0; d < dimension; ++d)
            {
                x_centroid[d] = 0;
            }
            for (int i = 0; i < solution_count; ++i)
            {
                ContinuousSolution s = solutions[i];
                double[] x = s.Values;

                for (int d = 0; d < dimension; ++d)
                {
                    x_centroid[d] += x[d];
                }
            }
            for (int d = 0; d < dimension; ++d)
            {
                x_centroid[d] /= solution_count;
            }
            centroid.Values = x_centroid;
        }
    }
}
