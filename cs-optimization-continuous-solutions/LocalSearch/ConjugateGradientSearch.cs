using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using ContinuousOptimization.ProblemModels;

namespace ContinuousOptimization.LocalSearch
{
    /// <summary>
    /// Nonlinear conjugate gradient search
    /// </summary>
    public class ConjugateGradientSearch : SingleTrajectoryContinuousSolver
    {
        public enum BetaFormula
        {
            FletcherReeves,
            PolakRebiere,
            HestenesStiefel
        }

        protected BetaFormula mBetaFormula = BetaFormula.FletcherReeves;

        public BetaFormula BetaFormulaUsed
        {
            get { return mBetaFormula; }
            set { mBetaFormula = value; }
        }

        public override ContinuousSolution Minimize(double[] x_0, CostEvaluationMethod evaluate, GradientEvaluationMethod calc_gradient, TerminationEvaluationMethod should_terminate, object constraints = null)
        {
            ContinuousSolution best_solution = new ContinuousSolution();

            int dimension = x_0.Length;

            double[] x = (double[])x_0.Clone();
            double fx = evaluate(x, mLowerBounds, mUpperBounds, constraints);

            double[] Vfx = new double[dimension];
            calc_gradient(x, Vfx, mLowerBounds, mUpperBounds, constraints);

            double[] deltaX = new double[dimension];
            double[] deltaX_prev = new double[dimension];


            for (int d = 0; d < dimension; ++d)
            {
                deltaX[d] = -Vfx[d];
            }

            double alpha;
            double[] x_next;
            double fx_next;
            LineSearch(x, fx, deltaX, out x_next, out fx_next, out alpha, evaluate, calc_gradient, mLowerBounds, mUpperBounds, constraints);

            double beta = 0;

            double[] s = new double[dimension];

            for (int d = 0; d < dimension; ++d)
            {
                s[d] = deltaX[d];
            }

            double? improvement = null;
            int iteration = 0;
            while (!should_terminate(improvement, iteration))
            {

                for (int d = 0; d < dimension; ++d)
                {
                    deltaX_prev[d] = deltaX[d];
                    x[d] = x_next[d];
                }

                calc_gradient(x, Vfx, mLowerBounds, mUpperBounds, constraints);
                for (int d = 0; d < dimension; ++d)
                {
                    deltaX[d] = -Vfx[d];
                }

                beta = ComputeBeta(deltaX, deltaX_prev, s);

                for (int d = 0; d < dimension; ++d)
                {
                    s[d] = deltaX[d] + beta * s[d];
                }

                LineSearch(x, fx, s, out x_next, out fx_next, out alpha, evaluate, calc_gradient, mLowerBounds, mUpperBounds, constraints);

                if (best_solution.TryUpdateSolution(x_next, fx_next, out improvement))
                {
                    OnSolutionUpdated(best_solution, iteration);
                }
                OnStepped(new ContinuousSolution(x_next, fx_next), iteration);
                iteration++;
            }

            return best_solution;
        }

        public double ComputeBeta(double[] deltaX, double[] deltaX_prev, double[] s)
        {
            double beta = 0;
            int dimension = deltaX.Length;
            if (mBetaFormula == BetaFormula.FletcherReeves)
            {
                double num1 = 0;
                for (int d = 0; d < dimension; ++d)
                {
                    num1 += Math.Pow(deltaX[d], 2);
                }
                double num2 = 0;
                for (int d = 0; d < dimension; ++d)
                {
                    num2 += Math.Pow(deltaX_prev[d], 2);
                }
                if (num2 != 0)
                {
                    beta = num1 / num2;
                }
            }
            else if (mBetaFormula == BetaFormula.HestenesStiefel)
            {
                double num1 = 0;
                for (int d = 0; d < dimension; ++d)
                {
                    num1 += deltaX[d] * (deltaX[d] - deltaX_prev[d]);
                }
                double num2 = 0;
                for (int d = 0; d < dimension; ++d)
                {
                    num2 += Math.Pow(deltaX_prev[d], 2);
                }
                if (num2 != 0)
                {
                    beta = num1 / num2;
                }
            }
            else if (mBetaFormula == BetaFormula.PolakRebiere)
            {
                double num1 = 0;
                for (int d = 0; d < dimension; ++d)
                {
                    num1 += deltaX[d] * (deltaX[d] - deltaX_prev[d]);
                }
                double num2 = 0;
                for (int d = 0; d < dimension; ++d)
                {
                    num2 += s[d] * (deltaX[d] - deltaX_prev[d]);
                }
                if (num2 != 0)
                {
                    beta = num1 / num2;
                }
            }
            return beta;
        }
    }
}
