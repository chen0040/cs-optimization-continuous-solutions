using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using ContinuousOptimization.ProblemModels;
using MathNet.Numerics.LinearAlgebra.Generic;
using MathNet.Numerics.LinearAlgebra.Double;

namespace ContinuousOptimization.LocalSearch
{
    public class BFGS : SingleTrajectoryContinuousSolver
    {
        public BFGS()
            : base()
        {

        }

        public override ContinuousSolution Minimize(double[] x_0, CostEvaluationMethod evaluate, GradientEvaluationMethod calc_gradient, TerminationEvaluationMethod should_terminate, object constraints = null)
        {
            int dimension = x_0.Length;

            double[] x = (double[])x_0.Clone();
            double[] x_prev = (double[])x.Clone();
            double fx = evaluate(x, mLowerBounds, mUpperBounds, constraints);

            ContinuousSolution best_solution = new ContinuousSolution(x, fx);

            double[] Vfx = new double[dimension];
            double[] Vfx_prev = new double[dimension];

            double[] y = new double[dimension];

            double[] p = new double[dimension];
            double[] s = new double[dimension];

            double[] x_out;
            double fx_out = 0;
            double alpha = 1;

            calc_gradient(x, Vfx, mLowerBounds, mUpperBounds, constraints);

            Matrix<double> B_matrix = SparseMatrix.Identity(dimension);

            ContinuousSolution solution = new ContinuousSolution(x, fx);

            int iteration = 0;
            double? improvement = null;
            while (!should_terminate(improvement, iteration))
            {

                Matrix<double> Vfx_matrix = new SparseMatrix(dimension, 1, 0);

                for (int d = 0; d < dimension; ++d)
                {
                    Vfx_matrix[d, 0] = -Vfx[d];
                }

                Matrix<double> p_matrix = B_matrix.Inverse().Multiply(Vfx_matrix);

                for (int d = 0; d < dimension; ++d)
                {
                    p[d] = p_matrix[d, 0];
                }

                fx = evaluate(x, mLowerBounds, mUpperBounds, constraints);
                //Console.WriteLine("before");
                SingleTrajectoryContinuousSolver.LineSearch(x, fx, p, out x_out, out fx_out, out alpha, evaluate, calc_gradient, mLowerBounds, mUpperBounds, constraints);
                //Console.WriteLine("after");
                if (best_solution.TryUpdateSolution(x, fx, out improvement))
                {
                    OnSolutionUpdated(best_solution, iteration);
                }

                for (int d = 0; d < dimension; ++d)
                {
                    s[d] = alpha * p[d];
                }
                Matrix<double> s_matrix = new SparseMatrix(dimension, 1, s);
                Matrix<double> s_matrix_t = s_matrix.Transpose();

                for (int d = 0; d < dimension; ++d)
                {
                    x_prev[d] = x[d];
                }

                for (int d = 0; d < dimension; ++d)
                {
                    Vfx_prev[d] = Vfx[d];
                }

                for (int d = 0; d < dimension; ++d)
                {
                    x[d] += alpha * p[d];
                }

                calc_gradient(x, Vfx, mLowerBounds, mUpperBounds, constraints);

                for (int d = 0; d < dimension; ++d)
                {
                    y[d] = Vfx[d] - Vfx_prev[d];
                }

                Matrix<double> y_matrix = new SparseMatrix(dimension, 1, y);
                Matrix<double> y_matrix_t = y_matrix.Transpose();

                Matrix<double> yts_matrix = y_matrix_t.Multiply(s_matrix);
                double yts = yts_matrix[0, 0];

                Matrix<double> sBs_matrix = s_matrix_t.Multiply(B_matrix).Multiply(s_matrix);
                double sBs = sBs_matrix[0, 0];

                B_matrix = B_matrix + y_matrix.Multiply(y_matrix_t).Divide(yts) - B_matrix.Multiply(s_matrix).Multiply(s_matrix_t).Multiply(B_matrix).Divide(sBs);

                OnStepped(new ContinuousSolution(x, fx), iteration);
                iteration++;
            }

            return best_solution;
        }






        public override string ToString()
        {
            return "BFGS";
        }
    }
}
