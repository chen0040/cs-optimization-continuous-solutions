using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using ContinuousOptimization.ProblemModels;

namespace ContinuousOptimization
{
    public abstract class SingleTrajectoryContinuousSolver : ContinuousSolver
    {
        public SingleTrajectoryContinuousSolver()
        {

        }

        public virtual ContinuousSolution Minimize(double[] x_0, CostFunction f, int max_iterations)
        {
            return Minimize(x_0, (x, lower_bounds, upper_bounds, constraints) =>
                {
                    return f.Evaluate(x);
                },
                (x, gradX, lower_bounds, upper_bounds, constraints) =>
                {
                    f.CalcGradient(x, gradX);
                },
                (improvement, iterations) =>
                {
                    return iterations > max_iterations;
                });
        }


        public abstract ContinuousSolution Minimize(double[] x_0, CostEvaluationMethod evaluate, GradientEvaluationMethod calc_gradient, TerminationEvaluationMethod should_terminate, object constraints = null);

        protected static double SIGMA = 1.0E-4;
        protected static double BETA = 0.5;

        protected static double ZERO = 1.0E-10;

        public static bool LineSearch(double[] x_0, double fx_0, double[] direction, out double[] x_out, out double fx_out, out double alpha, CostEvaluationMethod evaluate, GradientEvaluationMethod calc_gradient, double[] lower_bounds, double[] upper_bounds, object constraints = null)
        {
            int dimension = x_0.Length;

            double[] Vfx = new double[dimension];
            x_out = new double[dimension];
            alpha = 1.0;
            fx_out = double.MaxValue;

            calc_gradient(x_0, Vfx, lower_bounds, upper_bounds, constraints);

            double direction_length = 0;
            for (int d = 0; d < dimension; ++d)
            {
                direction_length += direction[d] * direction[d];
            }
            direction_length = Math.Sqrt(direction_length);

            if (direction_length > 0)
            {
                for (int d = 0; d < dimension; ++d)
                {
                    direction[d] /= direction_length;
                }
            }

            double p = 0.0;
            for (int d = 0; d < dimension; ++d)
            {
                p += (direction[d] * Vfx[d]);
            }

            //Console.WriteLine("p: {0}", p);


            if (double.IsNaN(p))
            {
                return false;
            }

            if (p >= 0.0) // not in the descending direction return false;
            {
                return false;
            }


            for (int k = 0; ; ++k)
            {
                for (int d = 0; d < dimension; ++d)
                {
                    x_out[d] = x_0[d] + alpha * direction[d];
                }
                fx_out = evaluate(x_out, lower_bounds, upper_bounds, constraints);

                if (fx_out < fx_0 + SIGMA * alpha * p)
                {
                    return true;
                }
                else
                {
                    if (k == 0)
                    {
                        double enumerator = (p + fx_0 - fx_out);
                        if (enumerator == 0)
                        {
                            alpha = 0.5 * p / enumerator;
                        }
                        else
                        {
                            alpha = 0.5 * p;
                        }

                        //Console.WriteLine("alpha: {0}", alpha);
                    }
                    else
                    {
                        alpha *= BETA;
                    }
                }

                //Console.WriteLine("alpha: {0}", alpha);

                if (alpha < ZERO)
                {
                    if (fx_out > fx_0)
                    {
                        for (int d = 0; d < dimension; ++d)
                        {
                            x_out[d] = x_0[d];
                        }

                        fx_out = fx_0;
                        return true;
                    }
                    else
                    {
                        return true;
                    }
                }
            }
        }

        public static bool LineSearch(double[] x_0, double fx_0, double[] direction, out double[] x_out, out double fx_out, out double alpha, CostFunction f)
        {
            int dimension = x_0.Length;

            double[] Vfx = new double[dimension];
            x_out = new double[dimension];
            alpha = 1.0;
            fx_out = double.MaxValue;

            f.CalcGradient(x_0, Vfx);

            double direction_length = 0;
            for (int d = 0; d < dimension; ++d)
            {
                direction_length += direction[d] * direction[d];
            }
            direction_length = Math.Sqrt(direction_length);

            if (direction_length > 0)
            {
                for (int d = 0; d < dimension; ++d)
                {
                    direction[d] /= direction_length;
                }
            }

            double p = 0.0;
            for (int d = 0; d < dimension; ++d)
            {
                p += (direction[d] * Vfx[d]);
            }

            //Console.WriteLine("p: {0}", p);


            if (double.IsNaN(p))
            {
                return false;
            }

            if (p >= 0.0) // not in the descending direction return false;
            {
                return false;
            }


            for (int k = 0; ; ++k)
            {
                for (int d = 0; d < dimension; ++d)
                {
                    x_out[d] = x_0[d] + alpha * direction[d];
                }
                fx_out = f.Evaluate(x_out);

                if (fx_out < fx_0 + SIGMA * alpha * p)
                {
                    return true;
                }
                else
                {
                    if (k == 0)
                    {
                        double enumerator = (p + fx_0 - fx_out);
                        if (enumerator == 0)
                        {
                            alpha = 0.5 * p / enumerator;
                        }
                        else
                        {
                            alpha = 0.5 * p;
                        }

                        //Console.WriteLine("alpha: {0}", alpha);
                    }
                    else
                    {
                        alpha *= BETA;
                    }
                }

                //Console.WriteLine("alpha: {0}", alpha);

                if (alpha < ZERO)
                {
                    if (fx_out > fx_0)
                    {
                        for (int d = 0; d < dimension; ++d)
                        {
                            x_out[d] = x_0[d];
                        }

                        fx_out = fx_0;
                        return true;
                    }
                    else
                    {
                        return true;
                    }
                }
            }

        }

    }
}
