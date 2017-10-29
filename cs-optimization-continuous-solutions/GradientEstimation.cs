using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;

namespace ContinuousOptimization
{
    public class GradientEstimation
    {

        public static double small()
        {
            double one, two, z, tsmall;
            one = 1;
            two = 2;
            tsmall = one;
            do
            {
                tsmall = tsmall / two;
                fool(tsmall, one, out z);
            } while (z > 1);
            return tsmall * two * two;
        }

        public static void fool(double x, double y, out double z)
        {
            z = x * y + y;
            return;
        }

        public delegate double CostEvaluationMethod(double[] solution, object constraints = null);

        //calculate gradient
        public static void CalcGradient(double[] solution, double[] gradf, CostEvaluationMethod evaluate)
        {
            double xi, delta;
            double udelta = 0.0;

            int dimension_count = solution.Length;

            double rteps = Math.Sqrt(small());
            for (int i = 0; i < dimension_count; i++)
            {
                xi = solution[i];

                double tmp = 1.0e0;
                if (1.0e0 > Math.Abs(xi))
                {
                    tmp = 1.0e0;
                }
                else
                {
                    tmp = Math.Abs(xi);
                }
                if (udelta > rteps * tmp)
                {
                    delta = udelta;
                }
                else
                {
                    delta = rteps * tmp;
                }
                if (xi < 0.0) delta = -delta;

                solution[i] = xi + delta;
                double f2 = evaluate(solution);
                solution[i] = xi;
                double f1 = evaluate(solution);
                if (double.IsInfinity(f1) && double.IsInfinity(f2))
                {
                    gradf[i] = 0;
                }
                else
                {
                    gradf[i] = (f2 - f1) / delta;
                }
            }
            return;
        }

    }
}
