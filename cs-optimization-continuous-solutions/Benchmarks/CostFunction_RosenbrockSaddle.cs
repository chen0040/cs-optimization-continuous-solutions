using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using ContinuousOptimization.ProblemModels;

namespace ContinuousOptimization.Benchmarks
{
    public class CostFunction_RosenbrockSaddle : CostFunction
    {
        public CostFunction_RosenbrockSaddle()
            : base(2, -2.048, 2.048)
        {

        }

        protected override void _CalcGradient(double[] solution, double[] grad)
        {
            double x0 = solution[0];
            double x1 = solution[1];
            grad[0] = 400 * (x0 * x0 - x1) * x0 - 2 * (1 - x0);
            grad[1] = -200 * (x0 * x0 - x1);
        }

        protected override double _Evaluate(double[] solution)
        {
            double x0 = solution[0];
            double x1 = solution[1];

            double cost =100 * Math.Pow(x0 * x0 - x1, 2) + Math.Pow(1 - x0, 2);
            return cost;
        }

    }
}
