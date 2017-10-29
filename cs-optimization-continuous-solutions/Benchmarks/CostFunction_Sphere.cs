using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using ContinuousOptimization.ProblemModels;

namespace ContinuousOptimization.Benchmarks
{
    public class CostFunction_Sphere : CostFunction
    {
        public CostFunction_Sphere()
            : base(3, -5.12, 5.12)
        {

        }

        protected override void _CalcGradient(double[] solution, double[] grad)
        {
            int dimension=solution.Length;
            for (int i = 0; i < dimension; ++i)
            {
                grad[i] = 2 * solution[i];
            }
        }

        protected override double _Evaluate(double[] solution)
        {
            int dimension=solution.Length;
            double cost = 0;
            for (int i = 0; i < dimension; ++i)
            {
                double x = solution[i];
                cost += x * x;
            }
            return cost;
        }

       
    }
}
