using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using ContinuousOptimization.Benchmarks;
using ContinuousOptimization.LocalSearch;
using ContinuousOptimization.ProblemModels;

namespace ContinuousOptimization.UT
{
    public class UT_ConjugateGradientSearch
    {
        public static void Run(CostFunction f, int max_iteration=500)
        {
            ConjugateGradientSearch s = new ConjugateGradientSearch();
            
            double[] x_0 = f.CreateRandomSolution();

            s.SolutionUpdated += (best_solution, step) =>
            {
                Console.WriteLine("Step {0}: Fitness = {1}", step, best_solution.Cost);
            };

            s.Minimize(x_0, f, max_iteration);
        }

        public static void Run_Sphere()
        {
            CostFunction_Sphere f = new CostFunction_Sphere();
            Run(f);
        }

        public static void RunRosenbrockSaddle(int max_iterations=500)
        {
            CostFunction_RosenbrockSaddle f = new CostFunction_RosenbrockSaddle();
            Run(f, max_iterations);
        }
    }
}
