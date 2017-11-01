using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using ContinuousOptimization.Benchmarks;
using ContinuousOptimization.MetaHeuristics;
using ContinuousOptimization.ProblemModels;

namespace ContinuousOptimization.UT
{
    public class UT_NelderMead
    {
        public static void Run(CostFunction f, int max_iterations=200)
        {
            NelderMeadMethod s = new NelderMeadMethod();
        

            double[] x_0 = f.CreateRandomSolution();

            s.SolutionUpdated += (best_solution, step) =>
            {
                Console.WriteLine("Step {0}: Fitness = {1}", step, best_solution.Cost);
            };

            s.Minimize(x_0, f, max_iterations);
        }

        public static void Run_Sphere()
        {
            CostFunction_Sphere f = new CostFunction_Sphere();
            Run(f);
        }

        public static void RunRosenbrockSaddle(int max_iterations=1000)
        {
            CostFunction_RosenbrockSaddle f = new CostFunction_RosenbrockSaddle();
            Run(f,max_iterations);
        }
    }
}
