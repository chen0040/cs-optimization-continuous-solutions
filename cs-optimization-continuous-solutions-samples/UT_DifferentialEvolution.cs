using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using ContinuousOptimization.Benchmarks;
using ContinuousOptimization.LocalSearch;
using ContinuousOptimization.MetaHeuristics;
using ContinuousOptimization.ProblemModels;
using static ContinuousOptimization.MetaHeuristics.EvolutionaryProgramming;

namespace ContinuousOptimization.UT
{
    public class UT_DifferentialEvolution
    {
       
        public static void RunMain(string[] args)
        {
            CostFunction_RosenbrockSaddle f = new CostFunction_RosenbrockSaddle();

            int maxIterations = 200;
            DifferentialEvolution s = new DifferentialEvolution(f);

            s.SolutionUpdated += (best_solution, step) =>
            {
                Console.WriteLine("Step {0}: Fitness = {1}", step, best_solution.Cost);
            };

            ContinuousSolution finalSolution = s.Minimize(f, maxIterations);
        }
    }
}
