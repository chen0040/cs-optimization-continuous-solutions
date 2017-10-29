using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;

namespace ContinuousOptimization.MetaHeuristics
{
    public class TabuSearch : StochasticHillClimber
    {
        protected Dictionary<int, int> mTabuList = new Dictionary<int, int>();
        protected int mTabuDuration;

        public TabuSearch(double[] masks, int tabu_duration)
            : base(masks)
        {
            mTabuDuration = tabu_duration;
        }

        protected virtual void TabuMove(int move_id)
        {
            mTabuList[move_id] = mTabuDuration;
        }

        protected virtual void LowerTabuList()
        {
            List<int> tabu_move_id_list = mTabuList.Keys.ToList();
            for (int i = 0; i < tabu_move_id_list.Count; ++i)
            {
                int tabu_move_id = tabu_move_id_list[i];
                mTabuList[tabu_move_id] -= 1;
                if (mTabuList[tabu_move_id] <= 0)
                {
                    mTabuList.Remove(tabu_move_id);
                }
            }
        }

        public virtual bool IsMoveTabu(int move_id)
        {
            if (mTabuList.ContainsKey(move_id))
            {
                return mTabuList[move_id] > 0;
            }
            return false;
        }

        public override ContinuousSolution Minimize(double[] x_0, CostEvaluationMethod evaluate, GradientEvaluationMethod calc_grad, TerminationEvaluationMethod should_terminate, object constraints = null)
        {
            double? improvement = null;
            int iteration = 0;

            double fx_0 = evaluate(x_0, mLowerBounds, mUpperBounds, constraints);
            ContinuousSolution best_solution = new ContinuousSolution(x_0, fx_0);

            int dimension = x_0.Length;

            while (!should_terminate(improvement, iteration))
            {
                double[] best_x_in_neighborhood = null;
                double best_x_in_neighborhood_fx = double.MaxValue;
                int move_id = -1;
                for (int i = 0; i < dimension; ++i)
                {
                    if (!IsMoveTabu(i))
                    {
                        double[] x_pi = GetNeighbor(best_solution.Values, i, constraints);
                        double fx_pi = evaluate(x_pi, mLowerBounds, mUpperBounds, constraints);

                        if (fx_pi < best_x_in_neighborhood_fx)
                        {
                            best_x_in_neighborhood = x_pi;
                            best_x_in_neighborhood_fx = fx_pi;
                            move_id = i;
                        }
                    }
                }

                if (best_x_in_neighborhood != null)
                {
                    if (best_solution.TryUpdateSolution(best_x_in_neighborhood, best_x_in_neighborhood_fx, out improvement))
                    {
                        TabuMove(move_id);
                        OnSolutionUpdated(best_solution, iteration);
                    }
                }

                LowerTabuList();

                OnStepped(best_solution, iteration);
                iteration++;
            }

            return best_solution;
        }
    }
}
