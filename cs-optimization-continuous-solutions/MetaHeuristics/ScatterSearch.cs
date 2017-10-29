using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using ContinuousOptimization.MetaHeuristics;

using ContinuousOptimization.MetaHeuristics.ExtensionMethods;

namespace ContinuousOptimization.MetaHeuristics
{
    public class ScatterSearch : MultiTrajectoryContinuousSolver
    {
        protected int mReferenceSetSize = 10; //smaller reference set is preferred, typically 10 to 20
        protected int mDiverseSetSize = 20;
        protected int mNumElite = 5;

        protected SingleTrajectoryContinuousSolver mLocalSearch = null;
        protected TerminationEvaluationMethod mLocalSearchTerminationCondition = null;

        public delegate double[] GenerateSolutionMethod(CostEvaluationMethod evaluate, double[] lower_bounds, double[] upper_bounds, object constraints);
        protected GenerateSolutionMethod mSolutionGenerator;

        public int ReferenceSetSize
        {
            get { return mReferenceSetSize; }
            set { mReferenceSetSize = value; }
        }

        public int DiverseSetSize
        {
            get { return mDiverseSetSize; }
            set { mDiverseSetSize = value; }
        }

        public int NumElite
        {
            get { return mNumElite; }
            set { mNumElite = value; }
        }

        public ScatterSearch(GenerateSolutionMethod solution_generator, SingleTrajectoryContinuousSolver local_search, TerminationEvaluationMethod local_search_termination_condition)
        {
            mSolutionGenerator = solution_generator;
            mLocalSearch = local_search;
            mLocalSearchTerminationCondition = local_search_termination_condition;

            if (mSolutionGenerator == null || mLocalSearch == null || mLocalSearchTerminationCondition == null)
            {
                throw new NullReferenceException();
            }
        }

        public SingleTrajectoryContinuousSolver LocalSearch
        {
            get { return mLocalSearch; }
            set { mLocalSearch = value; }
        }

        public TerminationEvaluationMethod LocalSearchTerminationCondition
        {
            get { return mLocalSearchTerminationCondition; }
            set { mLocalSearchTerminationCondition = value; }
        }

        protected ContinuousSolution DoLocalSearch(double[] x, CostEvaluationMethod evaluate, GradientEvaluationMethod calc_gradient, object constraints)
        {
            if (mLocalSearch != null)
            {
                return mLocalSearch.Minimize(x, evaluate, calc_gradient, mLocalSearchTerminationCondition, constraints);
            }
            double fx = evaluate(x, mLowerBounds, mUpperBounds, constraints);
            return new ContinuousSolution(x, fx);
        }

        protected double[] CreateRandomSolution(CostEvaluationMethod evaluate, double[] lower_bounds, double[] upper_bounds, object constraints)
        {
            return mSolutionGenerator(evaluate, lower_bounds, upper_bounds, constraints);
        }

        protected ContinuousSolution[] Diversify(List<ContinuousSolution> diverse_set)
        {
            ContinuousSolution[] ref_set = new ContinuousSolution[mReferenceSetSize];
            for (int i = 0; i < mNumElite; ++i)
            {
                ref_set[i] = diverse_set[i];
            }

            int remainder_set_size = mDiverseSetSize - mNumElite;
            ContinuousSolution[] remainder_set = new ContinuousSolution[remainder_set_size];
            for (int i = mNumElite; i < mDiverseSetSize; ++i)
            {
                remainder_set[i - mNumElite] = diverse_set[i];
            }
            Dictionary<ContinuousSolution, double> remainder_distance_set = new Dictionary<ContinuousSolution, double>();
            for (int i = 0; i < remainder_set_size; ++i)
            {
                double distance = 0;
                for (int j = 0; j < mNumElite; ++j)
                {
                    distance += remainder_set[i].GetDistance2(ref_set[j]);
                }
                remainder_distance_set[remainder_set[i]] = distance;
            }
            remainder_set = remainder_set.OrderBy(s => remainder_distance_set[s]).ToArray();

            for (int i = mNumElite; i < mReferenceSetSize; ++i)
            {
                ref_set[i] = remainder_set[i - mNumElite];
            }

            return ref_set;
        }

        protected List<ContinuousSolution> ConstructDiverseSet(CostEvaluationMethod evaluate, GradientEvaluationMethod calc_grad, object constraints)
        {
            double[] x = null;
            ContinuousSolution best_solution = new ContinuousSolution();

            double? improvement2 = null;
            List<ContinuousSolution> diverse_set = new List<ContinuousSolution>();
            while (diverse_set.Count < mReferenceSetSize)
            {
                x = CreateRandomSolution(evaluate, mLowerBounds, mUpperBounds, constraints);

                ContinuousSolution xs = DoLocalSearch(x, evaluate, calc_grad, constraints);

                double minDistanceSq = double.MaxValue;
                double distanceSq = 0;
                bool contains_item = false;
                foreach (ContinuousSolution s in diverse_set)
                {
                    distanceSq = s.GetDistanceSq2(xs);
                    if (distanceSq.Equals(minDistanceSq))
                    {
                        contains_item = true;
                        break;
                    }
                }

                if (!contains_item)
                {
                    diverse_set.Add(xs);
                }

                best_solution.TryUpdateSolution(xs.Values, xs.Cost, out improvement2);
            }

            return diverse_set;
        }

        protected List<ContinuousSolution[]> SelectSubsets(ContinuousSolution[] ref_set)
        {
            List<ContinuousSolution> additions = new List<ContinuousSolution>();
            List<ContinuousSolution> remainders = new List<ContinuousSolution>();

            for (int i = 0; i < mReferenceSetSize; ++i)
            {
                if (ref_set[i].IsNew())
                {
                    additions.Add(ref_set[i]);
                }
                else
                {
                    remainders.Add(ref_set[i]);
                }
            }
            if (remainders.Count == 0)
            {
                foreach (ContinuousSolution s in additions)
                {
                    remainders.Add(s);
                }
            }

            List<ContinuousSolution[]> subsets = new List<ContinuousSolution[]>();

            for (int i = 0; i < additions.Count; ++i)
            {
                ContinuousSolution addition = additions[i];
                for (int j = 0; j < remainders.Count; ++j)
                {
                    ContinuousSolution remainder = remainders[j];

                    if (addition != remainder)
                    {
                        subsets.Add(new ContinuousSolution[] { remainder, addition });
                    }
                }
            }

            return subsets;
        }

        public ContinuousSolution[] Recombine(ContinuousSolution[] subset, object constraints)
        {
            ContinuousSolution a = subset[0];
            ContinuousSolution b = subset[1];

            ContinuousSolution d = (a - b) / 2;

            double[] lower_bounds = null;
            double[] upper_bounds = null;

            if (mLowerBounds == null || mUpperBounds == null)
            {
                if (constraints != null && constraints is Tuple<double[], double[]>)
                {
                    Tuple<double[], double[]> bounds = constraints as Tuple<double[], double[]>;
                    lower_bounds = bounds.Item1;
                    upper_bounds = bounds.Item2;
                }
                else
                {
                    throw new InvalidCastException();
                }
            }
            else
            {
                lower_bounds = mLowerBounds;
                upper_bounds = mUpperBounds;
            }

            if (lower_bounds.Length < d.Length)
            {
                throw new IndexOutOfRangeException();
            }
            if (upper_bounds.Length < d.Length)
            {
                throw new IndexOutOfRangeException();
            }

            ContinuousSolution[] children = new ContinuousSolution[subset.Length];
            for (int i = 0; i < subset.Length; ++i)
            {
                double[] x = new double[d.Length];
                for (int j = 0; j < d.Length; ++j)
                {
                    int direction = RandomEngine.NextDouble() < 0.5 ? 1 : -1;
                    double r = RandomEngine.NextDouble();

                    x[j] = subset[i][j] + d[j] * direction * r;
                    x[j] = System.Math.Max(lower_bounds[j], x[j]);
                    x[j] = System.Math.Min(upper_bounds[j], x[j]);
                }

                children[i] = new ContinuousSolution(x, double.MaxValue);
            }

            return children;
        }

        protected bool ExploreSubsets(ref ContinuousSolution[] ref_set, CostEvaluationMethod evaluate, GradientEvaluationMethod calc_grad, object constraints)
        {
            bool is_improved = false;

            List<ContinuousSolution[]> subsets = SelectSubsets(ref_set);
            for (int i = 0; i < ref_set.Length; ++i)
            {
                ref_set[i].SetIsNew(false);
            }

            for (int i = 0; i < subsets.Count; ++i)
            {
                ContinuousSolution[] subset = subsets[i];
                ContinuousSolution[] candidate_solutions = Recombine(subset, constraints);
                ContinuousSolution[] improvements = new ContinuousSolution[candidate_solutions.Length];
                for (int j = 0; j < improvements.Length; ++j)
                {
                    //double fx = evaluate(candidate_solutions[j].Values, mLowerBounds, mUpperBounds, constraints);
                    ContinuousSolution s = mLocalSearch.Minimize(candidate_solutions[j].Values, evaluate, calc_grad, mLocalSearchTerminationCondition, constraints);
                    //double improvement = fx - s.Cost;
                    improvements[j] = s;
                }
                for (int j = 0; j < improvements.Length; ++j)
                {
                    if (ref_set.ContainsItem(improvements[j]))
                    {
                        improvements[j].SetIsNew(false);
                    }
                    else
                    {
                        improvements[j].SetIsNew(true);
                        ref_set = ref_set.OrderBy(s => s.Cost).ToArray();
                        if (ref_set[mReferenceSetSize - 1].Cost > improvements[j].Cost)
                        {
                            ref_set[mReferenceSetSize - 1] = improvements[j];
                            is_improved = true;
                        }
                    }
                }
            }

            return is_improved;
        }

        public override ContinuousSolution Minimize(CostEvaluationMethod evaluate, GradientEvaluationMethod calc_grad, TerminationEvaluationMethod should_terminate, object constraints = null)
        {
            double? improvement = null;
            int iteration = 0;

            mLocalSearch.LowerBounds = mLowerBounds;
            mLocalSearch.UpperBounds = mUpperBounds;

            List<ContinuousSolution> diverse_set = ConstructDiverseSet(evaluate, calc_grad, constraints);

            diverse_set = diverse_set.OrderBy(s => s.Cost).ToList();

            ContinuousSolution[] ref_set = Diversify(diverse_set);

            foreach (ContinuousSolution s in ref_set)
            {
                s.SetIsNew(true);
            }

            ContinuousSolution best_solution = ref_set[0].Clone() as ContinuousSolution;

            while (!should_terminate(improvement, iteration))
            {
                bool was_changed = ExploreSubsets(ref ref_set, evaluate, calc_grad, constraints);

                if (!was_changed) break;

                if (best_solution.TryUpdateSolution(ref_set[0].Values, ref_set[0].Cost, out improvement))
                {
                    OnSolutionUpdated(best_solution, iteration);
                }

                OnStepped(best_solution, iteration);
                iteration++;
            }

            return best_solution;
        }
    }
}
