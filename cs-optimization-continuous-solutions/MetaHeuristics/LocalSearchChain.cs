using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;

namespace ContinuousOptimization.MetaHeuristics
{
    public class LocalSearchChain
    {
        protected List<SingleTrajectoryContinuousSolver> mLocalSearches = new List<SingleTrajectoryContinuousSolver>();
        protected List<SingleTrajectoryContinuousSolver.TerminationEvaluationMethod> mTerminationConditions = new List<SingleTrajectoryContinuousSolver.TerminationEvaluationMethod>();

        public void Add(SingleTrajectoryContinuousSolver local_search, SingleTrajectoryContinuousSolver.TerminationEvaluationMethod termination_condition)
        {
            mLocalSearches.Add(local_search);
            mTerminationConditions.Add(termination_condition);
        }

        public int Count
        {
            get { return mLocalSearches.Count; }
        }

        public SingleTrajectoryContinuousSolver GetLocalSearchAt(int index)
        {
            return mLocalSearches[index];
        }

        public SingleTrajectoryContinuousSolver.TerminationEvaluationMethod GetTerminationConditionAt(int index)
        {
            return mTerminationConditions[index];
        }

        public double[] LowerBounds
        {
            set
            {
                foreach (SingleTrajectoryContinuousSolver local_search in mLocalSearches)
                {
                    local_search.LowerBounds = value;
                }
            }
        }

        public double[] UpperBounds
        {
            set
            {
                foreach (SingleTrajectoryContinuousSolver local_search in mLocalSearches)
                {
                    local_search.UpperBounds = value;
                }
            }
        }
    }
}
