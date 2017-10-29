using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;

namespace ContinuousOptimization.MetaHeuristics.ExtensionMethods
{
    public static class ContinuousSolutionExtensionMethods
    {
        public static bool IsNew(this ContinuousSolution s)
        {
            if (s.HasAttribute("IsNew"))
            {
                return Convert.ToInt32(s["IsNew"]) == 1;
            }
            else
            {
                return false;
            }
        }

        public static void SetIsNew(this ContinuousSolution s, bool is_new)
        {
            s["IsNew"] = is_new ? 1 : 0;
        }

        public static double[] GetMutationStrategy(this ContinuousSolution s)
        {
            if (s.HasAttribute("MutationStrategy"))
            {
                return s["MutationStrategy"] as double[];
            }
            return null;
        }

        public static void SetMutationStrategy(this ContinuousSolution s, double[] strategy)
        {
            s["MutationStrategy"] = strategy;
        }

        public static int GetWins(this ContinuousSolution s)
        {
            if (s.HasAttribute("Wins"))
            {
                return (int)s["Wins"];
            }
            return 0;
        }

        public static void SetWins(this ContinuousSolution s, int val)
        {
            s["Wins"] = val;
        }
    }
}
