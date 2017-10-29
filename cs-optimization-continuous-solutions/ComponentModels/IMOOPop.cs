using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using ContinuousOptimization.ProblemModels;

namespace ContinuousOptimization.ComponentModels
{
    public interface IMOOPop : IPop
    {
        IMOOProblem Problem
        {
            get;
            set;
        }
    }
}
