using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;

namespace ContinuousOptimization.ProblemModels
{
    /// <summary>
    /// function whose fitness is always the smaller the better
    /// </summary>
    public abstract class CostFunction
    {

        protected int mDimensionCount; //number of dimension
        protected int mBasicFunctionCount; //number of basic functions
        protected double[] mLowerBounds; //lower bound vector
        protected double[] mUpperBounds; //upper bound vector

        //algorithm variables
        protected int mEvaluationCount; //number of evaluation

        public int DimensionCount
        {
            get { return mDimensionCount; }
        }

        public int BasicFunctionCount
        {
            get { return mBasicFunctionCount; }
        }

        public double FindLowerBoundAtDimension(int dimension)
        {
            return mLowerBounds[dimension];
        }

        public double[] LowerBounds
        {
            get { return mLowerBounds; }
            set
            {
                if (value == null || value.Length != mDimensionCount)
                {
                    throw new IndexOutOfRangeException();
                }
                mLowerBounds = value;
            }
        }

        public double[] UpperBounds
        {
            get { return mUpperBounds; }
            set
            {
                if (value == null && value.Length != mDimensionCount)
                {
                    throw new IndexOutOfRangeException();
                }
                mUpperBounds = value;
            }
        }

        public void SetLowerBoundAtDimension(int dimension, double lower_bound)
        {
            mLowerBounds[dimension] = lower_bound;
        }

        public double FindUpperBoundAtDimension(int dimension)
        {
            return mUpperBounds[dimension];
        }

        public void SetUpperBoundAtDimension(int dimension, double upper_bound)
        {
            mUpperBounds[dimension] = upper_bound;
        }

        public int Evaluationcount
        {
            get { return mEvaluationCount; }
        }

        protected virtual double _Evaluate(double[] solution)
        {
            mEvaluationCount++;
            return 0;
        }

        public virtual double Evaluate(double[] solution)
        {
            double objective_value = _Evaluate(solution);

            bool within_bounds = true;

            for (int i = 0; i < mDimensionCount; ++i)
            {
                if ((solution[i] < mLowerBounds[i]) || (solution[i] > mUpperBounds[i]))
                {
                    within_bounds = false;
                    break;
                }
            }

            if (!within_bounds)
            {
                objective_value = double.MaxValue;
            }

            return objective_value;
        }

        public double Evaluate(double x)
        {
            double[] solution = new double[1];
            solution[0] = x;
            return Evaluate(solution);
        }

        protected virtual void _CalcGradient(double[] solution, double[] grad)
        {
            GradientEstimation.CalcGradient(solution, grad, (s, constraints) =>
                {
                    return Evaluate(s);
                });
            mEvaluationCount += mDimensionCount;
        }

        public void CalcGradient(double[] solution, double[] grad)
        {
            _CalcGradient(solution, grad);
        }

        public void CalcGradient(double x, out double fprime)
        {
            double[] solution = new double[1];
            solution[0] = x;
            double[] grad = new double[1];
            _CalcGradient(solution, grad);
            fprime = grad[0];
        }

        public bool IsOutOfBounds(double[] solution)
        {
            bool result = false;
            for (int i = 0; i < mDimensionCount; ++i)
            {
                if (solution[i] < mLowerBounds[i])
                {
                    result = true;
                    break;
                }
                else if (solution[i] > mUpperBounds[i])
                {
                    result = true;
                    break;
                }
            }

            return result;
        }

        public CostFunction(int dimension_count, double lower_bound, double upper_bound)
        {
            mLowerBounds = new double[dimension_count];
            mUpperBounds = new double[dimension_count];

            for (int i = 0; i < dimension_count; ++i)
            {
                mLowerBounds[i] = lower_bound;
                mUpperBounds[i] = upper_bound;
            }

            Initialize(dimension_count);
        }

        public CostFunction()
        {

        }

        protected void Initialize(int dimension_count)
        {
            mDimensionCount = dimension_count;

            //initialize local variables
            mEvaluationCount = 0;
        }

        public virtual double[] CreateRandomSolution()
        {
            double[] x_0 = new double[mDimensionCount];

            Random r = new Random();
            for (int i = 0; i < mDimensionCount; ++i)
            {
                x_0[i] = mLowerBounds[i] + r.NextDouble() * (mUpperBounds[i] - mLowerBounds[i]);
            }

            return x_0;
        }

    }
}
