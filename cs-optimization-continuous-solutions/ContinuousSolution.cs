using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;

namespace ContinuousOptimization
{
    /// <summary>
    /// global best solution, which is the solution that produces the minimum fitness
    /// </summary>
    public class ContinuousSolution : BaseSolution<double>
    {
        public void MutateNormal(int index, double sigma)
        {
            mValues[index] += RandomEngine.Gauss(0, sigma);
        }



        public static ContinuousSolution operator /(ContinuousSolution a, double val)
        {
            double[] x = new double[a.Length];

            for (int i = 0; i < x.Length; ++i)
            {
                x[i] = a[i] / val;
            }
            return new ContinuousSolution(x, double.MaxValue);
        }

        public static ContinuousSolution operator +(ContinuousSolution a, ContinuousSolution b)
        {
            int length = a.Length;
            if (length == 0)
            {
                throw new IndexOutOfRangeException();
            }
            if (b.Length != length)
            {
                throw new IndexOutOfRangeException();
            }

            double[] x = new double[length];

            for (int i = 0; i < length; ++i)
            {
                x[i] = a[i] + b[i];
            }
            return new ContinuousSolution(x, double.MaxValue);
        }

        public static ContinuousSolution operator *(ContinuousSolution a, double val)
        {
            double[] x = new double[a.Length];

            for (int i = 0; i < x.Length; ++i)
            {
                x[i] = a[i] * val;
            }
            return new ContinuousSolution(x, double.MaxValue);
        }

        public static ContinuousSolution operator -(ContinuousSolution a, ContinuousSolution b)
        {
            int length = a.Length;
            if (length == 0)
            {
                throw new IndexOutOfRangeException();
            }
            if (b.Length != length)
            {
                throw new IndexOutOfRangeException();
            }

            double[] x = new double[length];
            for (int i = 0; i < length; ++i)
            {
                x[i] = a[i] - b[i];
            }
            ContinuousSolution result = new ContinuousSolution(x, double.MaxValue);

            return result;
        }

        public override double GetDistanceSq2(BaseSolution<double> rhs)
        {
            if (this.mValues == null)
            {
                return double.MaxValue;
            }
            if (rhs.Values == null)
            {
                return double.MaxValue;
            }

            if (rhs.Values.Length != this.mValues.Length)
            {
                return double.MaxValue;
            }

            double distanceSq = 0;
            for (int i = 0; i < this.Length; ++i)
            {
                distanceSq += System.Math.Pow((mValues[i] - rhs[i]), 2);
            }

            return distanceSq;
        }

        public ContinuousSolution(int dimension)
            : base(dimension)
        {

        }

        public ContinuousSolution()
            : base()
        {

        }

        public ContinuousSolution(double[] s, double fitness)
            : base(s, fitness)
        {
        }

        public override BaseSolution<double> Clone()
        {
            ContinuousSolution clone = new ContinuousSolution(mValues, mCost);
            return clone;
        }

        public override bool Equals(object obj)
        {
            if (obj is BaseSolution<double>)
            {
                BaseSolution<double> cast_obj = obj as BaseSolution<double>;
                int length1 = this.Length;
                int length2 = cast_obj.Length;
                if (length1 == length2)
                {
                    for (int i = 0; i < length1; ++i)
                    {
                        if (this[i] != cast_obj[i])
                        {
                            return false;
                        }
                    }
                    return true;
                }
            }
            return false;
        }

        public override int GetHashCode()
        {
            return base.GetHashCode();
        }
    }
}
