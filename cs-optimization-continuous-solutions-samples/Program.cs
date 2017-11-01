using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;

namespace ContinuousOptimization.UT
{
    class Program
    {
        static void Main(string[] args)
        {
            //UT_BFGS.Run_Sphere();
            //UT_NelderMead.Run_Sphere();
            UT_ConjugateGradientSearch.Run_Sphere();
            //UT_SLOP.Run_Sphere();
                //UT_Powell.Run_Sphere();

            //UT_BFGS.RunRosenbrockSaddle(2000);
            //UT_NelderMead.RunRosenbrockSaddle(1000);
            UT_ConjugateGradientSearch.RunRosenbrockSaddle(1000);
            //UT_SLOP.RunRosenbrockSaddle(1000);
            //UT_Powell.RunRosenbrockSaddle(1000);
        }
    }
}
