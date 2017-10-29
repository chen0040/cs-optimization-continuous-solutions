using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;

namespace ContinuousOptimization.MetaHeuristics.ExtensionMethods
{
    public static class ArrayExtensionMethods
    {
        public static bool ContainsItem<T>(this T[] list, T item)
            where T : class
        {
            foreach (T list_item in list)
            {
                if (list_item.Equals(item))
                {
                    return true;
                }
            }
            return false;
        }

    }
}
