using System;
using System.Collections;
using System.Collections.Generic;
using System.Linq;
using System.Text;

namespace ArdupilotMega.Utilities
{
   public static class CollectionExtensions
   {
      /// <summary>
      /// Performs the specified <paramref name="action"/> on each element of the <paramref name="enumerable"/>.
      /// 
      /// </summary>
      /// <param name="enumerable">An enumerable instance.
      ///             </param><param name="action"/>
      public static void ForEach(this IEnumerable enumerable, Action<object> action)
      {
         foreach (object obj in enumerable)
            action(obj);
      }

      /// <summary>
      /// Performs the specified <paramref name="action"/> on each element of the <paramref name="enumerable"/>.
      /// 
      /// </summary>
      /// <param name="enumerable">An enumerable instance.
      ///             </param><param name="action"/>
      public static void ForEach<T>(this IEnumerable enumerable, Action<T> action)
      {
         foreach (T obj in enumerable)
            action(obj);
      }

      /// <summary>
      /// Performs the specified <paramref name="action"/> on each element of the <paramref name="enumerable"/>.
      /// 
      /// </summary>
      /// <typeparam name="T">The type contained in the <paramref name="enumerable"/>.
      ///             </typeparam><param name="enumerable"/><param name="action"/>
      public static void ForEach<T>(this IEnumerable<T> enumerable, Action<T> action)
      {
         foreach (T obj in enumerable)
            action(obj);
      }
   }
}
