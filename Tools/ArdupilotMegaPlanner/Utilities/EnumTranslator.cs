#region Using Statements

using System;
using System.Collections.Generic;
using System.Linq;
using System.Reflection;
using ArdupilotMega.Attributes;

#endregion

namespace ArdupilotMega.Utilities
{
   public static class EnumTranslator
   {
      /// <summary>
      /// Translates this instance.
      /// </summary>
      /// <typeparam name="T"></typeparam>
      /// <returns></returns>
      public static Dictionary<int, string> Translate<T>()
      {
         return Translate<T>(true);
      }

      /// <summary>
      /// Translates the specified check private.
      /// </summary>
      /// <typeparam name="T"></typeparam>
      /// <param name="checkPrivate">if set to <c>true</c> [check private].</param>
      /// <returns></returns>
      public static Dictionary<int, string> Translate<T>(bool checkPrivate)
      {
         return Translate<T>(string.Empty, checkPrivate);
      }

      /// <summary>
      /// Translates the specified default text.
      /// </summary>
      /// <typeparam name="T"></typeparam>
      /// <param name="defaultText">The default text.</param>
      /// <param name="checkPrivate">if set to <c>true</c> [check private].</param>
      /// <returns></returns>
      public static Dictionary<int, string> Translate<T>(string defaultText, bool checkPrivate)
      {
         return Translate<T>(defaultText, checkPrivate, true);
      }

      /// <summary>
      /// Translates the specified default text.
      /// </summary>
      /// <typeparam name="T"></typeparam>
      /// <param name="defaultText">The default text.</param>
      /// <param name="checkPrivate">if set to <c>true</c> [check private].</param>
      /// <param name="sorting">if set to <c>true</c> [sorting].</param>
      /// <returns></returns>
      public static Dictionary<int, string> Translate<T>(string defaultText, bool checkPrivate, bool sorting)
      {
         var types = new Dictionary<int, string>();
         var tempTypes = new Dictionary<int, string>();
         if (!String.IsNullOrEmpty(defaultText)) types.Add(-1, defaultText);
         foreach (FieldInfo fieldInfo in typeof(T).GetFields(BindingFlags.Static | BindingFlags.GetField | BindingFlags.Public))
         {
            bool add = true;
            string displayText = string.Empty;
            T type = (T)fieldInfo.GetValue(typeof(T));
            object[] displayTextObjectArr = fieldInfo.GetCustomAttributes(typeof(DisplayTextAttribute), true);
            displayText = (displayTextObjectArr.Length > 0)
                             ? ((DisplayTextAttribute)displayTextObjectArr[0]).Text
                             : type.ToString();
            if (checkPrivate)
            {
               object[] privateAttributeObjectArr = fieldInfo.GetCustomAttributes(typeof(PrivateAttribute), true);
               if (privateAttributeObjectArr.Length > 0)
               {
                  add = !((PrivateAttribute)privateAttributeObjectArr[0]).IsPrivate;
               }
            }
            if (add)
            {
               tempTypes.Add(Convert.ToInt32(type), displayText);
            }
         }
         if (sorting)
         {
            foreach(var x in tempTypes.OrderBy(x => x.Value).ToDictionary(x => x.Key, x => x.Value)){ types.Add(x.Key, x.Value); }
         }
         else
         {
            foreach (var x in tempTypes.ToDictionary(x => x.Key, x => x.Value)) { types.Add(x.Key, x.Value); }
         }
         return types;
      }

      /// <summary>
      /// Gets the display text.
      /// </summary>
      /// <typeparam name="T"></typeparam>
      /// <param name="value">The value.</param>
      /// <returns></returns>
      public static string GetDisplayText<T>(T value)
      {
         var displayText = string.Empty;
         var fieldInfo = value.GetType().GetField(value.ToString());
         if(fieldInfo != null)
         {
            T type = (T)fieldInfo.GetValue(typeof(T));
            if(type != null)
            {
               object[] displayTextObjectArr = fieldInfo.GetCustomAttributes(typeof(DisplayTextAttribute), true);
               displayText = (displayTextObjectArr.Length > 0)
                                       ? ((DisplayTextAttribute)displayTextObjectArr[0]).Text
                                       : type.ToString();
            }
         }
         return displayText;
      }

      public static int GetValue<T>(string item)
      {
          var list = Translate<T>();

          foreach (var kvp in list)
          {
              if (kvp.Value == item)
                  return kvp.Key;
          }

          return -1;
      }

   }  

}
