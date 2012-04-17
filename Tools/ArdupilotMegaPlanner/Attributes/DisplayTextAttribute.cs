#region Using Statements

using System;

#endregion

namespace ArdupilotMega.Attributes
{
   /// <summary>
   /// Used to decorate a type or type member with display text.
   /// </summary>
   [AttributeUsage(AttributeTargets.All, Inherited = false, AllowMultiple = false)]
   public sealed class DisplayTextAttribute : Attribute
   {

      private readonly string _text;

      /// <summary>
      /// Initializes a new instance of the <see cref="DisplayTextAttribute"/> class.
      /// </summary>
      /// <param name="text">The text.</param>
      /// <exception cref="ArgumentException">
      /// Thrown when <paramref name="text"/> is null or empty.
      /// </exception>
      public DisplayTextAttribute(string text)
      {
         if (String.IsNullOrEmpty(text))
         {
            throw new ArgumentException("\"text\" is required.");
         }
         _text = text;
      }

      /// <summary>
      /// Gets the text.
      /// </summary>
      /// <value>The text.</value>
      public string Text
      {
         get { return _text; }
      }

   }
}
