#region Using Statements

using System;

#endregion

namespace ArdupilotMega.Attributes
{
   [AttributeUsage(AttributeTargets.All, Inherited = false, AllowMultiple = false)]
   public sealed class PrivateAttribute : Attribute
   {
      private readonly bool _isPrivate;

      /// <summary>
      /// Initializes a new instance of the <see cref="PrivateAttribute"/> class.
      /// </summary>
      /// <param name="isPrivate">if set to <c>true</c> [is private].</param>
      public PrivateAttribute(bool isPrivate)
      {
         _isPrivate = isPrivate;
      }

      /// <summary>
      /// Gets a value indicating whether this instance is private.
      /// </summary>
      /// <value>
      /// 	<c>true</c> if this instance is private; otherwise, <c>false</c>.
      /// </value>
      public bool IsPrivate
      {
         get { return _isPrivate; }
      }

   }
}
