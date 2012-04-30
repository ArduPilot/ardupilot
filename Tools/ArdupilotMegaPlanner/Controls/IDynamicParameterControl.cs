using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;

namespace ArdupilotMega.Controls
{
   public interface IDynamicParameterControl
   {
      /// <summary>
      /// Gets the name.
      /// </summary>
      /// <value>
      /// The name.
      /// </value>
      string Name { get; set; }

      /// <summary>
      /// Gets the value.
      /// </summary>
      /// <value>
      /// The value.
      /// </value>
      string Value { get; set; }
   }
}
