using System;
using System.Configuration;
using System.IO;
using System.Windows.Forms;
using System.Xml.Linq;
using System.Linq;
using System.Collections;
using System.Collections.Generic;

namespace ArdupilotMega.Utilities
{
   public class ParameterMetaDataRepository
   {
      private static XDocument _parameterMetaDataXML;

      /// <summary>
      /// Initializes a new instance of the <see cref="ParameterMetaDataRepository"/> class.
      /// </summary>
      public ParameterMetaDataRepository()
      {
          Reload();
      }

      public void Reload()
      {
          string paramMetaDataXMLFileName = String.Format("{0}{1}{2}", Application.StartupPath, Path.DirectorySeparatorChar, ConfigurationManager.AppSettings["ParameterMetaDataXMLFileName"]);
          try
          {
              if (File.Exists(paramMetaDataXMLFileName))
                  _parameterMetaDataXML = XDocument.Load(paramMetaDataXMLFileName);

          }
          catch { } // Exception System.Xml.XmlException: Root element is missing.
      }

      /// <summary>
      /// Gets the parameter meta data.
      /// </summary>
      /// <param name="nodeKey">The node key.</param>
      /// <param name="metaKey">The meta key.</param>
      /// <returns></returns>
      public string GetParameterMetaData(string nodeKey, string metaKey)
      {
         if(_parameterMetaDataXML != null)
         {
            // Use this to find the endpoint node we are looking for
            // Either it will be pulled from a file in the ArduPlane hierarchy or the ArduCopter hierarchy
             try
             {
                 var element = _parameterMetaDataXML.Element("Params").Element(MainV2.cs.firmware.ToString());
                 if (element != null && element.HasElements)
                 {
                     var node = element.Element(nodeKey);
                     if (node != null && node.HasElements)
                     {
                         var metaValue = node.Element(metaKey);
                         if (metaValue != null)
                         {
                             return metaValue.Value;
                         }
                     }
                 }
             }
             catch { } // Exception System.ArgumentException: '' is an invalid expanded name.
         }
         return string.Empty;
      }
   }
}
