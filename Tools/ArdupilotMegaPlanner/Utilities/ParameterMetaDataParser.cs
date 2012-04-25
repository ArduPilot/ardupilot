using System;
using System.Collections.Generic;
using System.Configuration;
using System.IO;
using System.Linq;
using System.Net;
using System.Text.RegularExpressions;
using System.Windows.Forms;
using System.Xml;
using log4net;

namespace ArdupilotMega.Utilities
{
   public static class ParameterMetaDataParser
   {
      private static readonly Regex _paramMetaRegex = new Regex(String.Format("{0}(?<MetaKey>[^:]+):(?<MetaValue>.+)", ParameterMetaDataConstants.Delimeter));
      private static readonly ILog log =
         LogManager.GetLogger(System.Reflection.MethodBase.GetCurrentMethod().DeclaringType);

      public static void GetParameterInformation()
      {
         string parameterLocationsString = ConfigurationManager.AppSettings["ParameterLocations"];

         if(!String.IsNullOrEmpty(parameterLocationsString))
         {
            var parameterLocations = parameterLocationsString.Split(';').ToList();
            parameterLocations.RemoveAll(String.IsNullOrEmpty);

            string sStartupPath = Application.StartupPath;
            using (var objXmlTextWriter = new XmlTextWriter(String.Format("{0}\\{1}", sStartupPath, ConfigurationManager.AppSettings["ParameterMetaDataXMLFileName"]), null))
            {
               objXmlTextWriter.Formatting = Formatting.Indented;
               objXmlTextWriter.WriteStartDocument();

               foreach (string parameterLocation in parameterLocations)
               {
                  log.Info(parameterLocation);

                  var request = WebRequest.Create(parameterLocation);

                  // Plenty of timeout
                  request.Timeout = 10000;

                  // Set the Method property of the request to GET.
                  request.Method = "GET";

                  // Get the response.
                  using (var response = request.GetResponse())
                  {
                     // Display the status.
                     log.Info(((HttpWebResponse)response).StatusDescription);

                     // Get the stream containing content returned by the server.
                     using (var dataStream = response.GetResponseStream())
                     {
                        if (dataStream != null)
                        {
                           // Open the stream using a StreamReader for easy access.
                           using (var reader = new StreamReader(dataStream))
                           {
                              // Write the start element for this parameter location
                              objXmlTextWriter.WriteStartElement(parameterLocation);

                              // Read and parse the content.
                              ParseParameterInformation(reader.ReadToEnd(), objXmlTextWriter);

                              // Write the end element for this parameter location
                              objXmlTextWriter.WriteEndElement();

                              // Close the reader
                              reader.Close();
                           }

                           // Close the datastream
                           dataStream.Close();
                        }
                     }

                     // Close the response
                     response.Close();
                  }

               }
               
               // Clear the stream
               objXmlTextWriter.WriteEndDocument();
               objXmlTextWriter.Flush();
               objXmlTextWriter.Close();
            }
         }
      }

      /// <summary>
      /// Parses the parameter information.
      /// </summary>
      /// <param name="fileContents">The file contents.</param>
      /// <param name="objXmlTextWriter">The obj XML text writer.</param>
      private static void ParseParameterInformation(string fileContents, XmlTextWriter objXmlTextWriter)
      {
         var indicies = new List<int>();
         GetIndexOfMarkers(ref indicies, fileContents, ParameterMetaDataConstants.Delimeter + ParameterMetaDataConstants.Param, 0);

         if(indicies.Count > 0)
         {
            // Loop through the indicies of the parameter comments found
            for(int i = 0; i < indicies.Count; i++)
            {
               // This is the end index for a substring to search for parameter attributes
               // If we are on the last index in our collection, we will search to the end of the file
               var stopIdx = (i == indicies.Count - 1) ? fileContents.Length : indicies[i + 1];

               string subStringToSearch = fileContents.Substring(indicies[i], (stopIdx - indicies[i]));
               if(!String.IsNullOrEmpty(subStringToSearch))
               {
                  var metaIndicies = new List<int>();
                  GetIndexOfMarkers(ref metaIndicies, subStringToSearch, ParameterMetaDataConstants.Delimeter, 0);

                  if(metaIndicies.Count > 0)
                  {
                     // This meta param key
                     var paramNameKey = subStringToSearch.Substring(metaIndicies[0], (metaIndicies[1] - metaIndicies[0]));

                     // Match based on the regex defined at the top of this class
                     Match paramNameKeyMatch = _paramMetaRegex.Match(paramNameKey);

                     if (paramNameKeyMatch.Success && paramNameKeyMatch.Groups["MetaKey"].Value == ParameterMetaDataConstants.Param)
                     {
                        objXmlTextWriter.WriteStartElement(paramNameKeyMatch.Groups["MetaValue"].Value.Trim(new char[] { ' ' }));

                        // Loop through the indicies of the meta data found
                        for (int x = 1; x < metaIndicies.Count; x++)
                        {
                           // This is the end index for a substring to search for parameter attributes
                           // If we are on the last index in our collection, we will search to the end of the file
                           var stopMetaIdx = (x == metaIndicies.Count - 1) ? subStringToSearch.Length : metaIndicies[x + 1];

                           // This meta param string
                           var metaString = subStringToSearch.Substring(metaIndicies[x], (stopMetaIdx - metaIndicies[x]));

                           // Match based on the regex defined at the top of this class
                           Match metaMatch = _paramMetaRegex.Match(metaString);

                           // Test for success
                           if (metaMatch.Success)
                           {
                              // Write the key value pair to XML
                              objXmlTextWriter.WriteStartElement(metaMatch.Groups["MetaKey"].Value.Trim(new char[] { ' ' }));
                              objXmlTextWriter.WriteString(metaMatch.Groups["MetaValue"].Value.Trim(new char[] { ' ' }));
                              objXmlTextWriter.WriteEndElement();
                           }
                        }

                        // End this parameter node
                        objXmlTextWriter.WriteEndElement();
                     }
                  }
               }
            }
         }
      }

      /// <summary>
      /// Gets the index of param markers.
      /// </summary>
      /// <param name="indicies">The indicies.</param>
      /// <param name="inspectThis">The string to be inspected for a parameter.</param>
      /// <param name="delimeter">The delimeter.</param>
      /// <param name="prevIdx">The prev idx.</param>
      private static void GetIndexOfMarkers(ref List<int> indicies, string inspectThis, string delimeter, int prevIdx)
      {
         // Find the index of the start of a parameter comment
         int idx = inspectThis.IndexOf(delimeter, StringComparison.InvariantCultureIgnoreCase);

         // If we can't find one we stop here
         if(idx != -1)
         {
            // Add the index we found
            indicies.Add(idx + prevIdx);
            
            // Move the index after the parameter delimeter
            int newIdx = idx + delimeter.Length;

            // If we have more string to inspect
            if(newIdx < inspectThis.Length)
            {
               // Recursively search for the next index
               GetIndexOfMarkers(ref indicies, inspectThis.Substring(newIdx, (inspectThis.Length - newIdx)), delimeter, idx + prevIdx);
            }
         }
      }
   }
}
