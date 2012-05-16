using System;
using System.Collections.Generic;
using System.Configuration;
using System.IO;
using System.Linq;
using System.Net;
using System.Text.RegularExpressions;
using System.Windows.Forms;
using System.Xml;
using ArdupilotMega.Utilities;
using log4net;

namespace ArdupilotMega.Utilities
{
   public static class ParameterMetaDataParser
   {
      private static readonly Regex _paramMetaRegex = new Regex(String.Format("{0}(?<MetaKey>[^:]+):(?<MetaValue>.+)", ParameterMetaDataConstants.ParamDelimeter));
      private static readonly Regex _parentDirectoryRegex = new Regex("(?<ParentDirectory>[../]*)(?<Path>.+)");
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

               objXmlTextWriter.WriteStartElement("Params");

               foreach (string parameterLocation in parameterLocations)
               {
                   string element = "none";

                   if (parameterLocation.ToLower().Contains("arducopter")) {
                       element = MainV2.Firmwares.ArduCopter2.ToString();
                   } else if (parameterLocation.ToLower().Contains("arduplane")) {
                       element = MainV2.Firmwares.ArduPlane.ToString();
                   } else if (parameterLocation.ToLower().Contains("rover")) {
                       element = MainV2.Firmwares.ArduRover.ToString();
                   }

                  // Write the start element for this parameter location
                  objXmlTextWriter.WriteStartElement(element);

                  // Read and parse the content.
                  string dataFromAddress = ReadDataFromAddress(parameterLocation);
                  ParseGroupInformation(dataFromAddress, objXmlTextWriter, parameterLocation);
                  ParseParameterInformation(dataFromAddress, objXmlTextWriter);

                  // Write the end element for this parameter location
                  objXmlTextWriter.WriteEndElement();

               }

               objXmlTextWriter.WriteEndElement();
               
               // Clear the stream
               objXmlTextWriter.WriteEndDocument();
               objXmlTextWriter.Flush();
               objXmlTextWriter.Close();
            }
         }
      }

      /// <summary>
      /// Parses the group parameter information.
      /// </summary>
      /// <param name="fileContents">The file contents.</param>
      /// <param name="objXmlTextWriter">The obj XML text writer.</param>
      /// <param name="parameterLocation">The parameter location.</param>
      private static void ParseGroupInformation(string fileContents, XmlTextWriter objXmlTextWriter, string parameterLocation)
      {
         var parsedInformation = ParseKeyValuePairs(fileContents, ParameterMetaDataConstants.Group);
         if (parsedInformation != null && parsedInformation.Count > 0)
         {
            // node is the prefix of the parameter group here
            parsedInformation.ForEach(node =>
            {
               // node.Value is a nested dictionary containing the additional meta data
               // In this case we are looking for the @Path key
               if (node.Value != null && node.Value.Count > 0)
               {
                  // Find the @Path key
                  node.Value
                     .Where(meta => meta.Key == ParameterMetaDataConstants.Path)
                     // We might have multiple paths to inspect, so break them out by the delimeter
                     .ForEach(path => path.Value.Split(new []{ ParameterMetaDataConstants.PathDelimeter }, StringSplitOptions.None)
                        .ForEach(separatedPath =>
                        {
                           // Match based on the regex defined at the top of this class
                           Match pathMatch = _parentDirectoryRegex.Match(separatedPath);
                           if (pathMatch.Success && pathMatch.Groups["Path"] != null && !String.IsNullOrEmpty(pathMatch.Groups["Path"].Value))
                           {
                              if (pathMatch.Groups["ParentDirectory"] != null && !String.IsNullOrEmpty(pathMatch.Groups["ParentDirectory"].Value))
                              {
                                 // How many directories from the original path do we have to move
                                 int numberOfParentDirectoryMoves = pathMatch.Groups["ParentDirectory"].Value.Split(new string[] { "../" }, StringSplitOptions.None).Count();

                                 // We need to remove the http:// or https:// prefix to build the new URL properly
                                 string httpTest = parameterLocation.Substring(0, 7);
                                 int trimHttpPrefixIdx = httpTest == "http://" ? 7 : 8;

                                 // Get the parts of the original URL
                                 var parameterLocationParts = parameterLocation.Substring(trimHttpPrefixIdx, parameterLocation.Length - trimHttpPrefixIdx).Split(new char[] { '/' });

                                 // Rebuild the new url taking into account the numberOfParentDirectoryMoves
                                 string urlAfterParentDirectory = string.Empty;
                                 for (int i = 0; i < parameterLocationParts.Length && i < parameterLocationParts.Length - numberOfParentDirectoryMoves; i++)
                                 {
                                    urlAfterParentDirectory += parameterLocationParts[i] + "/";
                                 }

                                 // This is the URL of the file we need to parse for comments
                                 string newPath = String.Format("{0}{1}{2}", parameterLocation.Substring(0, trimHttpPrefixIdx), urlAfterParentDirectory, pathMatch.Groups["Path"].Value);

                                 // Parse the param info from the newly constructed URL
                                 ParseParameterInformation(ReadDataFromAddress(newPath), objXmlTextWriter, node.Key);
                              }
                           }                                         
                        }));
               }
            });
         }
      }

      /// <summary>
      /// Parses the parameter information.
      /// </summary>
      /// <param name="fileContents">The file contents.</param>
      /// <param name="objXmlTextWriter">The obj XML text writer.</param>
      private static void ParseParameterInformation(string fileContents, XmlTextWriter objXmlTextWriter)
      {
         ParseParameterInformation(fileContents, objXmlTextWriter, string.Empty);
      }

      /// <summary>
      /// Parses the parameter information.
      /// </summary>
      /// <param name="fileContents">The file contents.</param>
      /// <param name="objXmlTextWriter">The obj XML text writer.</param>
      /// <param name="parameterPrefix">The parameter prefix.</param>
      private static void ParseParameterInformation(string fileContents, XmlTextWriter objXmlTextWriter, string parameterPrefix)
      {
         var parsedInformation = ParseKeyValuePairs(fileContents, ParameterMetaDataConstants.Param);
         if(parsedInformation != null && parsedInformation.Count > 0)
         {
            parsedInformation.ForEach(node =>
            {
               objXmlTextWriter.WriteStartElement(String.Format("{0}{1}", parameterPrefix, node.Key));
               if (node.Value != null && node.Value.Count > 0)
               {
                  node.Value.ForEach(meta =>
                  {
                     // Write the key value pair to XML
                     objXmlTextWriter.WriteStartElement(meta.Key);
                     objXmlTextWriter.WriteString(meta.Value);
                     objXmlTextWriter.WriteEndElement();                 
                  });
               }  
               objXmlTextWriter.WriteEndElement();
            });
         }
      }

      /// <summary>
      /// Parses the parameter information.
      /// </summary>
      /// <param name="fileContents">The file contents.</param>
      /// <param name="nodeKey">The node key.</param>
      /// <returns></returns>
      private static Dictionary<string, Dictionary<string, string>> ParseKeyValuePairs(string fileContents, string nodeKey)
      {
         var returnDict = new Dictionary<string, Dictionary<string, string>>();

         var indicies = new List<int>();
         GetIndexOfMarkers(ref indicies, fileContents, ParameterMetaDataConstants.ParamDelimeter + nodeKey, 0);

         if(indicies.Count > 0)
         {
            // Loop through the indicies of the parameter comments found
            for(int i = 0; i < indicies.Count; i++)
            {
               // This is the end index for a substring to search for parameter attributes
               // If we are on the last index in our collection, we will search to the end of the file
               var stopIdx = (i == indicies.Count - 1) ? fileContents.Length : indicies[i + 1] + 1;

                string subStringToSearch = fileContents.Substring(indicies[i], (stopIdx - indicies[i]));
               if(!String.IsNullOrEmpty(subStringToSearch))
               {
                  var metaIndicies = new List<int>();
                  GetIndexOfMarkers(ref metaIndicies, subStringToSearch, ParameterMetaDataConstants.ParamDelimeter, 0);

                  if(metaIndicies.Count > 0)
                  {
                     // This meta param key
                     var paramNameKey = subStringToSearch.Substring(metaIndicies[0], (metaIndicies[1] - metaIndicies[0]));

                     // Match based on the regex defined at the top of this class
                     Match paramNameKeyMatch = _paramMetaRegex.Match(paramNameKey);

                     if (paramNameKeyMatch.Success && paramNameKeyMatch.Groups["MetaKey"].Value == nodeKey)
                     {
                        string key = paramNameKeyMatch.Groups["MetaValue"].Value.Trim(new char[] {' '});
                        var metaDict = new Dictionary<string, string>();
                        if(!returnDict.ContainsKey(key))
                        {
                           // Loop through the indicies of the meta data found
                           for (int x = 1; x < metaIndicies.Count; x++)
                           {
                              // This is the end index for a substring to search for parameter attributes
                              // If we are on the last index in our collection, we will search to the end of the file
                              var stopMetaIdx = (x == metaIndicies.Count - 1) ? subStringToSearch.Length : metaIndicies[x + 1] + 1;

                              // This meta param string
                              var metaString = subStringToSearch.Substring(metaIndicies[x], (stopMetaIdx - metaIndicies[x]));

                              // Match based on the regex defined at the top of this class
                              Match metaMatch = _paramMetaRegex.Match(metaString);

                              // Test for success
                              if (metaMatch.Success)
                              {
                                 string metaKey = metaMatch.Groups["MetaKey"].Value.Trim(new char[] {' '});
                                 if(!metaDict.ContainsKey(metaKey))
                                 {
                                    metaDict.Add(metaKey, metaMatch.Groups["MetaValue"].Value.Trim(new char[] { ' ' }));
                                 }
                              }
                           }
                        }
                        returnDict.Add(key, metaDict);
                     }
                  }
               }
            }
         }
         return returnDict;
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
         int idx = inspectThis.IndexOf(delimeter, prevIdx, StringComparison.InvariantCultureIgnoreCase);

         // If we can't find one we stop here
         if(idx != -1)
         {
            // Add the index we found
            indicies.Add(idx);
            
            // Move the index after the parameter delimeter
            int newIdx = idx + delimeter.Length;

            // If we have more string to inspect
            if(newIdx < inspectThis.Length)
            {
               // Recursively search for the next index
                GetIndexOfMarkers(ref indicies, inspectThis, delimeter, newIdx);
            }
         }
      }

      /// <summary>
      /// Reads the data from address.
      /// </summary>
      /// <param name="address">The address.</param>
      /// <returns></returns>
      private static string ReadDataFromAddress(string address)
      {
         string data = string.Empty;

         log.Info(address);

         // Make sure we don't blow up if the user is not connected or the endpoint is not available
         try
         {
            var request = WebRequest.Create(address);

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
                        // Store the data to return
                        data = reader.ReadToEnd();

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

            // Return the data
            return data;
         }
         catch (WebException ex)
         {
            log.Error(String.Format("The request to {0} failed.", address), ex);
         }
         return string.Empty;
      }
   }
}
