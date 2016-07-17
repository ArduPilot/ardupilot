/*
 * Created by SharpDevelop.
 * Author: Hiroshi Takey
 * Date: 16/01/2016
 * Time: 17:36
 * 
 * Thanks to:
 * 
 * Ardupilot Team
 *
 * Drone.bo Team: Yuichiro Sasaki, Roberto Soncini, Eduardo Droguett, 
 * Pablo Maldonado, Cesar Banzer and many people making growing up the team.
 * 
 * Thanks to Ardupilot APM Developers for making UAS possible.
 */

// one line to give the program's name and an idea of what it does.
// Copyright (C) 16/01/2016  Hiroshi Takey
// 
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// as published by the Free Software Foundation; either version 2
// of the License, or (at your option) any later version.
// 
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
// 
// You should have received a copy of the GNU General Public License
// along with this program; if not, If not, see <http://www.gnu.org/licenses/>.

using System;
using System.Text;
using System.IO;

namespace fix_includes_arduino166
{
	class fixer
	{
		public static void Main(string[] args)
		{
			bool sw = false;
			if (args.Length > 0){
				try{
					if (args[0] == "-P"){
						sw = true;
						Console.WriteLine("Running...");
						string[] lines2 = Directory.GetFiles(args[1]  , "*", SearchOption.AllDirectories);
						for (int i3 = 0; i3 < lines2.Length; i3++){
							string[] lines1 = File.ReadAllLines(@lines2[i3], Encoding.UTF8);
							for(int i = 0; i < lines1.Length; i++){
								bool b1 = lines1[i].Contains("#include <");
								bool b2 = lines1[i].Contains("<avr/")||lines1[i].Contains("<util/")||lines1[i].Contains("<compat/");
								if ((b1 == true)&&(b2 == false)){
									int ini1 = lines1[i].IndexOf("<");
									int fin1 = lines1[i].IndexOf(">");
									string s3 = lines1[i];
									s3 = s3.Substring(ini1,fin1 - ini1 + 1);
									int back1 = s3.IndexOf("/");
									if (back1 > 0){
										string s1 = lines1[i];
										int tam1 = s1.IndexOf("/") - s1.IndexOf("<");
										s1 = s1.Remove(s1.IndexOf("<")+1,tam1);
										lines1[i] = s1;
									}
								}
							}
							File.WriteAllLines(@lines2[i3], lines1);
						}
						Console.WriteLine("Completed!");
						Console.WriteLine("Press any key to continue . . . ");
						Console.ReadKey(true);
					}
				}catch{
					sw = false;
				}
				
			}
			if (!sw){
				Console.WriteLine();
				Console.WriteLine("----------------");
				Console.WriteLine("Usage: fix_includes_arduino166 [OPTIONS] <PATH_TO_FOLDER>");
				Console.WriteLine();
				Console.WriteLine("OPTIONS:");
				Console.WriteLine("	-P : Path dir to Ardupilot Libraries or Sketch.");
				Console.WriteLine();
				Console.WriteLine("WARNING! \n" +
				                  "This tool make changes to the files in all librarie path. You must be sure.");
				Console.WriteLine("----------------");
			}
		}
		
	}
}