/*
 www.ArduCopter.com - www.DIYDrones.com
 Copyright (c) 2010.  All rights reserved.
 An Open Source Arduino based multicopter.
 
 File     : CLI.pde
 Version  : v1.0, Oct 18, 2010
 Author(s): ArduCopter Team
 Jani Hirvinen, Jose Julio, Jordi Muñoz,
 Ted Carancho (aeroquad), Ken McEwans, Roberto Navoni,          
 Sandro Benigno, Chris Anderson, Randy McEvans
 
 This program is free software: you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation, either version 3 of the License, or
 (at your option) any later version.
 
 This program is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 GNU General Public License for more details.
 
 You should have received a copy of the GNU General Public License
 along with this program. If not, see <http://www.gnu.org/licenses/>.
 
 * ************************************************************** *
 ChangeLog:
 
 
 * ************************************************************** *
 TODO:
 
 
 * ************************************************************** */

// CLI Functions
// This can be moved later to CLI.pde
void RunCLI () {

  // We need to initialize Serial again due it was not initialized during startup. 
  SerBeg(SerBau);   
  SerPri("Welcome to ArduCopter CLI"); 
  SerPri("Firmware: ");
  SerPri(VER);
  
  // Our main loop that never ends. Only way to get away from here is to reboot APM
  for (;;) { 



  } // Mainloop ends

}


