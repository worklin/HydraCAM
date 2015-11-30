/* -*- c++ -*- */

/*
    HydraCAM firmware based on HydraCAM, Sprinter, and grbl.
 Copyright (C) 2014 Scott McGraw

 This program is free software: you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation, either version 3 of the License, or
 (at your option) any later version.

 This program is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details.

 You should have received a copy of the GNU General Public License
 along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

/*
 This firmware is a stripped down version based on HydraCAM for driving a 2 axis laser cutter
  (https://github.com/kliment/Sprinter)
  (https://github.com/simen/grbl/tree)
 */

/* All the implementation is done in *.cpp files to get better compatibility with avr-gcc without the Arduino IDE */
/* Use this file to help the Arduino IDE find which Arduino libraries are needed and to keep documentation on GCode */

#include "Configuration.h"
#include "pins.h"


