/*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

 Header:       TurbinePlantXML.h
 Date started: 2026-10-08

 ------------- Copyright (C) 2026 The JSBSim contributors ----------------------

 This program is free software; you can redistribute it and/or modify it under
 the terms of the GNU Lesser General Public License as published by the Free
 Software Foundation; either version 2 of the License, or (at your option) any
 later version.

 This program is distributed in the hope that it will be useful, but WITHOUT
 ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
 FOR A PARTICULAR PURPOSE.  See the GNU Lesser General Public License for more
 details.

 You should have received a copy of the GNU Lesser General Public License along
 with this program; if not, write to the Free Software Foundation, Inc., 59
 Temple Place - Suite 330, Boston, MA  02111-1307, USA.

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%*/

#ifndef TURBINEPLANTXML_H
#define TURBINEPLANTXML_H

#include "TurbinePlant.h"

namespace JSBSim {
class Element;

namespace TurbinePlant {

/** Reads a <plant> element into a configuration. Every quantity is SI and
    named with its unit; absent optional elements keep the documented
    defaults. Throws XMLLogException on a malformed or nonfinite value.
    The digest hashes the element's names, attributes and data so a
    captured state can be matched to the configuration that produced it. */
PlantConfig ReadPlantConfig(Element* plant, NumericalSettings& settings);

} // namespace TurbinePlant
} // namespace JSBSim

#endif
