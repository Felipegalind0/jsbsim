set(SOURCES FGElectric.cpp
            FGEngine.cpp
            FGForce.cpp
            FGNozzle.cpp
            FGPiston.cpp
            FGPropeller.cpp
            FGRocket.cpp
            FGTank.cpp
            FGThruster.cpp
            FGTurbine.cpp
            FGTurbinePlant.cpp
            FGTurboProp.cpp
            FGTransmission.cpp
            FGRotor.cpp
            FGBrushLessDCMotor.cpp
            plant/TurbinePlantGas.cpp
            plant/TurbinePlantComponents.cpp
            plant/TurbinePlant.cpp
            plant/TurbinePlantXML.cpp)

set(HEADERS FGElectric.h
            FGEngine.h
            FGForce.h
            FGNozzle.h
            FGPiston.h
            FGPropeller.h
            FGRocket.h
            FGTank.h
            FGThruster.h
            FGTurbine.h
            FGTurboProp.h
            FGTransmission.h
            FGRotor.h
            FGBrushLessDCMotor.h)

set(PLANT_HEADERS plant/TurbinePlantGas.h
                  plant/TurbinePlantComponents.h
                  plant/TurbinePlant.h
                  plant/TurbinePlantXML.h)

add_library(Propulsion OBJECT ${SOURCES})

if(CXXTEST_FOUND)
    add_coverage(Propulsion)
endif(CXXTEST_FOUND)

install(FILES ${HEADERS} DESTINATION include/JSBSim/models/propulsion
                         COMPONENT devel)
install(FILES ${PLANT_HEADERS} DESTINATION include/JSBSim/models/propulsion/plant
                               COMPONENT devel)
