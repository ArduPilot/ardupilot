cmake_minimum_required(VERSION 2.8)
project(ArduLibrary)

SET( AP_Terrain
  AP_Terrain
  TerrainGCS
  TerrainIO
  TerrainMission
  TerrainUtil
)

ADD_LIBRARY ( AP_Terrain STATIC ${AP_Terrain} )