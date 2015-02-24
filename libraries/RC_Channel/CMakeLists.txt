cmake_minimum_required(VERSION 2.8)
project(ArduLibrary)

SET( RC_Channel
  RC_Channel
  RC_Channel_aux
)

ADD_LIBRARY ( RC_Channel STATIC ${RC_Channel} )