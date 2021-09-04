project(BinPickingUI)

set(NACHISOCKET_INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR} CACHE PATH "")

add_library(NachiSocket SHARED STATIC NachiSocket.cpp NachiSocket.h)

