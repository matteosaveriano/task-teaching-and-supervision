cmake_minimum_required(VERSION 2.8.3)

project(lwr_rviz_visualization)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS xacro)

file(GLOB def_files ${CMAKE_CURRENT_SOURCE_DIR}/defs/*.xml
                    ${CMAKE_CURRENT_SOURCE_DIR}/robots/*groups*.xml)

file(GLOB xacros ${CMAKE_CURRENT_SOURCE_DIR}/robots/*.urdf.xml)

set(outfiles "")

foreach( _xacro ${xacros} )
  string(REPLACE ".urdf." ".expanded." _outfile ${_xacro})
  add_custom_command(
    OUTPUT ${_outfile}
    COMMAND xacro
    ARGS ${_xacro} > ${_outfile}
    DEPENDS ${_xacro} ${def_files})
  list(APPEND outfiles ${_outfile})
endforeach( _xacro)

add_custom_target( expanded_defs ALL DEPENDS ${outfiles} )
