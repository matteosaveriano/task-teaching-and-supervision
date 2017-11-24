# This file is generated automatically. Do not edit.
# Use project properties -> Build -> Qt -> Expert -> Custom Definitions.
TEMPLATE = app
DESTDIR = dist/Debug/GNU-Linux-x86
TARGET = AlfredDialogueManager
VERSION = 1.0.0
CONFIG -= debug_and_release app_bundle lib_bundle
CONFIG += debug 
PKGCONFIG +=
QT = core gui
SOURCES += tinyxml2.cpp POMDPState.cpp Observ.cpp FusionEngine.cpp main.cpp POMDP.cpp Mouse.cpp POMDPTrainer.cpp DialogFlow.cpp FlowState.cpp POMDPTest.cpp
HEADERS += POMDP.h FlowState.h FusionEngine.h Observ.h Mouse.h POMDPTest.h POMDPState.h tinyxml2.h DialogFlow.h POMDPTrainer.h Param.h
FORMS +=
RESOURCES +=
TRANSLATIONS +=
OBJECTS_DIR = build/Debug/GNU-Linux-x86
MOC_DIR = 
RCC_DIR = 
UI_DIR = 
QMAKE_CC = gcc
QMAKE_CXX = g++
DEFINES += 
INCLUDEPATH += 
LIBS += 
