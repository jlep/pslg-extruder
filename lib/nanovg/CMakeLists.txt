cmake_minimum_required(VERSION 3.10)
project(nanovg C)

set(CMAKE_C_STANDARD 11)

add_library(nanovg STATIC
        fontstash.h
        nanovg.c
        nanovg.h
        nanovg_gl.h
        nanovg_gl_utils.h
        stb_image.h
        stb_truetype.h)
