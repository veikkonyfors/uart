cmake_minimum_required(VERSION 3.10)

# Set some basic project attributes
project (uart
	VERSION 0.1
	DESCRIPTION "Lib for UART functions")

# This project will output a library
add_library(uart STATIC uart.c)

# Create a simple configuration header
configure_file(config.h.in config.h)

# Include the configuration header in the build
target_include_directories(${PROJECT_NAME} PUBLIC "${PROJECT_BINARY_DIR}")
