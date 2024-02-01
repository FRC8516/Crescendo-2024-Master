# CRESCENDO v2024.1

FRC 8516 Wired Up

## Description

Drive Train: swerve drivetrain that uses REV MAXSwerve Modules.
- drivetrain composed of four MAXSwerve Modules, each configured with two SPARKS MAX, a NEO as the driving motor, a NEO 550 as the turning motor, and a REV Through Bore Encoder as the absolute turning encoder.

To get started, make sure you have calibrated the zero offsets for the absolute encoders in the Hardware Client using the `Absolute Encoder` tab under the associated turning SPARK MAX devices.

Actuators: Falcon 500, using Pigeon 2.
-Motion Magic for routines for intake and shooter positions.

## Prerequisites
* WPI 2024.2.1
* SPARK MAX Firmware v1.6.3 - Adds features that are required for swerve
* REVLib v2024.2.- - Includes APIs for the new firmware features
* Phoenix 6 v24.1.0 -- Includes APIs for firmware features ^Changed to Phoenix 6 api

## Configuration

It is possible that this project will not work for your robot right out of the box. Various things like the CAN IDs, PIDF gains, chassis configuration, etc. must be determined for your own robot!

These values can be adjusted in the `Constants.java` file.
