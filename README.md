# Digital Twin-Driven Trajectory and Resource Optimization for UAV Swarms in Low-Altitude Urban Environments: Addressing Position Uncertainty and 3D Building Blockage

## Overview
This repository contains the MATLAB simulation code for our research. We model Unmanned Aerial Vehicle (UAV) communication and trajectory planning in urban environments. The system uses a digital twin framework. The framework handles 3D building blockages and user position uncertainties.

## Data Privacy Statement
We replaced the real urban map data with randomly generated synthetic data. We did this to protect privacy. The synthetic data ensures the code runs correctly. The code produces valid academic results without exposing sensitive locations.

## File Descriptions
We provide four MATLAB scripts. These scripts reproduce the main results.

* **`dt226.m`**: This script runs Monte Carlo simulations. The script compares the average outage probability of our scheme against baseline algorithms.
* **`dt304.m`** and **`dttrajactory.m`**: These scripts simulate the 3D urban environment. The scripts perform DBSCAN clustering. The scripts plan the 3D UAV flight paths. The scripts also generate the 2D signal outage probability heatmaps.
* **`dtsecond.m`**: This script calculates the normalized dual-metric performance. The script compares the outage cost and the flight time. The script finds the optimal balance.

## Requirements
You need MATLAB to run the code. We tested the code with standard MATLAB toolboxes. You do not need extra external datasets. The scripts generate the synthetic urban environments automatically.

## How to Run
You can open any file in your MATLAB software. You click the run button. The software will execute the simulation. The software will generate the figures.
