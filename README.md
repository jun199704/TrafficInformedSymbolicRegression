# 🚦 Traffic-Informed Symbolic Regression (TI-SR)

This repository implements **Traffic-Informed Symbolic Regression (TI-SR)**, a traffic informed symbolic regression approach to improve model interpretability and performance.  

---

## 📌 Prerequisites

Before running the code, make sure the following are installed:

- **[PySR](https://github.com/MilesCranmer/PySR)** – Symbolic regression library for Python and Julia  
- **MATLAB** – For traffic scenario and metric generation  
- **Julia** – Required by PySR  

> **Note:** Replace the `population.jl` file in your PySR installation with the **customized `population.jl`** provided in this repository.

---

## 📂 Repository Structure

| File | Description |
|------|-------------|
| **`GenerateCR/GenerateCRMatrixIDM1-4.m`** | MATLAB script to generate driving scenarios and compute traffic-related metrics. **Run this first** before running `SymbolicRegressionCode.py`. |
| **`TISR/SymbolicRegressionCode.py`** | Python script to execute the Traffic-Informed Symbolic Regression (TI-SR) using PySR. This code will load the dataset and random split it for training and testing. |
| **`TISR/GeneratePySRTestTrajectory.py`** | Python script to compute trajectory-related metrics for testing scenarios. |
| **`TISR/PySRResultAnalysis.m`** | MATLAB script to analyze TI-SR results and calculate additional traffic-related metrics. |


