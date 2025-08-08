# 🚦 Traffic-Informed Symbolic Regression (TI-SR)

This repository implements **Traffic-Informed Symbolic Regression (TI-SR)**, a method that incorporates traffic-related metrics into symbolic regression to improve model interpretability and performance.  

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
| **`GenerateCRMatrixIDM1-4.m`** | MATLAB script to generate driving scenarios and compute traffic-related metrics. **Run this first** before running `SymbolicRegressionCode.py`. |
| **`SymbolicRegressionCode.py`** | Python script to execute the Traffic-Informed Symbolic Regression (TI-SR) using PySR. |
| **`GeneratePySRTestTrajectory.py`** | Python script to compute trajectory-related metrics for testing scenarios. |
| **`PySRResultAnalysis.m`** | MATLAB script to analyze TI-SR results and calculate additional traffic-related metrics. |


