# ⚡ Power-Aware Structural Mixed State Codes (PA-SMSC) for FPGA FSM Optimization

[![Verilog](https://img.shields.io/badge/RTL-Verilog-blue)]()
[![Python](https://img.shields.io/badge/EDA-Python-green)]()
[![FPGA](https://img.shields.io/badge/Target-Xilinx%20Virtex--7-orange)]()
[![License](https://img.shields.io/badge/License-MIT-yellow)]()

## 📖 Project Overview
This project presents an advanced hardware optimization methodology for FPGA-based Mealy Finite State Machines (FSMs). Building upon the theoretical foundation of **Mixed State Codes (MSC)** (Barkalov et al., IEEE Access 2024), this repository introduces **PA-SMSC**: a comprehensive, automated EDA toolchain that guarantees a strict **2-level LUT architecture** while simultaneously minimizing dynamic power consumption and chip area.

Unlike standard behavioral RTL implementations that suffer from unpredictable multi-level logic flattening during synthesis, PA-SMSC utilizes **Explicit Structural Verilog** and **Graph-Based Clique Partitioning** to mathematically guarantee optimal Look-Up Table (LUT) utilization.

## 🚀 Key Innovations & Methodological Extensions
This project extends the base MSC methodology with four critical industry-grade upgrades:

1. **Advanced Graph-Based Clique Partitioning:** Replaces the base paper's greedy bin-packing algorithm with a `networkx`-based Compatibility Graph approach. By extracting maximal cliques based on node degrees, it mathematically guarantees the absolute minimum number of partition classes ($K$), minimizing Level-2 multiplexer overhead.
2. **Switching-Activity Aware State Assignment:** Introduces a Transition Frequency Matrix to minimize the **Hamming distance** between frequently transitioning states *within* each partition class. This directly reduces the switching activity ($\alpha$) in the dynamic power equation ($P_{dynamic} = \alpha C V^2 f$).
3. **Explicit Structural RTL Enforcement:** Abandons standard behavioral `case(state)` statements. The architecture is explicitly coded into **Level-1 (Partial Function Generators)** and **Level-2 (MUX-based Class Selection)**. This forces the synthesis tool (Vivado) to honor the theoretical 2-level LUT depth, maximizing $f_{max}$.
4. **Automated Python EDA Toolchain:** Features a custom Python script that parses FSM specifications (CSV), runs the graph-partitioning algorithm, and **auto-generates the structural Verilog RTL**, eliminating human transcription errors and mirroring modern Hardware-Software Co-Design flows.

## 🏗️ Architecture: The 2-Level LUT Guarantee
```text
[ FSM Inputs (X) ] + [ Partial State Codes (S) ]
          │
          ▼
  ┌───────────────────────────────────────┐
  │ LEVEL 1: Partial Function Generators  │  <-- 1 LUT Level
  │ (LUTer_0, LUTer_1 ... LUTer_K)        │      (L_k + R_k <= 6)
  └───────────────────────────────────────┘
          │ (Partial Next-State & Outputs)
          ▼
  ┌───────────────────────────────────────┐
  │ LEVEL 2: Final Value Generator (MUX)  │  <-- 1 LUT Level
  │ (Selects active class via Class Code) │      (R_V + K <= 6)
  └───────────────────────────────────────┘
          │
          ▼
[ Final Next-State (D) ] & [ Final Outputs (Y) ]