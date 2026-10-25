# PA-SMSC: Power-Aware Structural Mixed State Codes for FPGA FSMs

**Base paper:** A. Barkalov et al., "Hardware Reduction for FSMs With Extended
State Codes," IEEE Access, vol. 12, pp. 42369–42384, 2024.
DOI: 10.1109/ACCESS.2024.3376472

An extension of the Mixed State Code (MSC) methodology: graph-based state
partitioning, switching-activity-aware encoding, and explicit structural RTL
that preserves a two-level LUT architecture after synthesis.

## 1. Problem

Modern FPGA LUTs accept only 6 inputs. When an FSM's next-state and output
functions exceed that limit, the synthesizer chains LUTs together, increasing
logic depth, routing congestion, area and power. Standard behavioral RTL
(`case(state)`) gives the designer no control over this mapping.

## 2. What we adopt from the base paper

- Partition the internal state set into classes of compatible states.
- Encode each state as a Class Code V concatenated with a Partial State Code S.
- Enforce the set-based constraint: L_k + R_k <= S_L, with S_L = 6.
- Two-level structure: Level 1 partial function generators, Level 2 class MUX.

## 3. Our extensions

| Extension | Base paper | This project |
|---|---|---|
| Partitioning | Greedy heuristic | Compatibility graph + degree-guided clique extraction (networkx) |
| Encoding | Sequential partial codes | Transition-frequency-weighted, minimum-Hamming-distance assignment |
| RTL | Theoretical 2-level model | Explicit structural RTL that constrains synthesis to 2 levels |
| Validation | Academic benchmarks | 32-state AMR navigation / safety / power supervisor |
| Tooling | Manual flow | CSV -> partition -> encode -> RTL generation in Python |

## 4. Target FSM: Autonomous Mobile Robot supervisor

- States: 32 (navigation, safety/fault, power/idle)
- Inputs: 12 (LiDAR, bumper, IMU, battery, thermal, nav commands, feedback)
- Outputs: 12 (motor enables, brake, alarm, charger, fault/status codes)
- Target device: Xilinx 7-series, 6-input LUT

### Partitioning output (actual run)

| Class | States | L_k | R_k | L_k + R_k |
|---|---|---|---|---|
| 0 | 4 | 4 | 2 | 6 |
| 1 | 8 | 3 | 3 | 6 |
| 2 | 4 | 4 | 2 | 6 |
| 3 | 4 | 4 | 2 | 6 |
| 4 | 3 | 4 | 2 | 6 |
| 5 | 2 | 4 | 1 | 5 |
| 6 | 4 | 4 | 2 | 6 |
| 7 | 2 | 3 | 1 | 4 |
| 8 | 1 | 4 | 1 | 5 |

K = 9, R_V = 4, R_S = 3, R_EC = 7 bits. All classes satisfy L_k + R_k <= 6.

## 5. Architecture

    Inputs X + Partial State Code S
               |
    +----------v---------------------------+
    | LEVEL 1: partial function generators |  one LUT level per class
    | LUTer_0 ... LUTer_8                  |  L_k + R_k <= 6
    +----------+---------------------------+
               | partial D / partial Y
    +----------v---------------------------+
    | LEVEL 2: class-code multiplexer      |  one LUT level
    +----------+---------------------------+
               |
          next state D, outputs Y

## 6. Repository layout

    amr_fsm.csv                  FSM transition specification
    pa_smsc_toolchain.py         Graph partitioning + power-aware encoding
    verilog_generator.py         CSV -> structural Verilog generator
    amr_pa_smsc_structural.v     RTL (9 Level-1 blocks + Level-2 MUX)
    amr_fsm_simple_tb.v          Directed verification testbench
    README.md                    This file

## 7. Reproduce

Python (macOS / Linux):

    python3 -m venv venv
    source venv/bin/activate
    pip install networkx
    python pa_smsc_toolchain.py
    python verilog_generator.py

Simulation (Icarus Verilog):

    brew install icarus-verilog gtkwave
    iverilog -o simv amr_pa_smsc_structural.v amr_fsm_simple_tb.v
    vvp simv
    gtkwave dump.vcd

Synthesis: open a Vivado project on a Windows/Linux host, add the RTL as
design source and the testbench as simulation source, target a 7-series
device, then run Synthesis and collect utilization, timing and power reports.

## 8. Verification status

Directed tests passing in Icarus Verilog:

    PASS  reset -> a1
    PASS  a1  -> a2   (navigation command asserted)
    PASS  a2  -> a3   (front obstacle detected)
    PASS  a3  -> a13  (bumper contact)
    PASS  a13 -> a2   (obstacle cleared)

Known limitation: the prototype transition table contains overlapping input
predicates. The RTL resolves these with first-match priority and holds state
when no predicate matches. A complete reachability-aware coverage pass and
Vivado PPA characterization are the next milestones.

## 9. Target metrics (not yet measured)

Compared against binary / Gray baselines on the same device:

- Strict two-level combinational depth
- Lower LUT count via minimized class count
- Lower switching activity via intra-class Hamming-distance minimization
- Higher f_max from bounded logic depth

No silicon or Vivado PPA numbers are claimed until synthesis is completed.

## 10. Reference

[1] A. Barkalov, L. Titarenko, K. Mielcarek, M. Mazurkiewicz,
    "Hardware Reduction for FSMs With Extended State Codes,"
    IEEE Access, vol. 12, 2024.