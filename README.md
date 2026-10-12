# femtoRV32

A Verilog implementation of a 32-bit RISC-V processor. The project explores processor datapath design, pipelining, hazard handling, and simulation-based verification, with the Nexys A7 trainer kit as the intended FPGA target.

## Overview

The repository contains a single-cycle CPU implementation, a pipelined CPU design, supporting Verilog modules, and test benches for checking individual instruction groups and pipeline behavior.

The project focuses on:

- Instruction fetch, decode, execution, memory access, and register write-back.
- A 32-bit datapath and 32 general-purpose integer registers.
- Arithmetic and logical operations, immediate instructions, branches, jumps, and load/store operations.
- Forwarding and load-use hazard detection in the pipelined design.
- Instruction and data memory interactions, including the structural constraints of shared memory resources.
- Simulation tests for instruction behavior and selected pipeline hazards.

## Architecture

The project uses a three-stage pipeline distributed across six clock phases:

| Stage | Operations |
| --- | --- |
| Stage 0 | Instruction fetch (C0) and register read (C1) |
| Stage 1 | ALU execution (C0) and memory access (C1) |
| Stage 2 | Register write-back (C0) |

The processor is built from modules for the program counter, instruction and data memory, register file, immediate generation, control logic, ALU and ALU control, branch handling, pipeline registers, and forwarding/hazard management.

## Instruction coverage

The report states that the processor implements **42 user-level instructions from the RV32I base integer instruction set**. The documented and tested instruction categories include:

- **Arithmetic and logic:** `ADD`, `SUB`, `XOR`, `AND`, `OR`, and shift operations.
- **Immediate operations:** arithmetic, logical, comparison, and shift-immediate instructions.
- **Branches:** `BEQ`, `BNE`, `BLT`, `BGE`, `BLTU`, and `BGEU`.
- **Loads and stores:** byte, half-word, and word accesses, including signed and unsigned loads.
- **Control flow:** `JAL` and `JALR`.
- **Upper-immediate operations:** `LUI` and `AUIPC`.

Instruction support and behavior should be checked against the relevant CPU module and test bench, as the repository includes both single-cycle and pipelined implementations.

### Project-specific instruction behavior

The report describes `ECALL` as a halt instruction and `EBREAK`, `PAUSE`, `FENCE`, and `FENCE.TSO` as no-ops for the project. These are implementation-specific choices for this coursework, not general statements about how every RISC-V system handles these instructions.

## Repository structure

```text
.
├── verilog files/
│   ├── CPU.v                         # Single-cycle CPU
│   ├── pipelined_cpu.v               # Pipelined CPU top-level
│   ├── alu.v
│   ├── alu_control.v
│   ├── branch_unit.v
│   ├── data_mem.v
│   ├── instr_mem.v
│   ├── main_control_unit.v
│   ├── forwarding_unit.v
│   ├── hazard_unti.v
│   ├── Register_file.v
│   ├── ImmGen.v
│   └── ...                           # Other supporting and display modules
├── test_benches/                     # Focused Verilog simulation test benches
│   ├── branch_test.v
│   ├── fetch_test.v
│   ├── forward_test.v
│   ├── i_type.v
│   ├── jump_test.v
│   ├── load_type.v
│   ├── mem_test.v
│   └── r_type.v
└── real_test/                        # Assembly programs and test benches
    ├── Branch/
    ├── Forwarding/
    ├── IF/
    ├── I_type/
    ├── Jump/
    ├── Load/
    ├── Overall/
    ├── R_type/
    └── Store/
```

## Testing

Tests are organized in two groups:

- **`test_benches/`** contains focused benches for fetch, branch behavior, forwarding, jumps, instruction types, and memory operations.
- **`real_test/`** contains assembly test programs and Verilog benches for individual features, including branch handling, forwarding, instruction fetch, immediate instructions, jumps, loads, stores, and an overall test.

The Milestone 3 report also documents two broader programs. One checks conditional branching and jump behavior; the other exercises arithmetic, logic, shifts, memory operations, and upper-immediate instructions. The report compares simulated register and memory states with their expected values.

## FPGA target

The project targets the **Digilent Nexys A7 trainer kit**. The pipelined top-level module includes LED and seven-segment display signals for observing selected internal values. Deploying the design to the board requires the relevant Vivado project setup, pin constraints, and board configuration.

## Project documentation

The accompanying **Milestone 3 Report – femtoRV32: RISC-V Processor Implementation and Testing** (CSCE 3301, Fall 2025) describes the datapath, pipeline organization, instruction coverage, test programs, and simulation results.

## Contributors

- Abdelrahman AboIsmaiel
- Omar Khalil

## References

- [RISC-V Specifications](https://riscv.org/technical/specifications/)
- [Source repository](https://github.com/omarramy22/-femtoRV32)
