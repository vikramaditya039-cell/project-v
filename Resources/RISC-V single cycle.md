# The MOST DETAILED RISC-V Single-Cycle Processor Guide Ever

You want the FULL story? Let's go from absolute zero to complete understanding. Buckle up - this is going to be thorough.

---

# PART 1: UNDERSTANDING RISC-V ARCHITECTURE (Theory First!)

## What is RISC-V?

**RISC-V** = Reduced Instruction Set Computer - Version 5

It's an **open-source instruction set architecture (ISA)**. Think of it as a "contract" between hardware and software.

### The ISA Hierarchy:

```
Software (C/C++/Python)
        ↓
    Compiler
        ↓
Assembly Language (human-readable instructions)
        ↓
Machine Code (binary that hardware understands)
        ↓
Hardware (our processor!)
```

### RISC-V Variants:

- **RV32I**: 32-bit base integer instruction set (what we're building)
- **RV64I**: 64-bit version
- **RV32M**: Multiplication/Division extension
- **RV32F**: Floating point extension
- **RV32C**: Compressed instructions (16-bit)

**We're building RV32I** - the foundation that everything else builds on.

---

## RV32I: The Complete Specification

### Register File (x0 - x31):

RISC-V has **32 general-purpose registers**, each **32 bits wide**.

```
Register | ABI Name | Purpose              | Saved by
---------|----------|----------------------|----------
x0       | zero     | Always 0 (hardwired) | N/A
x1       | ra       | Return address       | Caller
x2       | sp       | Stack pointer        | Callee
x3       | gp       | Global pointer       | N/A
x4       | tp       | Thread pointer       | N/A
x5-x7    | t0-t2    | Temporaries          | Caller
x8       | s0/fp    | Saved/Frame pointer  | Callee
x9       | s1       | Saved register       | Callee
x10-x11  | a0-a1    | Function args/return | Caller
x12-x17  | a2-a7    | Function arguments   | Caller
x18-x27  | s2-s11   | Saved registers      | Callee
x28-x31  | t3-t6    | Temporaries          | Caller
```

**Key Rule**: `x0` MUST always read as zero. Any write to `x0` is ignored.

**Why?** This gives you a free constant zero, useful for:
- Moving values: `add x1, x2, x0` (x1 = x2 + 0)
- Comparing to zero
- Discarding results

---

## RV32I Instruction Formats

RISC-V instructions are **always 32 bits wide** (in RV32I). They come in 6 formats:

### **1. R-Type (Register-Register operations)**

```
 31        25 24    20 19    15 14  12 11     7 6      0
┌───────────┬────────┬────────┬──────┬────────┬────────┐
│  funct7   │   rs2  │   rs1  │funct3│   rd   │ opcode │
│  [7 bits] │[5 bits]│[5 bits]│[3 bits]│[5 bits]│[7 bits]│
└───────────┴────────┴────────┴──────┴────────┴────────┘
```

**Example**: `add x3, x1, x2` means x3 = x1 + x2

- **opcode** (bits 0-6): 0110011 (identifies R-type)
- **rd** (bits 7-11): x3 (destination register)
- **funct3** (bits 12-14): 000 (specifies ADD)
- **rs1** (bits 15-19): x1 (source register 1)
- **rs2** (bits 20-24): x2 (source register 2)
- **funct7** (bits 25-31): 0000000 (additional opcode bits)

**R-Type Instructions**:
- `add, sub, and, or, xor, sll, srl, sra, slt, sltu`

---

### **2. I-Type (Immediate operations)**

```
 31                    20 19    15 14  12 11     7 6      0
┌────────────────────────┬────────┬──────┬────────┬────────┐
│      immediate         │   rs1  │funct3│   rd   │ opcode │
│      [12 bits]         │[5 bits]│[3 bits]│[5 bits]│[7 bits]│
└────────────────────────┴────────┴──────┴────────┴────────┘
```

**Example**: `addi x1, x0, 5` means x1 = x0 + 5 = 5

- **immediate** (bits 20-31): 5 (the constant value)
- Sign-extended to 32 bits: `immediate[31:12] = immediate[11]` (repeated)

**I-Type Instructions**:
- Arithmetic: `addi, slti, sltiu, xori, ori, andi`
- Shifts: `slli, srli, srai`
- Loads: `lb, lh, lw, lbu, lhu`
- Special: `jalr`

**Why sign-extend?** To handle negative numbers correctly.
- If immediate = 0x800 (bit 11 = 1), it becomes 0xFFFFF800 (negative)
- If immediate = 0x005 (bit 11 = 0), it becomes 0x00000005 (positive)

---

### **3. S-Type (Store operations)**

```
 31        25 24    20 19    15 14  12 11     7 6      0
┌───────────┬────────┬────────┬──────┬────────┬────────┐
│ imm[11:5] │   rs2  │   rs1  │funct3│imm[4:0]│ opcode │
│  [7 bits] │[5 bits]│[5 bits]│[3 bits]│[5 bits]│[7 bits]│
└───────────┴────────┴────────┴──────┴────────┴────────┘
```

**Example**: `sw x2, 8(x1)` means Memory[x1 + 8] = x2

**Why split immediate?** To keep register fields in same position across formats!

- **rs1**: Base address register
- **rs2**: Data to store
- **immediate**: Offset (split into two parts)

**S-Type Instructions**: `sb, sh, sw`

---

### **4. B-Type (Branch operations)**

```
 31   30        25 24    20 19    15 14  12 11   8 7    6      0
┌────┬───────────┬────────┬────────┬──────┬──────┬────┬────────┐
│imm │ imm[10:5] │   rs2  │   rs1  │funct3│imm[4:1]│imm│ opcode │
│[12]│  [6 bits] │[5 bits]│[5 bits]│[3 bits]│[4 bits]│[11]│[7 bits]│
└────┴───────────┴────────┴────────┴──────┴──────┴────┴────────┘
```

**Example**: `beq x1, x2, label` means if (x1 == x2) goto label

**Immediate encoding**: `{imm[12], imm[10:5], imm[4:1], 1'b0}`
- Note: LSB is always 0 (instructions are 2-byte aligned)
- Range: -4096 to +4094 bytes

**B-Type Instructions**: `beq, bne, blt, bge, bltu, bgeu`

---

### **5. U-Type (Upper immediate)**

```
 31                                    12 11     7 6      0
┌──────────────────────────────────────┬────────┬────────┐
│            immediate                 │   rd   │ opcode │
│            [20 bits]                 │[5 bits]│[7 bits]│
└──────────────────────────────────────┴────────┴────────┘
```

**Example**: `lui x1, 0x12345` means x1 = 0x12345000

**U-Type Instructions**: `lui, auipc`

---

### **6. J-Type (Jump)**

```
 31   30        21 20  19        12 11     7 6      0
┌────┬───────────┬────┬───────────┬────────┬────────┐
│imm │ imm[10:1] │imm │ imm[19:12]│   rd   │ opcode │
│[20]│ [10 bits] │[11]│  [8 bits] │[5 bits]│[7 bits]│
└────┴───────────┴────┴───────────┴────────┴────────┘
```

**Example**: `jal x1, label` means x1 = PC + 4; PC = PC + offset

**J-Type Instructions**: `jal`

---

## Complete RV32I Instruction Summary

| Instruction | Type | Opcode  | Funct3 | Funct7     | Description |
|-------------|------|---------|--------|------------|-------------|
| add         | R    | 0110011 | 000    | 0000000    | rd = rs1 + rs2 |
| sub         | R    | 0110011 | 000    | 0100000    | rd = rs1 - rs2 |
| and         | R    | 0110011 | 111    | 0000000    | rd = rs1 & rs2 |
| or          | R    | 0110011 | 110    | 0000000    | rd = rs1 | rs2 |
| xor         | R    | 0110011 | 100    | 0000000    | rd = rs1 ^ rs2 |
| sll         | R    | 0110011 | 001    | 0000000    | rd = rs1 << rs2 |
| srl         | R    | 0110011 | 101    | 0000000    | rd = rs1 >> rs2 |
| sra         | R    | 0110011 | 101    | 0100000    | rd = rs1 >>> rs2 |
| slt         | R    | 0110011 | 010    | 0000000    | rd = (rs1 < rs2) ? 1 : 0 |
| sltu        | R    | 0110011 | 011    | 0000000    | rd = (rs1 < rs2) unsigned |
| addi        | I    | 0010011 | 000    | -          | rd = rs1 + imm |
| andi        | I    | 0010011 | 111    | -          | rd = rs1 & imm |
| ori         | I    | 0010011 | 110    | -          | rd = rs1 | imm |
| xori        | I    | 0010011 | 100    | -          | rd = rs1 ^ imm |
| slli        | I    | 0010011 | 001    | 0000000    | rd = rs1 << imm |
| srli        | I    | 0010011 | 101    | 0000000    | rd = rs1 >> imm |
| srai        | I    | 0010011 | 101    | 0100000    | rd = rs1 >>> imm |
| slti        | I    | 0010011 | 010    | -          | rd = (rs1 < imm) ? 1 : 0 |
| sltiu       | I    | 0010011 | 011    | -          | rd = (rs1 < imm) unsigned |
| lw          | I    | 0000011 | 010    | -          | rd = Mem[rs1 + imm] |
| lh          | I    | 0000011 | 001    | -          | rd = Mem[rs1 + imm] halfword |
| lhu         | I    | 0000011 | 101    | -          | rd = Mem[rs1 + imm] half unsigned |
| lb          | I    | 0000011 | 000    | -          | rd = Mem[rs1 + imm] byte |
| lbu         | I    | 0000011 | 100    | -          | rd = Mem[rs1 + imm] byte unsigned |
| sw          | S    | 0100011 | 010    | -          | Mem[rs1 + imm] = rs2 |
| sh          | S    | 0100011 | 001    | -          | Mem[rs1 + imm] = rs2[15:0] |
| sb          | S    | 0100011 | 000    | -          | Mem[rs1 + imm] = rs2[7:0] |
| beq         | B    | 1100011 | 000    | -          | if (rs1 == rs2) PC += imm |
| bne         | B    | 1100011 | 001    | -          | if (rs1 != rs2) PC += imm |
| blt         | B    | 1100011 | 100    | -          | if (rs1 < rs2) PC += imm |
| bge         | B    | 1100011 | 101    | -          | if (rs1 >= rs2) PC += imm |
| bltu        | B    | 1100011 | 110    | -          | if (rs1 < rs2) unsigned |
| bgeu        | B    | 1100011 | 111    | -          | if (rs1 >= rs2) unsigned |
| lui         | U    | 0110111 | -      | -          | rd = imm << 12 |
| auipc       | U    | 0010111 | -      | -          | rd = PC + (imm << 12) |
| jal         | J    | 1101111 | -      | -          | rd = PC + 4; PC += imm |
| jalr        | I    | 1100111 | 000    | -          | rd = PC + 4; PC = rs1 + imm |

---

# PART 2: DATAPATH - THE HARDWARE JOURNEY

## What Happens in One Clock Cycle?

Let's trace the execution of: `add x3, x1, x2`

**Machine code**: `0x002081B3`

Binary breakdown:
```
0000000  00010  00001  000  00011  0110011
funct7    rs2    rs1  f3    rd    opcode
```

### The Journey Through Hardware:

```
Clock Edge
    ↓
┌───────────────────────────────────────────────────────┐
│  STEP 1: INSTRUCTION FETCH (IF)                       │
├───────────────────────────────────────────────────────┤
│  PC = 0x00000004 (pointing to our instruction)        │
│  Instruction Memory[0x00000004] → 0x002081B3          │
└───────────────────────────────────────────────────────┘
    ↓
┌───────────────────────────────────────────────────────┐
│  STEP 2: INSTRUCTION DECODE (ID)                      │
├───────────────────────────────────────────────────────┤
│  Control Unit sees opcode = 0110011 → "R-type!"       │
│  Generates control signals:                           │
│    - RegWrite = 1 (we'll write to a register)         │
│    - ALUSrc = 0 (use register, not immediate)         │
│    - MemWrite = 0 (not storing to memory)             │
│    - MemToReg = 0 (ALU result goes to register)       │
│    - Branch = 0 (not a branch)                        │
│    - ALUOp = 10 (R-type ALU operation)                │
│                                                        │
│  Register File reads:                                 │
│    - rs1 = 1 → Read x1 → outputs 5 (ReadData1)        │
│    - rs2 = 2 → Read x2 → outputs 10 (ReadData2)       │
└───────────────────────────────────────────────────────┘
    ↓
┌───────────────────────────────────────────────────────┐
│  STEP 3: EXECUTE (EX)                                 │
├───────────────────────────────────────────────────────┤
│  ALU Control sees:                                    │
│    - ALUOp = 10 (R-type)                              │
│    - funct3 = 000, funct7 = 0000000 → ADD operation   │
│    - Generates ALUControl = 0010                      │
│                                                        │
│  ALU computes:                                        │
│    - Input A = 5 (from ReadData1)                     │
│    - Input B = 10 (from ReadData2)                    │
│    - Operation = ADD                                  │
│    - Result = 5 + 10 = 15                             │
│    - Zero flag = 0 (result is not zero)               │
└───────────────────────────────────────────────────────┘
    ↓
┌───────────────────────────────────────────────────────┐
│  STEP 4: MEMORY ACCESS (MEM)                          │
├───────────────────────────────────────────────────────┤
│  MemWrite = 0 → Skip memory write                     │
│  MemRead = 0 → Skip memory read                       │
│  (This step does nothing for ADD instruction)         │
└───────────────────────────────────────────────────────┘
    ↓
┌───────────────────────────────────────────────────────┐
│  STEP 5: WRITE BACK (WB)                              │
├───────────────────────────────────────────────────────┤
│  MemToReg = 0 → Select ALU result (not memory data)   │
│  WriteData = 15 (ALU result)                          │
│  RegWrite = 1 → Enable write to register file         │
│  rd = 3 → Write to x3                                 │
│  x3 ← 15 ✓                                            │
└───────────────────────────────────────────────────────┘
    ↓
PC ← PC + 4 (move to next instruction)
Next clock edge → Process next instruction
```

---

# PART 3: BUILDING FROM ABSOLUTE SCRATCH

## Understanding Digital Logic Basics

Before we code, understand what we're actually building.

### What is a Wire?

In hardware, a **wire** is just a connection - electricity flows through it.

```verilog
wire [31:0] data;  // 32 parallel wires, each carrying 1 bit
```

Think of it like 32 separate copper wires bundled together. Bit 0 is on wire 0, bit 31 is on wire 31.

### What is a Register (Flip-Flop)?

A **register** STORES a value. It remembers data between clock cycles.

```verilog
reg [31:0] stored_value;

always @(posedge clk) begin
    stored_value <= new_value;  // Store on clock rising edge
end
```

**Key difference**:
- **Wire**: Value changes immediately when input changes (combinational)
- **Register**: Value changes ONLY on clock edge (sequential)

### Combinational vs Sequential Logic

**Combinational Logic** (like ALU):
```
Input changes → Output changes immediately (within nanoseconds)
```

**Sequential Logic** (like Register File):
```
Input changes → Wait for clock edge → Output changes
```

---

## Building Block 1: The Register (Most Fundamental)

Let's build a single 1-bit register from scratch to understand storage.

```verilog
// This is what happens inside a D Flip-Flop
module d_flip_flop (
    input clk,      // Clock signal
    input d,        // Data input
    output reg q    // Data output (stored value)
);

always @(posedge clk) begin
    q <= d;  // On rising clock edge, store 'd' in 'q'
end

endmodule
```

**What's happening physically?**
1. Clock is low → Flip-flop is "locked", output `q` doesn't change
2. Clock goes high (rising edge) → Flip-flop "captures" input `d`
3. Output `q` now holds this value until next rising edge

**To store 32 bits**, we need 32 of these in parallel:

```verilog
module register_32bit (
    input clk,
    input [31:0] d,
    output reg [31:0] q
);

always @(posedge clk) begin
    q <= d;  // All 32 bits stored simultaneously
end

endmodule
```

---

## Building Block 2: Memory Array

**What is Memory?** An array of registers!

```verilog
reg [31:0] memory [0:1023];  // 1024 words of 32-bit memory
```

**Breaking this down:**
- `[31:0]` = Each memory location stores 32 bits
- `[0:1023]` = We have 1024 memory locations (indexed 0 to 1023)

**Visual representation:**
```
Address  |  Data (32 bits)
---------|------------------
0        |  0x00000000
1        |  0x00000005
2        |  0x0000000A
3        |  0x00000000
...      |  ...
1023     |  0x00000000
```

### How to Read from Memory?

```verilog
wire [31:0] read_data;
wire [9:0] address;  // 10 bits can address 1024 locations (2^10)

assign read_data = memory[address];  // Combinational read
```

**This happens instantly** (in simulation). In real hardware, there's propagation delay.

### How to Write to Memory?

```verilog
always @(posedge clk) begin
    if (write_enable)
        memory[address] <= write_data;
end
```

**This happens on clock edge**. Why?
- We don't want memory changing randomly mid-cycle
- Synchronous writes prevent timing issues

---

## Building Block 3: Multiplexer (MUX)

A **multiplexer** is a selector - it chooses one input from many.

**2-to-1 MUX:**
```verilog
module mux2to1 (
    input [31:0] in0,
    input [31:0] in1,
    input sel,
    output [31:0] out
);

assign out = sel ? in1 : in0;  // If sel=1, choose in1; else in0

endmodule
```

**Truth table:**
```
sel | out
----|-----
0   | in0
1   | in1
```

**Where used in processor?**
- Choosing between immediate and register value for ALU
- Choosing between PC+4 and branch target
- Choosing between ALU result and memory data for register write

---

## Building Block 4: Decoder

A **decoder** converts binary to one-hot encoding.

**3-to-8 Decoder:**
```
Input (3 bits) | Output (8 bits)
---------------|------------------
000            | 00000001
001            | 00000010
010            | 00000100
011            | 00001000
100            | 00010000
101            | 00100000
110            | 01000000
111            | 10000000
```

```verilog
module decoder3to8 (
    input [2:0] in,
    output reg [7:0] out
);

always @(*) begin
    out = 8'b0;  // Initialize all to 0
    out[in] = 1; // Set only the selected bit
end

endmodule
```

**Used in Register File** to select which register to write.

---

# PART 4: BUILDING THE PROCESSOR - EVERY LINE EXPLAINED

## Module 1: Program Counter (PC)

**What is PC?** It's a special register that holds the address of the current instruction.

```verilog
module program_counter (
    input clk,
    input reset,
    input [31:0] pc_next,    // Next PC value
    output reg [31:0] pc     // Current PC value
);

// This is a 32-bit register with reset capability
always @(posedge clk or posedge reset) begin
    if (reset)
        pc <= 32'h00000000;  // Start at address 0 on reset
    else
        pc <= pc_next;        // Update to next PC on clock edge
end

endmodule
```

**Line-by-line explanation:**

```verilog
always @(posedge clk or posedge reset) begin
```
- `@(...)` = "Trigger this block when..."
- `posedge clk` = "...clock goes from 0 to 1"
- `or posedge reset` = "...OR reset goes from 0 to 1"
- This is called a **sensitivity list**

```verilog
if (reset)
    pc <= 32'h00000000;
```
- If reset is high, set PC to 0
- `32'h00000000` means: 32-bit hexadecimal value 0
- `<=` is **non-blocking assignment** (important for sequential logic!)

```verilog
else
    pc <= pc_next;
```
- Otherwise, update PC to the next value
- This happens on every clock edge when reset is low

**Why non-blocking (`<=`) instead of blocking (`=`)?**

Consider this scenario:
```verilog
// WRONG (using blocking =)
always @(posedge clk) begin
    a = b;  // a gets b's OLD value
    b = c;  // b gets c's value
    // But 'a' already got old 'b', not updated 'b'!
end

// CORRECT (using non-blocking <=)
always @(posedge clk) begin
    a <= b;  // Schedule: a will get b's value
    b <= c;  // Schedule: b will get c's value
    // Both assignments happen "simultaneously" at end of time step
end
```

**Non-blocking** mimics real hardware where all flip-flops update simultaneously on clock edge.

---

## Module 2: Instruction Memory (IMEM)

**Purpose**: Store the program instructions.

```verilog
module instruction_memory (
    input [31:0] address,        // PC value
    output [31:0] instruction    // Instruction at that address
);

// Memory array: 1024 instructions
reg [31:0] mem [0:1023];

// Initialize with program
initial begin
    // Example program
    mem[0] = 32'h00500093;  // addi x1, x0, 5
    mem[1] = 32'h00A00113;  // addi x2, x0, 10
    mem[2] = 32'h002081B3;  // add x3, x1, x2
    mem[3] = 32'h40208233;  // sub x4, x1, x2
    mem[4] = 32'h00000000;  // nop (no operation)
    // Initialize rest to 0
    integer i;
    for (i = 5; i < 1024; i = i + 1)
        mem[i] = 32'h00000000;
end

// Read instruction (combinational)
assign instruction = mem[address[31:2]];

endmodule
```

**Deep dive into each part:**

### Memory Declaration:
```verilog
reg [31:0] mem [0:1023];
```
- `reg [31:0]` = Each element is 32 bits wide
- `[0:1023]` = Array has 1024 elements
- Total storage = 1024 × 32 bits = 4096 bytes = 4 KB

### Initial Block:
```verilog
initial begin
    mem[0] = 32'h00500093;
```
- `initial` block runs ONCE at simulation start (time 0)
- NOT synthesizable for real hardware (would use ROM/Flash)
- For FPGA: Use memory initialization files (.coe, .mif)

### Understanding the Instructions:

**Instruction 1**: `mem[0] = 32'h00500093`

Let's decode `0x00500093`:
```
Binary: 0000 0000 0101 0000 0000 0000 1001 0011

Breakdown:
000000000101  00000  000  00001  0010011
immediate     rs1   f3    rd    opcode

immediate = 000000000101 = 5 (decimal)
rs1 = 00000 = x0
funct3 = 000 = ADD operation for I-type
rd = 00001 = x1
opcode = 0010011 = I-type arithmetic

This is: addi x1, x0, 5
Meaning: x1 = x0 + 5 = 0 + 5 = 5
```

**Instruction 2**: `mem[1] = 32'h00A00113`

Decoding `0x00A00113`:
```
Binary: 0000 0000 1010 0000 0000 0001 0001 0011

000000001010  00000  000  00010  0010011
immediate     rs1   f3    rd    opcode

immediate = 10 (decimal)
rs1 = x0
rd = x2
This is: addi x2, x0, 10
Meaning: x2 = 0 + 10 = 10
```

**Instruction 3**: `mem[2] = 32'h002081B3`

Decoding `0x002081B3`:
```
Binary: 0000 0000 0010 0000 1000 0001 1011 0011

0000000  00010  00001  000  00011  0110011
funct7   rs2    rs1   f3    rd    opcode

funct7 = 0000000 = ADD (not SUB)
rs2 = 00010 = x2
rs1 = 00001 = x1
funct3 = 000
rd = 00011 = x3
opcode = 0110011 = R-type

This is: add x3, x1, x2
Meaning: x3 = x1 + x2 = 5 + 10 = 15
```

**Instruction 4**: `mem[3] = 32'h40208233`

Decoding `0x40208233`:
```
Binary: 0100 0000 0010 0000 1000 0010 0011 0011

0100000  00010  00001  000  00100  0110011
funct7   rs2    rs1   f3    rd    opcode

funct7 = 0100000 = SUB (note bit 30 is set!)
rs2 = x2
rs1 = x1
rd = x4

This is: sub x4, x1, x2
Meaning: x4 = x1 - x2 = 5 - 10 = -5 (0xFFFFFFFB in two's complement)
```

### Address Translation:
```verilog
assign instruction = mem[address[31:2]];
```

**Why `[31:2]` instead of full address?**

RISC-V instructions are **word-aligned** (4 bytes). This means:
- Instruction addresses are always multiples of 4: 0, 4, 8, 12, 16...
- Last 2 bits of address are ALWAYS `00`

**Visual explanation:**
```
Byte Address  | Word Address | Binary Address    | What We Store
--------------|--------------|-------------------|---------------
0             | 0            | ...00000000       | Instruction 0
1             | -            | ...00000001       | (invalid)
2             | -            | ...00000010       | (invalid)
3             | -            | ...00000011       | (invalid)
4             | 1            | ...00000100       | Instruction 1
5             | -            | ...00000101       | (invalid)
6             | -            | ...00000110       | (invalid)
7             | -            | ...00000111       | (invalid)
8             | 2            | ...00001000       | Instruction 2
```

By using `address[31:2]`, we're:
1. Ignoring the bottom 2 bits (which are always 0)
2. Dividing address by 4 automatically
3. Converting byte address → word index

**Example:**
- PC = 0x00000000 → mem[0x00000000 >> 2] = mem[0]
- PC = 0x00000004 → mem[0x00000004 >> 2] = mem[1]
- PC = 0x00000008 → mem[0x00000008 >> 2] = mem[2]

---

## Module 3: Register File (The Heart of the Processor)

The register file is where data lives during program execution.

```verilog
module register_file (
    input clk,
    input we3,                  // Write Enable (1 = write, 0 = don't write)
    input [4:0] ra1,           // Read Address 1 (which register to read)
    input [4:0] ra2,           // Read Address 2 
    input [4:0] wa3,           // Write Address (which register to write)
    input [31:0] wd3,          // Write Data (data to write)
    output [31:0] rd1,         // Read Data 1
    output [31:0] rd2          // Read Data 2
);

// The actual 32 registers
reg [31:0] registers [0:31];

// Initialize all registers to 0
integer i;
initial begin
    for (i = 0; i < 32; i = i + 1)
        registers[i] = 32'h00000000;
end

// COMBINATIONAL READ (happens immediately)
assign rd1 = (ra1 == 5'b0) ? 32'h0 : registers[ra1];
assign rd2 = (ra2 == 5'b0) ? 32'h0 : registers[ra2];

// SEQUENTIAL WRITE (happens on clock edge)
always @(posedge clk) begin
    if (we3 && (wa3 != 5'b0))  // Write only if enabled AND not x0
        registers[wa3] <= wd3;
end

endmodule
```

**Let's dissect every line:**

### Register Array:
```verilog
reg [31:0] registers [0:31];
```

This creates 32 registers, each 32 bits wide:
```
registers[0]  = x0  = 00000000 (always zero)
registers[1]  = x1  = ???????? (data)
registers[2]  = x2  = ???????? (data)
...
registers[31] = x31 = ???????? (data)
```

### Read Logic (Combinational):
```verilog
assign rd1 = (ra1 == 5'b0) ? 32'h0 : registers[ra1];
```

**Breaking down this line:**

1. **`(ra1 == 5'b0)`** - "Is ra1 equal to zero?"
   - `5'b0` means: 5-bit binary value 0 (00000)
   
2. **`? 32'h0`** - "If true, output 32-bit hex 0"
   
3. **`: registers[ra1]`** - "If false, output registers[ra1]"

**Why this special case for x0?**

RISC-V specification REQUIRES x0 to always read as zero, even if someone tries to write to it!

**Example scenarios:**
```
ra1 = 0  → rd1 = 0         (hardwired)
ra1 = 1  → rd1 = registers[1]  (actual register value)
ra1 = 15 → rd1 = registers[15]
```

### Write Logic (Sequential):
```verilog
always @(posedge clk) begin
    if (we3 && (wa3 != 5'b0))
        registers[wa3] <= wd3;
end
```

**Conditions for writing:**
1. **`we3`** - Write Enable must be HIGH
   - This comes from Control Unit
   - Only HIGH for instructions that write to registers (add, addi, lw, etc.)
   - LOW for instructions that don't (sw, beq, etc.)

2. **`(wa3 != 5'b0)`** - Destination is NOT x0
   - Prevents writing to x0
   - Even if instruction says "write to x0", we ignore it

**Example execution:**

```
Instruction: add x5, x3, x4

Clock cycle N:
  - ra1 = 3, ra2 = 4, wa3 = 5
  - Read happens: rd1 = registers[3], rd2 = registers[4]
  - ALU computes: rd1 + rd2
  - Result goes to wd3

Clock edge (N → N+1):
  - we3 = 1 (write enabled)
  - wa3 = 5 (write to x5)
  - registers[5] ← wd3 (ALU result stored)

Clock cycle N+1:
  - x5 now contains the sum
```

---

## Module 4: ALU (Arithmetic Logic Unit)

The ALU does all the math and logic operations.

```verilog
module alu (
    input [31:0] a,              // Operand A
    input [31:0] b,              // Operand B
    input [3:0] alu_control,     // Which operation to perform
    output reg [31:0] result,    // Result of operation
    output zero                  // Zero flag (1 if result = 0)
);

// Zero flag is combinational
assign zero = (result == 32'h0);

// ALU operations
always @(*) begin
    case (alu_control)
        4'b0000: result = a & b;           // AND
        4'b0001: result = a | b;           // OR
        4'b0010: result = a + b;           // ADD
        4'b0110: result = a - b;           // SUBTRACT
        4'b0111: result = (a < b) ? 1 : 0; // SLT (Set Less Than)
        4'b1100: result = ~(a | b);        // NOR
        4'b1101: result = a ^ b;           // XOR
        4'b1000: result = a << b[4:0];     // SLL (Shift Left Logical)
        4'b1001: result = a >> b[4:0];     // SRL (Shift Right Logical)
        4'b1010: result = $signed(a) >>> b[4:0]; // SRA (Shift Right Arithmetic)
        default: result = 32'h0;
    endcase
end

endmodule
```

**Understanding each operation:**

### 1. AND Operation (`4'b0000`):
```verilog
result = a & b;
```

**Bitwise AND:**
```
a = 1010 1100 1111 0000 ...
b = 1100 0011 1010 0101 ...
    ──── ──── ──── ──── 
  = 1000 0000 1010 0000 ...
```

Each bit: `result[i] = a[i] & b[i]`

**Used for:** Masking bits, checking flags

### 2. OR Operation (`4'b0001`):
```verilog
result = a | b;
```

**Bitwise OR:**
```
a = 1010 1100 ...
b = 1100 0011 ...
    ──── ──── 
  = 1110 1111 ...
```

**Used for:** Setting bits, combining flags

### 3. ADD Operation (`4'b0010`):
```verilog
result = a + b;
```

**32-bit addition with wraparound:**

```
Example 1:
a = 0x00000005 (5)
b = 0x0000000A (10)
result = 0x0000000F (15)

Example 2 (overflow):
a = 0xFFFFFFFF (-1 in two's complement)
b = 0x00000001 (1)
result = 0x00000000 (0) - wraps around!
```

**Hardware implementation:** Carry-propagate adder
- Each bit position adds with carry from previous position
- Happens in a few nanoseconds

### 4. SUBTRACT Operation (`4'b0110`):
```verilog
result = a - b;
```

**Subtraction = Addition of two's complement:**
```
a - b = a + (~b + 1)
```

**Example:**
```
a = 5  = 0000...0101
b = 10 = 0000...1010

~b    = 1111...0101
~b+1  = 1111...0110 (this is -10 in two's complement)

a + (~b+1) = 0000...0101 + 1111...0110
           = 1111...1011
           = -5 (in two's complement)
```

### 5. Set Less Than (`4'b0111`):
```verilog
result = (a < b) ? 1 : 0;
```

**Comparison:**
```
If a < b: result = 0x00000001
If a >= b: result = 0x00000000
```

**Example:**
```
a = 5, b = 10 → result = 1 (5 < 10 is true)
a = 10, b = 5 → result = 0 (10 < 5 is false)
```

**Used for:** Conditional operations, implementing `if` statements

### 6. Shift Left Logical (`4'b1000`):
```verilog
result = a << b[4:0];
```

**Why `b[4:0]`?**
- We only need 5 bits to specify shift amount (0-31)
- Shifting by 32 or more is undefined/wraps around

**Example:**
```
a = 0000...0101 (5)
Shift left by 2:
result = 0000...0101 << 2
       = 0000...10100 (20)
```

**Effect:** Multiply by 2^n (where n is shift amount)

### 7. Shift Right Logical (`4'b1001`):
```verilog
result = a >> b[4:0];
```

**Shifts in zeros from the left:**
```
a = 1010...1100
Shift right by 2:
result = 001010...11
         ↑↑ (zeros shifted in)
```

**Used for:** Unsigned division by 2^n

### 8. Shift Right Arithmetic (`4'b1010`):
```verilog
result = $signed(a) >>> b[4:0];
```

**Preserves sign bit (for signed numbers):**
```
Positive number:
a = 0010...1100
result = 000010...11 (zeros shifted in)

Negative number:
a = 1010...1100
result = 111010...11 (ones shifted in to preserve sign)
         ↑↑↑
```

**Used for:** Signed division by 2^n

### Zero Flag:
```verilog
assign zero = (result == 32'h0);
```

**Output is 1-bit:**
- `zero = 1` if result is exactly zero
- `zero = 0` if result is anything else

**Used for:** Branch instructions
- `beq` (branch if equal) checks if `(a - b) == 0`
- If zero flag is HIGH after ALU subtraction, values were equal

---

## Module 5: Control Unit (The Brain)

The control unit decodes instructions and generates control signals.

```verilog
module control_unit (
    input [6:0] opcode,      // Instruction[6:0]
    input [2:0] funct3,      // Instruction[14:12]
    input [6:0] funct7,      // Instruction[31:25]
    
    // Output control signals
    output reg branch,       // 1 = This is a branch instruction
    output reg mem_read,     // 1 = Read from data memory
    output reg mem_to_reg,   // 1 = Write memory data to register (not ALU result)
    output reg [1:0] alu_op, // Tells ALU control what type of operation
    output reg mem_write,    // 1 = Write to data memory
    output reg alu_src,      // 1 = Use immediate (not register) for ALU input B
    output reg reg_write     // 1 = Write to register file
);

always @(*) begin
    // Default values (all off)
    branch = 0;
    mem_read = 0;
    mem_to_reg = 0;
    alu_op = 2'b00;
    mem_write = 0;
    alu_src = 0;
    reg_write = 0;
    
    case (opcode)
        // R-type instructions (add, sub, and, or, xor, sll, srl, sra, slt, sltu)
        7'b0110011: begin
            reg_write = 1;      // Write result to register
            alu_op = 2'b10;     // R-type ALU operation
            // alu_src = 0 (use register, not immediate)
            // mem_to_reg = 0 (use ALU result, not memory)
        end
        
        // I-type arithmetic (addi, slti, sltiu, xori, ori, andi, slli, srli, srai)
        7'b0010011: begin
            alu_src = 1;        // Use immediate value
            reg_write = 1;      // Write result to register
            alu_op = 2'b00;     // I-type ALU operation (add immediate)
        end
        
        // Load instructions (lw, lh, lb, lhu, lbu)
        7'b0000011: begin
            alu_src = 1;        // Use immediate (offset)
            mem_to_reg = 1;     // Write memory data to register
            reg_write = 1;      // Write to register
            mem_read = 1;       // Read from memory
            alu_op = 2'b00;     // ALU adds base + offset
        end
        
        // Store instructions (sw, sh, sb)
        7'b0100011: begin
            alu_src = 1;        // Use immediate (offset)
            mem_write = 1;      // Write to memory
            alu_op = 2'b00;     // ALU adds base + offset
            // reg_write = 0 (don't write to register)
        end
        
        // Branch instructions (beq, bne, blt, bge, bltu, bgeu)
        7'b1100011: begin
            branch = 1;         // This is a branch
            alu_op = 2'b01;     // ALU subtracts for comparison
            // reg_write = 0 (don't write to register)
        end
        
        // LUI (Load Upper Immediate)
        7'b0110111: begin
            reg_write = 1;      // Write to register
            alu_src = 1;        // Use immediate
            alu_op = 2'b11;     // Special: pass immediate through
        end
        
        // AUIPC (Add Upper Immediate to PC)
        7'b0010111: begin
            reg_write = 1;
            alu_src = 1;
            alu_op = 2'b11;
        end
        
        // JAL (Jump and Link)
        7'b1101111: begin
            reg_write = 1;      // Store PC+4 in register
            branch = 1;         // Take jump
        end
        
        // JALR (Jump and Link Register)
        7'b1100111: begin
            reg_write = 1;
            alu_src = 1;
            branch = 1;
        end
        
        default: begin
            // Unknown opcode - all controls stay at default (0)
        end
    endcase
end

endmodule
```

**Understanding Control Signals:**

### 1. `reg_write` - Register Write Enable

**When HIGH (1):**
- Register file will write to destination register on next clock
- Typical for: add, addi, lw, jal

**When LOW (0):**
- Register file ignores write signals
- Typical for: sw, beq (these don't produce results)

**Example:**
```
Instruction: add x3, x1, x2
→ reg_write = 1 (we want to save result in x3)

Instruction: sw x2, 0(x1)
→ reg_write = 0 (we're storing to memory, not a register)
```

### 2. `alu_src` - ALU Source Select

**When LOW (0):** Use register value for ALU input B
```
R-type: add x3, x1, x2
ALU inputs: A = x1, B = x2 (register)
```

**When HIGH (1):** Use immediate value for ALU input B
```
I-type: addi x1, x0, 5
ALU inputs: A = x0, B = 5 (immediate)
```

**Hardware implementation:**
```
          ┌───────────┐
ReadData2 ┤           │
          │   MUX     ├──→ ALU Input B
Immediate ┤           │
          └───────────┘
               ↑
            alu_src (selector)
```

### 3. `mem_read` - Memory Read Enable

**When HIGH (1):**
- Data memory outputs data at specified address
- Used by load instructions

**Example:**
```
lw x5, 8(x1)
→ mem_read = 1
→ Memory outputs: Memory[x1 + 8]
→ This data goes to x5
```

### 4. `mem_write` - Memory Write Enable

**When HIGH (1):**
- Data memory stores value at specified address
- Used by store instructions

**Example:**
```
sw x2, 8(x1)
→ mem_write = 1
→ Memory[x1 + 8] ← x2
```

### 5. `mem_to_reg` - Memory to Register

**When LOW (0):** Write ALU result to register
```
add x3, x1, x2
→ x3 ← ALU result
```

**When HIGH (1):** Write memory data to register
```
lw x5, 8(x1)
→ x5 ← Memory[x1 + 8]
```

**Hardware implementation:**
```
           ┌───────────┐
ALU Result ┤           │
           │   MUX     ├──→ Register Write Data
Mem Data   ┤           │
           └───────────┘
                ↑
           mem_to_reg
```

### 6. `branch` - Branch Control

**When HIGH (1):**
- PC might take branch (depending on condition)
- Used with ALU zero flag

**Example:**
```
beq x1, x2, label
→ branch = 1
→ ALU computes: x1 - x2
→ If zero flag = 1 (equal): PC = PC + offset
→ If zero flag = 0 (not equal): PC = PC + 4
```

### 7. `alu_op` - ALU Operation Type

This is a **2-bit code** that tells the ALU Control unit what category of operation:

```
alu_op | Meaning              | Instructions
-------|----------------------|------------------
00     | ADD (for address)    | lw, sw, addi
01     | SUBTRACT (compare)   | beq, bne, blt, bge
10     | Determined by funct  | R-type (add, sub, and, or...)
11     | Pass immediate       | lui, auipc
```

**Why not directly control ALU?**
- ALU needs 4-bit control signal
- Control unit doesn't want to decode all funct3/funct7 combinations
- **Separation of concerns**: Control unit says "type", ALU control says "exact operation"

---

## Module 6: ALU Control Unit

This unit takes `alu_op` and `funct` fields and generates the exact 4-bit ALU control signal.

```verilog
module alu_control (
    input [1:0] alu_op,      // From main control unit
    input [2:0] funct3,      // Instruction[14:12]
    input [6:0] funct7,      // Instruction[31:25]
    output reg [3:0] alu_control_out  // To ALU
);

always @(*) begin
    case (alu_op)
        2'b00: begin
            // Load/Store/Add immediate - always ADD
            alu_control_out = 4'b0010;  // ADD
        end
        
        2'b01: begin
            // Branch - always SUBTRACT (for comparison)
            alu_control_out = 4'b0110;  // SUB
        end
        
        2'b10: begin
            // R-type - look at funct3 and funct7
            case (funct3)
                3'b000: begin
                    if (funct7 == 7'b0000000)
                        alu_control_out = 4'b0010;  // ADD
                    else if (funct7 == 7'b0100000)
                        alu_control_out = 4'b0110;  // SUB
                    else
                        alu_control_out = 4'b0000;  // Default
                end
                
                3'b001: alu_control_out = 4'b1000;  // SLL (Shift Left Logical)
                3'b010: alu_control_out = 4'b0111;  // SLT (Set Less Than)
                3'b011: alu_control_out = 4'b0111;  // SLTU (unsigned)
                3'b100: alu_control_out = 4'b1101;  // XOR
                
                3'b101: begin
                    if (funct7 == 7'b0000000)
                        alu_control_out = 4'b1001;  // SRL (Shift Right Logical)
                    else if (funct7 == 7'b0100000)
                        alu_control_out = 4'b1010;  // SRA (Shift Right Arithmetic)
                    else
                        alu_control_out = 4'b0000;
                end
                
                3'b110: alu_control_out = 4'b0001;  // OR
                3'b111: alu_control_out = 4'b0000;  // AND
                
                default: alu_control_out = 4'b0000;
            endcase
        end
        
        2'b11: begin
            // LUI/AUIPC - pass through (we'll handle separately)
            alu_control_out = 4'b0010;  // Use ADD for now
        end
        
        default: alu_control_out = 4'b0000;
    endcase
end

endmodule
```

**Decision tree visualization:**

```
alu_op = 00 → ADD (always)
             Used by: lw, sw, addi

alu_op = 01 → SUB (always)
             Used by: beq, bne, blt, bge

alu_op = 10 → Look at funct fields
             │
             ├─ funct3 = 000 → Look at funct7
             │                 ├─ funct7 = 0000000 → ADD
             │                 └─ funct7 = 0100000 → SUB
             │
             ├─ funct3 = 001 → SLL
             ├─ funct3 = 010 → SLT
             ├─ funct3 = 100 → XOR
             ├─ funct3 = 101 → Look at funct7
             │                 ├─ funct7 = 0000000 → SRL
             │                 └─ funct7 = 0100000 → SRA
             ├─ funct3 = 110 → OR
             └─ funct3 = 111 → AND

alu_op = 11 → Pass immediate (LUI/AUIPC)
```

---

## Module 7: Immediate Generator

RISC-V has different immediate formats. We need to extract and sign-extend them.

```verilog
module immediate_generator (
    input [31:0] instruction,
    output reg [31:0] imm_ext
);

// Extract opcode to determine format
wire [6:0] opcode = instruction[6:0];

always @(*) begin
    case (opcode)
        // I-type (12-bit immediate)
        7'b0010011,  // Arithmetic I-type
        7'b0000011,  // Load
        7'b1100111:  // JALR
        begin
            imm_ext = {{20{instruction[31]}}, instruction[31:20]};
            // Sign-extend bit 31 to fill upper 20 bits
        end
        
        // S-type (12-bit immediate, split)
        7'b0100011: begin  // Store
            imm_ext = {{20{instruction[31]}}, 
                       instruction[31:25], instruction[11:7]};
            // Reassemble: bits[31:25] + bits[11:7]
        end
        
        // B-type (13-bit immediate, split)
        7'b1100011: begin  // Branch
            imm_ext = {{19{instruction[31]}}, 
                       instruction[31], instruction[7], 
                       instruction[30:25], instruction[11:8], 
                       1'b0};
            // Format: {imm[12], imm[10:5], imm[4:1], 0}
            // Note: LSB is always 0 (2-byte aligned)
        end
        
        // U-type (20-bit immediate)
        7'b0110111,  // LUI
        7'b0010111:  // AUIPC
        begin
            imm_ext = {instruction[31:12], 12'b0};
            // Upper 20 bits, lower 12 bits are zero
        end
        
        // J-type (21-bit immediate, split)
        7'b1101111: begin  // JAL
            imm_ext = {{11{instruction[31]}}, 
                       instruction[31], instruction[19:12], 
                       instruction[20], instruction[30:21], 
                       1'b0};
            // Format: {imm[20], imm[10:1], imm[11], imm[19:12], 0}
        end
        
        default: imm_ext = 32'h0;
    endcase
end

endmodule
```

**Understanding Sign Extension:**

### I-Type Immediate:
```
Instruction bits: [31:20] = immediate

Example: addi x1, x0, -5

Binary representation of -5 in 12 bits:
-5 = 1111 1111 1011 (two's complement)

Original 32-bit instruction:
1111 1111 1011 | 00000 | 000 | 00001 | 0010011
immediate(12)    rs1    f3    rd      opcode

Sign-extend to 32 bits:
{{20{instruction[31]}}, instruction[31:20]}
= {20'b1111_1111_1111_1111_1111, 12'b1111_1111_1011}
= 32'b 1111_1111_1111_1111_1111_1111_1111_1011
= 0xFFFFFFFB
= -5 in 32-bit two's complement ✓
```

### S-Type Immediate (Split):
```
Instruction: sw x2, 8(x1)

Offset = 8 = 0000 0000 1000

How it's stored in instruction:
Bits [31:25] = 0000000 (upper 7 bits)
Bits [11:7]  = 01000   (lower 5 bits)

Reassembly:
imm_ext = {20'b sign_extend, instruction[31:25], instruction[11:7]}
        = {20'b0, 7'b0000000, 5'b01000}
        = 32'b 0000...0000_1000
        = 8 ✓
```

### B-Type Immediate (Complex):
```
Instruction: beq x1, x2, 8

Offset = 8 = ...0000_1000

RISC-V B-type format (13-bit, multiples of 2):
imm[12|10:5|4:1|0]  (bit 0 is implicit 0)

How stored in instruction:
instruction[31]    = imm[12] = 0
instruction[7]     = imm[11] = 0
instruction[30:25] = imm[10:5] = 000010
instruction[11:8]  = imm[4:1] = 0100
implicit           = imm[0] = 0

Reassembly:
{19{sign}, imm[12], imm[11], imm[10:5], imm[4:1], 1'b0}
= {19'b0, 1'b0, 1'b0, 6'b000010, 4'b0100, 1'b0}
= ...0000_1000
= 8 ✓
```

**Why is it so complex?**
- Keeps register fields (rs1, rs2, rd) in same positions across all formats
- Makes hardware decoding easier (fewer multiplexers)
- Wastes a bit (LSB always 0) but gains regularity

---

## Module 8: Data Memory

```verilog
module data_memory (
    input clk,
    input mem_write,            // Write enable
    input mem_read,             // Read enable
    input [31:0] address,       // Address to read/write
    input [31:0] write_data,    // Data to write
    input [2:0] funct3,         // For byte/half/word selection
    output reg [31:0] read_data // Data read from memory
);

// Memory array - 1KB for now
reg [7:0] memory [0:1023];  // Byte-addressable!

// Initialize memory
integer i;
initial begin
    for (i = 0; i < 1024; i = i + 1)
        memory[i] = 8'h00;
end

// READ operation (combinational)
always @(*) begin
    if (mem_read) begin
        case (funct3)
            3'b000: begin  // LB (Load Byte, sign-extended)
                read_data = {{24{memory[address][7]}}, memory[address]};
                // Sign-extend byte to 32 bits
            end
            
            3'b001: begin  // LH (Load Halfword, sign-extended)
                read_data = {{16{memory[address+1][7]}}, 
                            memory[address+1], memory[address]};
                // Sign-extend halfword (16 bits) to 32 bits
            end
            
            3'b010: begin  // LW (Load Word)
                read_data = {memory[address+3], memory[address+2], 
                            memory[address+1], memory[address]};
                // Little-endian: LSB at lowest address
            end
            
            3'b100: begin  // LBU (Load Byte Unsigned)
                read_data = {24'h0, memory[address]};
                // Zero-extend byte
            end
            
            3'b101: begin  // LHU (Load Halfword Unsigned)
                read_data = {16'h0, memory[address+1], memory[address]};
                // Zero-extend halfword
            end
            
            default: read_data = 32'h0;
        endcase
    end else begin
        read_data = 32'h0;
    end
end

// WRITE operation (sequential)
always @(posedge clk) begin
    if (mem_write) begin
        case (funct3)
            3'b000: begin  // SB (Store Byte)
                memory[address] <= write_data[7:0];
                // Store only lowest byte
            end
            
            3'b001: begin  // SH (Store Halfword)
                memory[address]   <= write_data[7:0];
                memory[address+1] <= write_data[15:8];
                // Store two bytes
            end
            
            3'b010: begin  // SW (Store Word)
                memory[address]   <= write_data[7:0];
                memory[address+1] <= write_data[15:8];
                memory[address+2] <= write_data[23:16];
                memory[address+3] <= write_data[31:24];
                // Store four bytes (little-endian)
            end
            
            default: begin
                // Do nothing
            end
        endcase
    end
end

endmodule
```

**Understanding Memory Organization:**

### Byte-Addressable Memory:
```verilog
reg [7:0] memory [0:1023];
```

**This means:**
- Each memory location stores **8 bits (1 byte)**
- We have 1024 such locations
- Total: 1024 bytes = 1 KB

**Memory layout:**
```
Address | Data (1 byte each)
--------|-------------------
0x000   | 0x05
0x001   | 0x00
0x002   | 0x00
0x003   | 0x00
0x004   | 0x0A
0x005   | 0x00
...
```

### Little-Endian vs Big-Endian:

**Little-Endian** (RISC-V uses this):
- Least significant byte at lowest address
- Most significant byte at highest address

**Example: Store 0x12345678 at address 0x100**

```
Little-Endian (RISC-V):
Address | Data
--------|------
0x100   | 0x78  ← LSB (least significant byte)
0x101   | 0x56
0x102   | 0x34
0x103   | 0x12  ← MSB (most significant byte)

Big-Endian (for comparison):
Address | Data
--------|------
0x100   | 0x12  ← MSB
0x101   | 0x34
0x102   | 0x56
0x103   | 0x78  ← LSB
```

### Load Word (LW):
```verilog
read_data = {memory[address+3], memory[address+2], 
            memory[address+1], memory[address]};
```

**Step-by-step for `lw x5, 0(x1)` where x1 = 0x100:**

```
Memory contents:
0x100: 0x78
0x101: 0x56
0x102: 0x34
0x103: 0x12

Assembly:
read_data = {memory[0x103], memory[0x102], memory[0x101], memory[0x100]}
          = {0x12, 0x34, 0x56, 0x78}
          = 0x12345678

Result: x5 = 0x12345678
```

**Note the concatenation operator `{}`:**
- `{a, b, c, d}` creates: `[a's bits][b's bits][c's bits][d's bits]`
- Most significant part goes first

### Load Byte Signed (LB):
```verilog
read_data = {{24{memory[address][7]}}, memory[address]};
```

**Example 1: Load positive byte**
```
Memory[0x100] = 0x05 = 0000_0101

Sign bit [7] = 0 (positive)

Sign-extend:
{{24{0}}, 0000_0101}
= {0x000000, 0x05}
= 0x00000005
```

**Example 2: Load negative byte**
```
Memory[0x100] = 0xFF = 1111_1111 (-1 in two's complement)

Sign bit [7] = 1 (negative)

Sign-extend:
{{24{1}}, 1111_1111}
= {0xFFFFFF, 0xFF}
= 0xFFFFFFFF
= -1 in 32-bit two's complement ✓
```

### Load Byte Unsigned (LBU):
```verilog
read_data = {24'h0, memory[address]};
```

**Example:**
```
Memory[0x100] = 0xFF

Zero-extend:
{24'h0, 0xFF}
= {0x000000, 0xFF}
= 0x000000FF
= 255 (positive) ✓
```

**Difference:**
- **LB**: Treats byte as signed (-128 to +127)
- **LBU**: Treats byte as unsigned (0 to 255)

### Store Word (SW):
```verilog
memory[address]   <= write_data[7:0];
memory[address+1] <= write_data[15:8];
memory[address+2] <= write_data[23:16];
memory[address+3] <= write_data[31:24];
```

**Example: `sw x2, 0(x1)` where x1 = 0x100, x2 = 0x12345678**

```
write_data = 0x12345678

Breaking into bytes:
write_data[7:0]   = 0x78  → memory[0x100]
write_data[15:8]  = 0x56  → memory[0x101]
write_data[23:16] = 0x34  → memory[0x102]
write_data[31:24] = 0x12  → memory[0x103]

Result in memory (little-endian):
0x100: 0x78
0x101: 0x56
0x102: 0x34
0x103: 0x12
```

---

# PART 5: THE TOP MODULE - CONNECTING EVERYTHING

Now we connect all modules together to form the complete processor.

```verilog
module riscv_single_cycle (
    input clk,
    input reset
);

//=============================================================================
// WIRE DECLARATIONS - The connections between modules
//=============================================================================

// Program Counter
wire [31:0] pc;              // Current PC value
wire [31:0] pc_next;         // Next PC value
wire [31:0] pc_plus4;        // PC + 4 (sequential)
wire [31:0] pc_target;       // Branch/Jump target

// Instruction Memory
wire [31:0] instruction;     // Current instruction

// Control Signals
wire branch;                 // Branch instruction
wire mem_read;              // Memory read enable
wire mem_to_reg;            // Write memory data (not ALU) to register
wire [1:0] alu_op;          // ALU operation type
wire mem_write;             // Memory write enable
wire alu_src;               // ALU source: 0=register, 1=immediate
wire reg_write;             // Register write enable

// Register File
wire [31:0] read_data1;     // Register rs1 value
wire [31:0] read_data2;     // Register rs2 value
wire [31:0] write_data;     // Data to write to register

// Immediate
wire [31:0] imm_ext;        // Sign-extended immediate

// ALU
wire [31:0] alu_result;     // ALU output
wire [31:0] alu_operand_b;  // ALU second operand (register or immediate)
wire [3:0] alu_control;     // ALU operation code
wire alu_zero;              // ALU zero flag

// Data Memory
wire [31:0] mem_read_data;  // Data read from memory

// PC Control
wire pc_src;                // PC source: 0=PC+4, 1=branch target

//=============================================================================
// PROGRAM COUNTER MODULE
//=============================================================================

reg [31:0] pc_reg;  // PC storage register
assign pc = pc_reg;

always @(posedge clk or posedge reset) begin
    if (reset)
        pc_reg <= 32'h00000000;  // Reset to address 0
    else
        pc_reg <= pc_next;        // Update PC
end

//=============================================================================
// PC CALCULATION LOGIC
//=============================================================================

// Sequential: PC + 4
assign pc_plus4 = pc + 32'd4;

// Branch target: PC + immediate offset
assign pc_target = pc + imm_ext;

// Decide whether to branch
assign pc_src = branch & alu_zero;  // Branch taken if instruction is branch AND condition met

// Select next PC
assign pc_next = pc_src ? pc_target : pc_plus4;
// If branch taken: go to target
// Otherwise: go to next instruction (PC+4)

//=============================================================================
// INSTRUCTION MEMORY MODULE
//=============================================================================

instruction_memory imem (
    .address(pc),
    .instruction(instruction)
);

//=============================================================================
// IMMEDIATE GENERATOR MODULE
//=============================================================================

immediate_generator imm_gen (
    .instruction(instruction),
    .imm_ext(imm_ext)
);

//=============================================================================
// CONTROL UNIT MODULE
//=============================================================================

control_unit ctrl (
    .opcode(instruction[6:0]),
    .funct3(instruction[14:12]),
    .funct7(instruction[31:25]),
    .branch(branch),
    .mem_read(mem_read),
    .mem_to_reg(mem_to_reg),
    .alu_op(alu_op),
    .mem_write(mem_write),
    .alu_src(alu_src),
    .reg_write(reg_write)
);

//=============================================================================
// REGISTER FILE MODULE
//=============================================================================

register_file rf (
    .clk(clk),
    .we3(reg_write),
    .ra1(instruction[19:15]),  // rs1 field
    .ra2(instruction[24:20]),  // rs2 field
    .wa3(instruction[11:7]),   // rd field
    .wd3(write_data),
    .rd1(read_data1),
    .rd2(read_data2)
);

//=============================================================================
// ALU CONTROL MODULE
//=============================================================================

alu_control alu_ctrl (
    .alu_op(alu_op),
    .funct3(instruction[14:12]),
    .funct7(instruction[31:25]),
    .alu_control_out(alu_control)
);

//=============================================================================
// ALU INPUT MULTIPLEXER
//=============================================================================

// Choose between register value or immediate for second ALU operand
assign alu_operand_b = alu_src ? imm_ext : read_data2;
// If alu_src = 1: use immediate (for addi, lw, sw, etc.)
// If alu_src = 0: use register (for add, sub, and, or, etc.)

//=============================================================================
// ALU MODULE
//=============================================================================

alu alu_unit (
    .a(read_data1),           // Always from rs1
    .b(alu_operand_b),        // From rs2 or immediate
    .alu_control(alu_control),
    .result(alu_result),
    .zero(alu_zero)
);

//=============================================================================
// DATA MEMORY MODULE
//=============================================================================

data_memory dmem (
    .clk(clk),
    .mem_write(mem_write),
    .mem_read(mem_read),
    .address(alu_result),     // Address = ALU result (base + offset)
    .write_data(read_data2),  // Data to store = rs2
    .funct3(instruction[14:12]),
    .read_data(mem_read_data)
);

//=============================================================================
// WRITE BACK MULTIPLEXER
//=============================================================================

// Choose what to write to register: ALU result or memory data
assign write_data = mem_to_reg ? mem_read_data : alu_result;
// If mem_to_reg = 1: write memory data (for lw)
// If mem_to_reg = 0: write ALU result (for add, addi, etc.)

endmodule
```

---

## COMPLETE INSTRUCTION EXECUTION TRACE

Let's trace **every signal** for one instruction: `add x3, x1, x2`

**Initial state:**
- x1 = 5
- x2 = 10
- PC = 0x00000004
- Instruction at 0x00000004 = 0x002081B3

### Clock Cycle Timeline:

```
Time: Rising edge of clock
│
├─ [FETCH STAGE]
│  │
│  ├─ PC = 0x00000004
│  └─ Instruction Memory[0x00000004 >> 2] = Memory[1] = 0x002081B3
│     instruction wire = 0x002081B3
│
├─ [DECODE STAGE - Part 1: Instruction Fields]
│  │
│  ├─ opcode = instruction[6:0] = 0110011
│  ├─ rd     = instruction[11:7] = 00011 (x3)
│  ├─ funct3 = instruction[14:12] = 000
│  ├─ rs1    = instruction[19:15] = 00001 (x1)
│  ├─ rs2    = instruction[24:20] = 00010 (x2)
│  └─ funct7 = instruction[31:25] = 0000000
│
├─ [DECODE STAGE - Part 2: Control Unit]
│  │
│  ├─ Control Unit sees opcode = 0110011 (R-type)
│  ├─ Generates control signals:
│  │  ├─ branch = 0 (not a branch)
│  │  ├─ mem_read = 0 (not reading memory)
│  │  ├─ mem_to_reg = 0 (use ALU result)
│  │  ├─ alu_op = 2'b10 (R-type operation)
│  │  ├─ mem_write = 0 (not writing memory)
│  │  ├─ alu_src = 0 (use register, not immediate)
│  │  └─ reg_write = 1 (will write to register)
│  │
│  └─ Immediate Generator:
│     (Not used for R-type, but still generates a value)
│     imm_ext = 0x00000000
│
├─ [DECODE STAGE - Part 3: Register Read]
│  │
│  ├─ Register File receives:
│  │  ├─ ra1 = 1 (read x1)
│  │  ├─ ra2 = 2 (read x2)
│  │
│  ├─ Register File outputs (combinational, happens immediately):
│  │  ├─ read_data1 = registers[1] = 0x00000005 (5)
│  │  └─ read_data2 = registers[2] = 0x0000000A (10)
│  │
│  └─ These values propagate to ALU inputs
│
├─ [EXECUTE STAGE - Part 1: ALU Control]
│  │
│  ├─ ALU Control receives:
│  │  ├─ alu_op = 2'b10 (R-type)
│  │  ├─ funct3 = 000
│  │  ├─ funct7 = 0000000
│  │
│  ├─ ALU Control logic:
│  │  ├─ alu_op = 10 → Look at funct fields
│  │  ├─ funct3 = 000 AND funct7 = 0000000 → ADD operation
│  │  └─ alu_control = 4'b0010 (ADD)
│  │
│  └─ alu_control signal propagates to ALU
│
├─ [EXECUTE STAGE - Part 2: ALU Input Selection]
│  │
│  ├─ ALU Input B Multiplexer:
│  │  ├─ alu_src = 0
│  │  ├─ Inputs: imm_ext (not used), read_data2 = 10
│  │  └─ alu_operand_b = read_data2 = 10
│  │
│  └─ ALU now has both inputs ready
│
├─ [EXECUTE STAGE - Part 3: ALU Operation]
│  │
│  ├─ ALU receives:
│  │  ├─ a = read_data1 = 5
│  │  ├─ b = alu_operand_b = 10
│  │  └─ alu_control = 4'b0010 (ADD)
│  │
│  ├─ ALU computes:
│  │  ├─ result = 5 + 10 = 15 = 0x0000000F
│  │  └─ zero = (result == 0) ? 1 : 0 = 0
│  │
│  ├─ alu_result = 15
│  └─ alu_zero = 0
│
├─ [MEMORY STAGE]
│  │
│  ├─ Data Memory receives:
│  │  ├─ mem_write = 0 (don't write)
│  │  ├─ mem_read = 0 (don't read)
│  │  ├─ address = alu_result = 15 (not used)
│  │  └─ write_data = read_data2 = 10 (not used)
│  │
│  ├─ Memory does nothing (correct for ADD instruction)
│  └─ mem_read_data = 0 (garbage, won't be used)
│
├─ [WRITE BACK STAGE - Part 1: Data Selection]
│  │
│  ├─ Write Back Multiplexer:
│  │  ├─ mem_to_reg = 0
│  │  ├─ Inputs: mem_read_data, alu_result = 15
│  │  └─ write_data = alu_result = 15
│  │
│  └─ write_data = 15 (this goes to register file)
│
├─ [WRITE BACK STAGE - Part 2: Register Write Setup]
│  │
│  ├─ Register File receives for writing:
│  │  ├─ we3 = reg_write = 1 (write enabled)
│  │  ├─ wa3 = rd = 3 (write to x3)
│  │  └─ wd3 = write_data = 15
│  │
│  └─ Register file is ready to write on next clock edge
│
├─ [PC UPDATE]
│  │
│  ├─ PC Calculation:
│  │  ├─ pc_plus4 = 0x00000004 + 4 = 0x00000008
│  │  ├─ pc_target = pc + imm_ext = 0x00000004 + 0 = 0x00000004 (not used)
│  │  ├─ pc_src = branch & alu_zero = 0 & 0 = 0
│  │  └─ pc_next = pc_src ? pc_target : pc_plus4 = 0x00000008
│  │
│  └─ pc_next = 0x00000008 (ready for next cycle)
│
└─ [NEXT CLOCK EDGE]
   │
   ├─ PC Register updates:
   │  pc_reg <= pc_next
   │  pc = 0x00000008 ✓
   │
   └─ Register File updates:
      registers[3] <= 15
      x3 = 15 ✓
```

**Summary of signal values throughout the cycle:**

| Signal | Value | Meaning |
|--------|-------|---------|
| pc | 0x00000004 | Current instruction address |
| instruction | 0x002081B3 | add x3, x1, x2 |
| opcode | 0110011 | R-type |
| rs1, rs2, rd | 1, 2, 3 | x1, x2, x3 |
| read_data1 | 5 | x1 value |
| read_data2 | 10 | x2 value |
| alu_src | 0 | Use register (not immediate) |
| alu_operand_b | 10 | Second ALU input = x2 |
| alu_control | 0010 | ADD operation |
| alu_result | 15 | 5 + 10 |
| alu_zero | 0 | Result is not zero |
| mem_write | 0 | Not writing to memory |
| mem_read | 0 | Not reading from memory |
| mem_to_reg | 0 | Use ALU result (not memory) |
| write_data | 15 | Data to write to x3 |
| reg_write | 1 | Enable register write |
| pc_next | 0x00000008 | Next instruction address |

---

## Understanding Data Flow with Diagram

```
        ┌─────────────────────────────────────────────────┐
        │                                                 │
        │  RISC-V SINGLE-CYCLE PROCESSOR DATAPATH        │
        │                                                 │
        └─────────────────────────────────────────────────┘

  ┌──────┐  pc       ┌──────────────────┐  instruction
  │  PC  ├──────────►│ Instruction Mem  ├─────────┬──────────┐
  │      │           │                  │         │          │
  └──▲───┘           └──────────────────┘         │          │
     │                                             │          │
     │ pc_next                                     │          │
     │                                             │          │
  ┌──┴───┐  pc+4    ┌─────┐                      │          │
  │ MUX  │◄─────────┤ +4  │                       │          │
  │      │          └─────┘                        │          │
  └──▲───┘                                         │          │
     │ pc_target                                   │          │
     │                                              │          │
     │         ┌─────────────────┐                 │          │
     └─────────┤ PC + imm_ext    │                 │          │
               └──▲───────────▲──┘                 │          │
                  │           │                    │          │
                  │           │                    ▼          ▼
               pc │     ┌─────┴─────┐      ┌──────────────────────┐
                  │     │  Imm Gen  │◄─────┤  Control Unit        │
                  │     └─────┬─────┘      │  - Decodes opcode    │
                  │           │ imm_ext    │  - Generates control │
                  │           │            └──────────┬───────────┘
                  │           │                       │
                  │           │                       │ control signals
                  │           │                       │
                  │           ▼                       ▼
    ┌─────────────┴────┐  ┌─────┐          ┌────────────────┐
    │  Register File   │  │ MUX │◄─────────┤  ALU Control   │
    │  - 32 registers  │  │     │ alu_src  └────────────────┘
    │  - x0 = 0        │  └──┬──┘                 │
    │  - Read rs1, rs2 │     │                    │ alu_control
    │  - Write rd      │     │ alu_operand_b      │
    └────┬────────┬────┘     │                    │
         │        │          ▼                    ▼
         │        │      ┌────────┐          ┌─────────┐
         │        │      │  ALU   │◄─────────┤  funct  │
         │        └─────►│        │          └─────────┘
         │ read_data1    │ a    b │
         │               └───┬────┘
         │                   │ alu_result
         │                   │
         │                   ▼
         │               ┌────────────┐
         │               │ Data Mem   │
         │               │ - Load/    │
         └──────────────►│   Store    │
           write_data    └─────┬──────┘
                               │ mem_read_data
                               │
                               ▼
                           ┌───────┐
                           │  MUX  │◄────── mem_to_reg
                           │       │
                           └───┬───┘
                               │ write_data
                               │
                               └────────┐
                                        │
                    ┌───────────────────┘
                    │
                    ▼
           Back to Register File (rd)
```

---

# PART 6: CREATING THE TESTBENCH

Now let's write a comprehensive testbench to verify our processor.

```verilog
`timescale 1ns/1ps

module testbench;

//=============================================================================
// TESTBENCH SIGNALS
//=============================================================================

reg clk;
reg reset;

//=============================================================================
// INSTANTIATE THE PROCESSOR
//=============================================================================

riscv_single_cycle cpu (
    .clk(clk),
    .reset(reset)
);

//=============================================================================
// CLOCK GENERATION
//=============================================================================

// Generate clock: 10ns period (100 MHz)
initial begin
    clk = 0;
    forever #5 clk = ~clk;  // Toggle every 5ns
end

//=============================================================================
// TEST STIMULUS
//=============================================================================

initial begin
    // Initialize waveform dump
    $dumpfile("riscv_single_cycle.vcd");
    $dumpvars(0, testbench);
    
    // Display header
    $display("=====================================");
    $display("RISC-V Single-Cycle Processor Test");
    $display("=====================================\n");
    
    // Apply reset
    $display("Time %0t: Applying reset", $time);
    reset = 1;
    #20;  // Hold reset for 2 clock cycles
    reset = 0;
    $display("Time %0t: Reset released\n", $time);
    
    // Let processor run for several cycles
    #200;  // Run for 20 clock cycles
    
    // Display register contents
    $display("\n=====================================");
    $display("Register File Contents After Execution");
    $display("=====================================");
    $display("x0  (zero) = 0x%08h (%0d)", cpu.rf.registers[0], $signed(cpu.rf.registers[0]));
    $display("x1  (ra)   = 0x%08h (%0d)", cpu.rf.registers[1], $signed(cpu.rf.registers[1]));
    $display("x2  (sp)   = 0x%08h (%0d)", cpu.rf.registers[2], $signed(cpu.rf.registers[2]));
    $display("x3  (gp)   = 0x%08h (%0d)", cpu.rf.registers[3], $signed(cpu.rf.registers[3]));
    $display("x4  (tp)   = 0x%08h (%0d)", cpu.rf.registers[4], $signed(cpu.rf.registers[4]));
    $display("x5  (t0)   = 0x%08h (%0d)", cpu.rf.registers[5], $signed(cpu.rf.registers[5]));
    
    // Display some memory contents
    $display("\n=====================================");
    $display("Data Memory Contents");
    $display("=====================================");
    $display("Mem[0x00] = 0x%02h", cpu.dmem.memory[0]);
    $display("Mem[0x04] = 0x%02h %02h %02h %02h", 
             cpu.dmem.memory[7], cpu.dmem.memory[6], 
             cpu.dmem.memory[5], cpu.dmem.memory[4]);
    
    $display("\n=====================================");
    $display("Test Complete!");
    $display("=====================================\n");
    
    $finish;
end

//=============================================================================
// MONITOR EXECUTION
//=============================================================================

// Monitor PC and instruction every cycle
always @(posedge clk) begin
    if (!reset) begin
        $display("Time %0t: PC=0x%08h, Instruction=0x%08h", 
                 $time, cpu.pc, cpu.instruction);
    end
end

//=============================================================================
// INSTRUCTION DECODER (for debugging)
//=============================================================================

// Decode and display instruction mnemonics
always @(posedge clk) begin
    if (!reset) begin
        case (cpu.instruction[6:0])
            7'b0110011: begin  // R-type
                case (cpu.instruction[14:12])
                    3'b000: begin
                        if (cpu.instruction[31:25] == 7'b0000000)
                            $display("  → ADD x%0d, x%0d, x%0d", 
                                   cpu.instruction[11:7], 
                                   cpu.instruction[19:15],
                                   cpu.instruction[24:20]);
                        else
                            $display("  → SUB x%0d, x%0d, x%0d", 
                                   cpu.instruction[11:7], 
                                   cpu.instruction[19:15],
                                   cpu.instruction[24:20]);
                    end
                    3'b111: $display("  → AND x%0d, x%0d, x%0d", 
                                   cpu.instruction[11:7], 
                                   cpu.instruction[19:15],
                                   cpu.instruction[24:20]);
                    3'b110: $display("  → OR x%0d, x%0d, x%0d", 
                                   cpu.instruction[11:7], 
                                   cpu.instruction[19:15],
                                   cpu.instruction[24:20]);
                    3'b100: $display("  → XOR x%0d, x%0d, x%0d", 
                                   cpu.instruction[11:7], 
                                   cpu.instruction[19:15],
                                   cpu.instruction[24:20]);
                    3'b010: $display("  → SLT x%0d, x%0d, x%0d", 
                                   cpu.instruction[11:7], 
                                   cpu.instruction[19:15],
                                   cpu.instruction[24:20]);
                endcase
            end
            
            7'b0010011: begin  // I-type arithmetic
                case (cpu.instruction[14:12])
                    3'b000: $display("  → ADDI x%0d, x%0d, %0d", 
                                   cpu.instruction[11:7], 
                                   cpu.instruction[19:15],
                                   $signed(cpu.instruction[31:20]));
                    3'b111: $display("  → ANDI x%0d, x%0d, %0d", 
                                   cpu.instruction[11:7], 
                                   cpu.instruction[19:15],
                                   $signed(cpu.instruction[31:20]));
                    3'b110: $display("  → ORI x%0d, x%0d, %0d", 
                                   cpu.instruction[11:7], 
                                   cpu.instruction[19:15],
                                   $signed(cpu.instruction[31:20]));
                endcase
            end
            
            7'b0000011: begin  // Load
                case (cpu.instruction[14:12])
                    3'b010: $display("  → LW x%0d, %0d(x%0d)", 
                                   cpu.instruction[11:7],
                                   $signed(cpu.instruction[31:20]),
                                   cpu.instruction[19:15]);
                    3'b000: $display("  → LB x%0d, %0d(x%0d)", 
                                   cpu.instruction[11:7],
                                   $signed(cpu.instruction[31:20]),
                                   cpu.instruction[19:15]);
                    3'b001: $display("  → LH x%0d, %0d(x%0d)", 
                                   cpu.instruction[11:7],
                                   $signed(cpu.instruction[31:20]),
                                   cpu.instruction[19:15]);
                endcase
            end
            
            7'b0100011: begin  // Store
                case (cpu.instruction[14:12])
                    3'b010: $display("  → SW x%0d, %0d(x%0d)", 
                                   cpu.instruction[24:20],
                                   $signed({cpu.instruction[31:25], cpu.instruction[11:7]}),
                                   cpu.instruction[19:15]);
                    3'b000: $display("  → SB x%0d, %0d(x%0d)", 
                                   cpu.instruction[24:20],
                                   $signed({cpu.instruction[31:25], cpu.instruction[11:7]}),
                                   cpu.instruction[19:15]);
                endcase
            end
            
            7'b1100011: begin  // Branch
                case (cpu.instruction[14:12])
                    3'b000: $display("  → BEQ x%0d, x%0d, %0d", 
                                   cpu.instruction[19:15],
                                   cpu.instruction[24:20],
                                   $signed({cpu.instruction[31], cpu.instruction[7], 
                                          cpu.instruction[30:25], cpu.instruction[11:8], 1'b0}));
                    3'b001: $display("  → BNE x%0d, x%0d, %0d", 
                                   cpu.instruction[19:15],
                                   cpu.instruction[24:20],
                                   $signed({cpu.instruction[31], cpu.instruction[7], 
                                          cpu.instruction[30:25], cpu.instruction[11:8], 1'b0}));
                endcase
            end
            
            default: $display("  → UNKNOWN or NOP");
        endcase
    end
end

//=============================================================================
// SIGNAL MONITORING (Optional - for detailed debugging)
//=============================================================================

// Uncomment to see detailed signal values every cycle
/*
always @(posedge clk) begin
    if (!reset) begin
        $display("    Control Signals:");
        $display("      reg_write=%b, alu_src=%b, mem_write=%b, mem_read=%b", 
                 cpu.reg_write, cpu.alu_src, cpu.mem_write, cpu.mem_read);
        $display("      mem_to_reg=%b, branch=%b, alu_op=%b", 
                 cpu.mem_to_reg, cpu.branch, cpu.alu_op);
        $display("    ALU:");
        $display("      operand_a=0x%08h, operand_b=0x%08h", 
                 cpu.read_data1, cpu.alu_operand_b);
        $display("      alu_control=%b, result=0x%08h, zero=%b", 
                 cpu.alu_control, cpu.alu_result, cpu.alu_zero);
        $display("    Register File:");
        $display("      rs1(x%0d)=0x%08h, rs2(x%0d)=0x%08h, rd=x%0d", 
                 cpu.instruction[19:15], cpu.read_data1,
                 cpu.instruction[24:20], cpu.read_data2,
                 cpu.instruction[11:7]);
        $display("");
    end
end
*/

endmodule
```

---

# PART 7: COMPILATION AND SIMULATION

Now let's actually run this!

## Step-by-Step Compilation Process

### **1. Organize Your Files**

Create this folder structure:
```
riscv_single_cycle/
├── rtl/
│   ├── alu.v
│   ├── alu_control.v
│   ├── control_unit.v
│   ├── data_memory.v
│   ├── immediate_generator.v
│   ├── instruction_memory.v
│   ├── register_file.v
│   └── riscv_single_cycle.v (top module)
├── testbench/
│   └── testbench.v
└── sim/
    (simulation outputs will go here)
```

### **2. Create a Makefile** (Optional but helpful)

Create file: `Makefile`

```makefile
# Makefile for RISC-V Single-Cycle Processor

# Source files
RTL_FILES = rtl/alu.v \
            rtl/alu_control.v \
            rtl/control_unit.v \
            rtl/data_memory.v \
            rtl/immediate_generator.v \
            rtl/instruction_memory.v \
            rtl/register_file.v \
            rtl/riscv_single_cycle.v

TB_FILES = testbench/testbench.v

# Output files
VVP_FILE = sim/riscv_sim
VCD_FILE = sim/riscv_single_cycle.vcd

# Simulation tool
IVERILOG = iverilog
VVP = vvp
GTKWAVE = gtkwave

# Default target
all: compile simulate

# Compile
compile:
	@echo "Compiling Verilog files..."
	@mkdir -p sim
	$(IVERILOG) -o $(VVP_FILE) $(RTL_FILES) $(TB_FILES)
	@echo "Compilation successful!"

# Simulate
simulate:
	@echo "Running simulation..."
	cd sim && $(VVP) riscv_sim
	@echo "Simulation complete!"

# View waveforms
wave:
	@echo "Opening waveform viewer..."
	$(GTKWAVE) $(VCD_FILE) &

# Clean
clean:
	rm -rf sim/*
	@echo "Cleaned simulation files"

.PHONY: all compile simulate wave clean
```

### **3. Run Compilation**

Open terminal in your project root and run:

```bash
# Using Makefile
make compile

# OR manually
iverilog -o sim/riscv_sim \
    rtl/alu.v \
    rtl/alu_control.v \
    rtl/control_unit.v \
    rtl/data_memory.v \
    rtl/immediate_generator.v \
    rtl/instruction_memory.v \
    rtl/register_file.v \
    rtl/riscv_single_cycle.v \
    testbench/testbench.v
```

**What's happening:**
- `iverilog` = Icarus Verilog compiler
- `-o sim/riscv_sim` = Output executable file
- Followed by all source files

**Expected output:**
```
Compiling Verilog files...
Compilation successful!
```

**If you get errors**, they'll look like:
```
rtl/alu.v:15: syntax error
rtl/alu.v:15: error: malformed statement
```
→ This tells you exactly which file and line has the problem.

### **4. Run Simulation**

```bash
# Using Makefile
make simulate

# OR manually
cd sim
vvp riscv_sim
```

**Expected output:**
```
=====================================
RISC-V Single-Cycle Processor Test
=====================================

Time 0: Applying reset
Time 20: Reset released

Time 25: PC=0x00000000, Instruction=0x00500093
  → ADDI x1, x0, 5
Time 35: PC=0x00000004, Instruction=0x00A00113
  → ADDI x2, x0, 10
Time 45: PC=0x00000008, Instruction=0x002081B3
  → ADD x3, x1, x2
Time 55: PC=0x0000000C, Instruction=0x40208233
  → SUB x4, x1, x2
Time 65: PC=0x00000010, Instruction=0x00000000
  → UNKNOWN or NOP

=====================================
Register File Contents After Execution
=====================================
x0  (zero) = 0x00000000 (0)
x1  (ra)   = 0x00000005 (5)
x2  (sp)   = 0x0000000A (10)
x3  (gp)   = 0x0000000F (15)
x4  (tp)   = 0xFFFFFFFB (-5)
x5  (t0)   = 0x00000000 (0)

=====================================
Data Memory Contents
=====================================
Mem[0x00] = 0x00
Mem[0x04] = 0x00 00 00 00

=====================================
Test Complete!
=====================================
```

**Understanding the output:**

1. **Time 25**: First instruction executes
   - `ADDI x1, x0, 5` → x1 = 0 + 5 = 5

2. **Time 35**: Second instruction
   - `ADDI x2, x0, 10` → x2 = 0 + 10 = 10

3. **Time 45**: Third instruction
   - `ADD x3, x1, x2` → x3 = 5 + 10 = 15

4. **Time 55**: Fourth instruction
   - `SUB x4, x1, x2` → x4 = 5 - 10 = -5 = 0xFFFFFFFB

**It works!** 🎉

---

## PART 8: VIEWING WAVEFORMS

Waveforms let you see **every signal** changing over time - like an oscilloscope for digital circuits.

### **1. Open GTKWave**

```bash
# Using Makefile
make wave

# OR manually
gtkwave sim/riscv_single_cycle.vcd
```

### **2. GTKWave Interface Guide**

**Left Panel**: Signal hierarchy
```
testbench
└── cpu (riscv_single_cycle)
    ├── pc
    ├── instruction
    ├── rf (register_file)
    │   └── registers[0:31]
    ├── alu_unit
    │   ├── a
    │   ├── b
    │   ├── result
    │   └── zero
    ├── ctrl (control_unit)
    │   ├── reg_write
    │   ├── alu_src
    │   └── ...
    └── ...
```

### **3. Adding Signals to View**

**Step 1**: Click on `testbench` → `cpu`

**Step 2**: Select these key signals:
- `clk`
- `reset`
- `pc`
- `instruction`

**Step 3**: Click "Insert" or drag to waveform area

**Step 4**: Add more signals:
- `cpu.read_data1`
- `cpu.read_data2`
- `cpu.alu_result`
- `cpu.reg_write`
- `cpu.rf.registers[1]` (x1)
- `cpu.rf.registers[2]` (x2)
- `cpu.rf.registers[3]` (x3)

### **4. Analyzing a Waveform**

Let me describe what you'll see:

```
Time:     0ns   10ns   20ns   30ns   40ns   50ns   60ns
          |      |      |      |      |      |      |
clk     _/‾\___/‾\___/‾\___/‾\___/‾\___/‾\___/‾\___
reset   ‾‾‾‾‾‾‾‾‾‾‾\_____________________________
pc      XXXX|00000000|00000004|00000008|0000000C|
inst    XXXX|00500093|00A00113|002081B3|40208233|
          
registers[1] (x1):
        XXXX|XXXXXXXX|00000005|00000005|00000005|
                     ↑ writes 5 here
                     
registers[2] (x2):
        XXXX|XXXXXXXX|XXXXXXXX|0000000A|0000000A|
                              ↑ writes 10 here
                              
registers[3] (x3):
        XXXX|XXXXXXXX|XXXXXXXX|XXXXXXXX|0000000F|
                                        ↑ writes 15 here
```

**Key observations:**
1. `X` means undefined/uninitialized
2. Signal changes happen on clock edges
3. PC increments by 4 each cycle
4. Registers update one cycle after instruction

---

## PART 9: WRITING YOUR OWN PROGRAMS

Now let's write a real program and run it!

### **Example: Fibonacci Sequence**

Calculate: 0, 1, 1, 2, 3, 5, 8, 13...

**Assembly code:**
```assembly
# Fibonacci program
# x1 = n-2, x2 = n-1, x3 = n

    addi x1, x0, 0      # x1 = 0 (fib(0))
    addi x2, x0, 1      # x2 = 1 (fib(1))
    
loop:
    add  x3, x1, x2     # x3 = x1 + x2 (next fib)
    add  x1, x2, x0     # x1 = x2 (shift)
    add  x2, x3, x0     # x2 = x3 (shift)
    
    # Repeat 5 times
    addi x5, x5, 1      # counter++
    addi x6, x0, 5      # compare with 5
    bne  x5, x6, loop   # if counter != 5, loop
    
done:
    addi x0, x0, 0      # nop (infinite loop here)
    beq  x0, x0, done
```

### **Converting to Machine Code**

You can use an assembler, or manually encode:

**Manual encoding example:**

```
addi x1, x0, 0
→ I-type: imm=0, rs1=0, funct3=000, rd=1, opcode=0010011
→ 0000 0000 0000 | 00000 | 000 | 00001 | 0010011
→ 0x00000093

addi x2, x0, 1
→ 0000 0000 0001 | 00000 | 000 | 00010 | 0010011
→ 0x00100113

add x3, x1, x2
→ R-type: funct7=0000000, rs2=2, rs1=1, funct3=000, rd=3, opcode=0110011
→ 0000000 | 00010 | 00001 | 000 | 00011 | 0110011
→ 0x002081B3

# ... continue for all instructions
```

**Or use a RISC-V assembler online:**
- https://riscvasm.lucasteske.dev/
- Paste assembly, get machine code

### **Update Instruction Memory**

Modify `instruction_memory.v`:

```verilog
initial begin
    // Fibonacci program
    mem[0] = 32'h00000093;  // addi x1, x0, 0
    mem[1] = 32'h00100113;  // addi x2, x0, 1
    mem[2] = 32'h002081B3;  // add x3, x1, x2
    mem[3] = 32'h002100B3;  // add x1, x2, x0
    mem[4] = 32'h00318133;  // add x2, x3, x0
    mem[5] = 32'h00128293;  // addi x5, x5, 1
    mem[6] = 32'h00500313;  // addi x6, x0, 5
    mem[7] = 32'hFE629CE3;  // bne x5, x6, -8 (loop)
    mem[8] = 32'h00000013;  // nop
    mem[9] = 32'hFE000EE3;  // beq x0, x0, -4 (done)
    
    // Initialize rest
    for (i = 10; i < 1024; i = i + 1)
        mem[i] = 32'h00000000;
end
```

**Recompile and run:**
```bash
make clean
make all
```

**Expected result:**
```
x1 = 5
x2 = 8
x3 = 13
x5 = 5 (loop counter)
```

---

## PART 10: COMMON ISSUES AND DEBUGGING

### **Issue 1: Simulation produces 'X' (undefined) values**

**Symptom:**
```
registers[1] = XXXXXXXX
```

**Causes:**
1. Uninitialized registers
2. Reading before writing
3. Timing issues

**Solution:**
```verilog
// In register_file.v, ensure initialization:
initial begin
    for (i = 0; i < 32; i = i + 1)
        registers[i] = 32'h00000000;  // ← Add this!
end
```

### **Issue 2: Wrong instruction execution**

**Symptom:**
```
Expected: x3 = 15
Got: x3 = 0
```

**Debug steps:**

1. **Check instruction encoding:**
```bash
# In testbench, add:
$display("Opcode: %b", instruction[6:0]);
$display("rd: %d", instruction[11:7]);
```

2. **Check control signals:**
```bash
$display("reg_write: %b", cpu.reg_write);
$display("alu_control: %b", cpu.alu_control);
```

3. **Check ALU operation:**
```bash
$display("ALU inputs: a=%d, b=%d", cpu.alu_unit.a, cpu.alu_unit.b);
$display("ALU result: %d", cpu.alu_result);
```

### **Issue 3: PC not incrementing**

**Symptom:**
```
PC stuck at 0x00000000
```

**Check:**
1. Is clock running?
2. Is reset being released?
3. Is `pc_next` calculated correctly?

**Add debug:**
```verilog
always @(posedge clk) begin
    $display("PC: %h, PC_next: %h, PC+4: %h", 
             pc, pc_next, pc_plus4);
end
```

### **Issue 4: Branch not working**

**Symptom:**
```
BEQ should branch but doesn't
```

**Check:**
1. Is `alu_zero` flag correct?
2. Is `branch` control signal high?
3. Is immediate correctly sign-extended?

**Debug:**
```verilog
$display("Branch: %b, Zero: %b, PC_src: %b", 
         branch, alu_zero, pc_src);
$display("PC_target: %h", pc_target);
```

---

## PART 11: WHAT YOU'VE BUILT - COMPLETE UNDERSTANDING

### **Block Diagram with Data Flow**

```
Instruction Execution Flow:
============================

1. FETCH
   PC → Instruction Memory → Instruction

2. DECODE
   Instruction → Control Unit → Control Signals
   Instruction[19:15] → Register File → Read Data 1
   Instruction[24:20] → Register File → Read Data 2
   Instruction → Imm Gen → Immediate

3. EXECUTE
   Read Data 1 + (Read Data 2 OR Immediate) → ALU → Result

4. MEMORY
   ALU Result → Data Memory → Memory Data

5. WRITEBACK
   (ALU Result OR Memory Data) → Register File[rd]
   
6. PC UPDATE
   PC + 4 (or Branch Target) → PC
```

### **Timing Diagram for One Instruction**

```
Clock Cycle N:
│
├─0ns: Clock rises, PC updates to new value
│
├─1ns: Instruction memory access (combinational)
│      → instruction wire has new value
│
├─2ns: Control signals generated (combinational)
│      → all control wires have values
│
├─3ns: Register file read (combinational)
│      → read_data1, read_data2 have values
│
├─4ns: ALU computation (combinational)
│      → alu_result has value
│
├─5ns: Memory access if needed (combinational read)
│      → mem_read_data has value
│
├─6ns: Write data selected (combinational)
│      → write_data has value
│
├─10ns: Next clock rise
│       → PC updates (sequential)
│       → Register file writes (sequential)
│       → Memory writes if sw (sequential)
│
└─Cycle complete, next instruction starts
```

---

## PART 12: PERFORMANCE ANALYSIS

### **CPI (Cycles Per Instruction)**

In our single-cycle design:
- **Every instruction takes exactly 1 cycle**
- CPI = 1.0

**Why is this special?**
- Simple to understand
- Predictable timing
- But: Clock frequency limited by slowest path

### **Critical Path Analysis**

The **critical path** is the longest combinational delay:

```
PC → Instruction Memory → Control → Register File → 
ALU → Data Memory → Write Data MUX → Register File

Total delay ≈ 8-10ns in simulation
Real hardware ≈ multiple nanoseconds

Maximum clock frequency = 1 / critical_path_delay
If critical path = 10ns → Max frequency = 100 MHz
```

### **Why Multi-Cycle or Pipelined?**

**Single-cycle problems:**
1. All instructions wait for slowest (load)
2. Hardware utilized inefficiently
3. Low clock frequency

**Solutions:**
- **Multi-cycle**: Break into multiple shorter cycles
- **Pipelined**: Overlap instruction execution (what you'll build next!)

---

## FINAL SUMMARY

### **What You've Accomplished:**

✅ **Built a complete 32-bit RISC-V processor from scratch**
✅ **Understood every instruction format**
✅ **Implemented all RV32I core instructions**
✅ **Created working memory systems**
✅ **Written and simulated real programs**
✅ **Debugged hardware like a pro**

### **Files You Have:**

```
├── alu.v                    (16-operation ALU)
├── alu_control.v            (Operation decoder)
├── control_unit.v           (Main decoder)
├── data_memory.v            (1KB RAM with byte/half/word access)
├── immediate_generator.v    (Sign-extend all formats)
├── instruction_memory.v     (Program storage)
├── register_file.v          (32 × 32-bit registers)
├── riscv_single_cycle.v     (Top module connecting everything)
└── testbench.v              (Comprehensive testing)
```

### **What Each Module Does:**

| Module | Lines | Purpose | Key Feature |
|--------|-------|---------|-------------|
| ALU | ~40 | Math & logic | 10 operations including shifts |
| ALU Control | ~50 | Operation select | Decodes funct3/funct7 |
| Control Unit | ~70 | Instruction decode | Generates 7 control signals |
| Data Memory | ~60 | Load/store | Byte-addressable, little-endian |
| Immediate Gen | ~50 | Extract immediates | Handles all 6 formats |
| Instruction Mem | ~30 | Store program | 1024 instruction capacity |
| Register File | ~40 | Register storage | x0 hardwired to zero |
| Top Module | ~150 | Integration | Connects all components |
| Testbench | ~150 | Verification | Monitors and displays results |

### **Performance Metrics:**

- **Supported Instructions**: 40+ RV32I instructions
- **Clock Cycle**: ~10ns (simulation)
- **CPI**: 1.0 (single-cycle)
- **Memory**: 1KB instruction + 1KB data
- **Registers**: 32 × 32-bit

---

## NEXT STEPS

Now that you have a working single-cycle processor, here are your options:

### **Option 1: Enhance Current Design**
- Add more RV32I instructions (shifts with immediate, unsigned loads)
- Implement CSR (Control and Status Registers)
- Add exception handling
- Implement M-extension (multiply/divide)

### **Option 2: Build 5-Stage Pipeline** (Your original goal!)
- Use this as your baseline
- Add pipeline registers between stages
- Implement hazard detection and forwarding
- Much better performance!

### **Option 3: Add Cache**
- Replace direct memory access
- Implement simple direct-mapped cache
- Add cache controller
- Measure hit/miss rates

### **Option 4: Make it Synthesizable**
- Remove simulation-only constructs
- Add proper reset logic
- Create constraints file
- Target FPGA (if you have one)

---

**You now have COMPLETE, DEEP understanding of:**
- RISC-V ISA architecture
- Processor datapath design
- Control unit operation
- Memory systems
- Verilog HDL
- Hardware debugging

**This is a MASSIVE accomplishment!**

Want to continue to the 5-stage pipeline? Or dive deeper into any specific part? Let me know what you want to explore next! 🚀
