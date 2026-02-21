# EtherCAT Object Reference Manual
## HDrive17-ETC Stepper Motor Controller

### Version: 1.0.0
### Date: January 2026
### Device: HDrive17-ETC Stepper Motor Controller
### Profile: CiA 402 (Drive Profile)

---

## Table of Contents

1. [Overview](#overview)
2. [CiA 402 Standard Objects](#cia-402-standard-objects)
3. [Custom Application Objects](#custom-application-objects)
4. [PDO Mapping](#pdo-mapping)
5. [Process Data](#process-data)
6. [Object Dictionary Structure](#object-dictionary-structure)
7. [Usage Examples](#usage-examples)

---

## Overview

The HDrive17-ETC is an EtherCAT stepper motor controller implementing the CiA 402 Drive Profile. This manual provides a comprehensive reference for all EtherCAT objects available in the device.

### Key Features
- **CiA 402 Drive Profile** compliance
- **Position, Velocity, and Torque Control** modes
- **Real-time Process Data** exchange
- **Advanced Motor Control** algorithms
- **Digital I/O** support
- **Calibration and Configuration** objects

---

## CiA 402 Standard Objects

### Control and Status Objects

#### 0x6040 - Control Word
- **Type**: UINT16
- **Access**: Read/Write
- **Description**: Control word for drive state machine
- **Bit Fields**:
  - Bit 0: Switch On
  - Bit 1: Enable Voltage
  - Bit 2: Quick Stop
  - Bit 3: Enable Operation
  - Bit 4-6: Operation Mode Specific
  - Bit 7: Fault Reset
  - Bit 8: Halt

#### 0x6041 - Status Word
- **Type**: UINT16
- **Access**: Read Only
- **Description**: Status word from drive state machine
- **Bit Fields**:
  - Bit 0: Ready to Switch On
  - Bit 1: Switched On
  - Bit 2: Operation Enabled
  - Bit 3: Fault
  - Bit 4: Voltage Enabled
  - Bit 5: Quick Stop
  - Bit 6: Switch On Disabled
  - Bit 7: Warning
  - Bit 8: Manufacturer Specific
  - Bit 9: Remote
  - Bit 10: Target Reached
  - Bit 11: Internal Limit Active

### Position Control Objects

#### 0x6060 - Modes of Operation
- **Type**: INT8
- **Access**: Read/Write
- **Description**: Operation mode selection
- **Values**:
  - 1: Profile Position Mode
  - 3: Profile Velocity Mode
  - 4: Profile Torque Mode
  - 8: Cyclic Synchronous Position Mode
  - 9: Cyclic Synchronous Velocity Mode
  - 10: Cyclic Synchronous Torque Mode

#### 0x6061 - Modes of Operation Display
- **Type**: INT8
- **Access**: Read Only
- **Description**: Currently active operation mode

#### 0x6062 - Position Demand Value
- **Type**: INT32
- **Access**: Read Only
- **Description**: Position demand value in tenths of degrees
- **Unit**: 0.1°

#### 0x6063 - Position Actual Internal Value
- **Type**: INT32
- **Access**: Read Only
- **Description**: Internal position actual value
- **Unit**: 0.1°

#### 0x6064 - Position Actual Value
- **Type**: INT32
- **Access**: Read Only
- **Description**: Position actual value with offset compensation
- **Unit**: 0.1°

#### 0x606B - Velocity Demand Value
- **Type**: INT32
- **Access**: Read Only
- **Description**: Velocity demand value
- **Unit**: 0.01 rpm

#### 0x606C - Velocity Actual Value
- **Type**: INT32
- **Access**: Read Only
- **Description**: Velocity actual value
- **Unit**: 0.01 rpm

#### 0x607A - Target Position
- **Type**: INT32
- **Access**: Read/Write
- **Description**: Target position for position control
- **Unit**: 0.1°

#### 0x60FF - Target Velocity
- **Type**: INT32
- **Access**: Read/Write
- **Description**: Target velocity for velocity control
- **Unit**: 0.01 rpm

#### 0x6071 - Target Torque
- **Type**: INT16
- **Access**: Read/Write
- **Description**: Target torque for torque control
- **Unit**: 0.1% of rated torque

#### 0x6077 - Torque Actual Value
- **Type**: INT16
- **Access**: Read Only
- **Description**: Actual torque value
- **Unit**: 0.1% of rated torque

### Profile Parameters

#### 0x607F - Max Profile Velocity
- **Type**: UINT32
- **Access**: Read/Write
- **Description**: Maximum profile velocity
- **Unit**: 0.01 rpm

#### 0x6080 - Max Motor Speed
- **Type**: UINT32
- **Access**: Read/Write
- **Description**: Maximum motor speed
- **Unit**: 0.01 rpm

#### 0x6083 - Profile Acceleration
- **Type**: UINT32
- **Access**: Read/Write
- **Description**: Profile acceleration
- **Unit**: 0.01 rpm/s

#### 0x6084 - Profile Deceleration
- **Type**: UINT32
- **Access**: Read/Write
- **Description**: Profile deceleration
- **Unit**: 0.01 rpm/s

#### 0x6072 - Max Torque
- **Type**: UINT16
- **Access**: Read/Write
- **Description**: Maximum torque
- **Unit**: 0.1% of rated torque

#### 0x6079 - DC Link Circuit Voltage
- **Type**: UINT16
- **Access**: Read Only
- **Description**: DC link circuit voltage
- **Unit**: 0.1V

### Digital I/O Objects

#### 0x60FD - Digital Inputs
- **Type**: UINT8
- **Access**: Read Only
- **Description**: Digital input states
- **Bit Fields**:
  - Bit 0: Digital Input 1
  - Bit 1: Digital Input 2

#### 0x60FE - Digital Inputs Polarity
- **Type**: UINT8
- **Access**: Read/Write
- **Description**: Digital input polarity configuration
- **Values**:
  - 0: Normal polarity
  - 1: Inverted polarity

---

## Custom Application Objects

### Motor Parameters

#### 0x6620 - Motor Dynamic Parameters
- **Type**: Structure
- **Access**: Read/Write
- **Description**: Motor dynamic parameters
- **Sub-Indexes**:
  - 0x6620.1: Current Control Damping Factor
  - 0x6620.2: Speed Control Damping Factor
  - 0x6620.3: Position Control Damping Factor

#### 0x6627 - Max Speed
- **Type**: UINT32
- **Access**: Read/Write
- **Description**: Maximum motor speed
- **Unit**: rpm

#### 0x6628 - Slew Rate
- **Type**: UINT32
- **Access**: Read/Write
- **Description**: Position slew rate
- **Unit**: rpm

#### 0x6629 - Max Current
- **Type**: UINT16
- **Access**: Read/Write
- **Description**: Maximum current
- **Unit**: mA

### Control Parameters

#### 0x6640 - Control Bandwidth
- **Type**: Structure
- **Access**: Read/Write
- **Description**: Control loop bandwidth parameters
- **Sub-Indexes**:
  - 0x6640.1: Current Control Bandwidth
  - 0x6640.2: Speed Control Bandwidth
  - 0x6640.3: Position Control Bandwidth

#### 0x6645 - Filter Parameters
- **Type**: Structure
- **Access**: Read/Write
- **Description**: Filter frequency parameters
- **Sub-Indexes**:
  - 0x6645.1: Position Filter Frequency
  - 0x6645.2: Position Pre-filter Frequency
  - 0x6645.3: Feed Forward Filter Frequency
  - 0x6645.4: AB Filter Frequency
  - 0x6645.5: QD Filter Frequency

#### 0x6660 - Cascade Control Parameters
- **Type**: Structure
- **Access**: Read/Write
- **Description**: Cascade control parameters
- **Sub-Indexes**:
  - 0x6660.1: Current P-Gain
  - 0x6660.2: Current I-Gain
  - 0x6660.3: Current Kb-Gain
  - 0x6660.4: Speed P-Gain
  - 0x6660.5: Speed I-Gain
  - 0x6660.6: Position P-Gain

### Motor Physical Parameters

#### 0x6630 - Torque Constant
- **Type**: UINT16
- **Access**: Read/Write
- **Description**: Motor torque constant
- **Unit**: mNm/A

#### 0x6631 - DC Coil Resistance
- **Type**: UINT16
- **Access**: Read/Write
- **Description**: Motor coil resistance
- **Unit**: mOhm

#### 0x6632 - Phase Inductance
- **Type**: UINT16
- **Access**: Read/Write
- **Description**: Motor phase inductance
- **Unit**: µH

#### 0x6633 - Rotor Inertia
- **Type**: UINT32
- **Access**: Read/Write
- **Description**: Rotor inertia
- **Unit**: g·cm²

#### 0x6634 - Damping
- **Type**: UINT16
- **Access**: Read/Write
- **Description**: Motor damping coefficient
- **Unit**: µN·m·s/rad

#### 0x6635 - Resonance Frequency
- **Type**: UINT16
- **Access**: Read/Write
- **Description**: Motor resonance frequency
- **Unit**: Hz

### Digital I/O Configuration

#### 0x6667 - Latch Position 1
- **Type**: INT32
- **Access**: Read/Write
- **Description**: Position latch 1 value
- **Unit**: 0.1°

#### 0x6668 - Latch Trigger Source 1
- **Type**: UINT8
- **Access**: Read/Write
- **Description**: Latch trigger source for latch 1

#### 0x6669 - Latch Reset 1
- **Type**: UINT8
- **Access**: Read/Write
- **Description**: Latch reset command for latch 1

#### 0x6670 - Latch Position 2
- **Type**: INT32
- **Access**: Read/Write
- **Description**: Position latch 2 value
- **Unit**: 0.1°

#### 0x6671 - Latch Trigger Source 2
- **Type**: UINT8
- **Access**: Read/Write
- **Description**: Latch trigger source for latch 2

#### 0x6672 - Latch Reset 2
- **Type**: UINT8
- **Access**: Read/Write
- **Description**: Latch reset command for latch 2

#### 0x6691 - Digital Input 1
- **Type**: UINT8
- **Access**: Read Only
- **Description**: Digital input 1 state

#### 0x6692 - Digital Input 2
- **Type**: UINT8
- **Access**: Read Only
- **Description**: Digital input 2 state

### Parameter Management

#### 0x6637 - Trigger New Parameter Calculation
- **Type**: UINT8
- **Access**: Read/Write
- **Description**: Triggers parameter recalculation
- **Values**:
  - 0: No action
  - 1: Recalculate all control parameters
  - 2: Save values to EEPROM
  - 3: Reset to default values
  - 4: Save ripple calibration data

---

## PDO Mapping

### RxPDO (Master to Slave)

#### 0x1600 - RxPdoMappingCspCsV
**Cyclic Synchronous Position/Velocity Mode**
- SubIndex 1: Target Position (0x607A)
- SubIndex 2: Target Velocity (0x60FF)
- SubIndex 3: Target Torque (0x6071)
- SubIndex 4: Mode of Operation (0x6060)
- SubIndex 5: Control Word (0x6040)

#### 0x1601 - RxPdoMappingCsp
**Cyclic Synchronous Position Mode**
- SubIndex 1: Target Position (0x607A)
- SubIndex 2: Control Word (0x6040)

#### 0x1602 - RxPdoMappingCsv
**Cyclic Synchronous Velocity Mode**
- SubIndex 1: Target Velocity (0x60FF)
- SubIndex 2: Control Word (0x6040)

### TxPDO (Slave to Master)

#### 0x1A00 - TxPdoMappingCspCsV
**Cyclic Synchronous Position/Velocity Mode**
- SubIndex 1: Position Actual Value (0x6064)
- SubIndex 2: Velocity Actual Value (0x606C)
- SubIndex 3: Torque Actual Value (0x6077)
- SubIndex 4: Status Word (0x6041)
- SubIndex 5: Mode of Operation Display (0x6061)

#### 0x1A01 - TxPdoMappingCsp
**Cyclic Synchronous Position Mode**
- SubIndex 1: Position Actual Value (0x6064)
- SubIndex 2: Status Word (0x6041)

#### 0x1A02 - TxPdoMappingCsv
**Cyclic Synchronous Velocity Mode**
- SubIndex 1: Velocity Actual Value (0x606C)
- SubIndex 2: Status Word (0x6041)

---

## Process Data

### Input Process Data (TxPDO)
The device provides real-time process data to the master:

```cpp
REAL32 etherCAT_TxData[16];  // Transmit data array
```

**Data Mapping**:
- Index 0: Position Actual Value (SPI)
- Index 1: Velocity Actual Value
- Index 2: Speed (Kalman Filter)
- Index 3: FID (Force in D-axis) × 1e-3
- Index 4: FIQ (Force in Q-axis) × 1e-3
- Index 5: FUA (Force in A-phase) × 1e-3
- Index 6: FUB (Force in B-phase) × 1e-3
- Index 7: FUD (Force in D-axis) × 1e-3
- Index 8: Current Error
- Index 9: Indexer
- Index 10: Current A
- Index 11: Current B
- Index 12: Calibration Array Value
- Index 13: Ripple Compensation Value
- Index 14: Voltage P
- Index 15: Position Error

### Output Process Data (RxPDO)
The device receives real-time process data from the master:

```cpp
INT32 etherCAT_RxData[8];    // Receive data array
```

**Data Mapping**:
- Index 0: Debug Parameter 0
- Index 1: Debug Parameter 1
- Index 2: Debug Parameter 2
- Index 3: Debug Parameter 3
- Index 4: Debug Parameter 4
- Index 5: Main Key Selector
- Index 6: Sub Key Selector
- Index 7: Value

---

## Object Dictionary Structure

The object dictionary is organized into main categories:

### Main Keys (enMainKeys)
```cpp
enum class enMainKeys {
    actualMotordata = 0,      // Actual motor data
    demandedValues = 1,       // Demanded values
    controlValues = 2,        // Control parameters
    motorValues = 3,          // Motor physical parameters
    motorSystemValues = 4,    // Motor system parameters
    storedValues = 5,         // Stored values
    // ... additional keys
};
```

### Sub-Keys for Each Category
Each main key contains multiple sub-keys for specific parameters:

**actualMotordata**:
- actualPositionSPI
- actualSpeedRPM
- actualSpeed_Kalmann
- actualTorque_mNm
- actualVoltageP
- motorMode
- actualPositionError
- actualCurrentError

**demandedValues**:
- demandedPosition
- demandedSpeed
- demandedAcc
- demandedDecc
- targetTorque

**controlValues**:
- currentCtrlBW
- speedCtrlBW
- positionCtrlBW
- filterFreqPos
- posPreFilter
- posFFFilter
- filterFreqAB
- filterFreqQD

---

## Usage Examples

### Basic Position Control Setup

```cpp
// Set operation mode to Profile Position
ModesOfOperation0x6060 = 1;

// Set target position (in tenths of degrees)
TargetPosition0x607A = 3600;  // 360.0 degrees

// Set profile parameters
MaxProfileVelocity0x607F = 1000;      // 10.00 rpm
ProfileAcceleration0x6083 = 500;      // 5.00 rpm/s
ProfileDeceleration0x6084 = 500;       // 5.00 rpm/s

// Enable operation
Controlword0x6040 = 0x001F;  // Switch on, enable voltage, enable operation
```

### Velocity Control Setup

```cpp
// Set operation mode to Profile Velocity
ModesOfOperation0x6060 = 3;

// Set target velocity (in hundredths of rpm)
TargetVelocity0x60FF = 500;  // 5.00 rpm

// Enable operation
Controlword0x6040 = 0x001F;
```

### Torque Control Setup

```cpp
// Set operation mode to Profile Torque
ModesOfOperation0x6060 = 4;

// Set target torque (in tenths of percent)
TargetTorque0x6071 = 500;  // 50.0% of rated torque

// Enable operation
Controlword0x6040 = 0x001F;
```

### Parameter Configuration

```cpp
// Set motor parameters
TorqueConstant0x6630 = 100;        // 100 mNm/A
DCCoilResistant0x6631 = 5000;      // 5.0 Ohm
PhaseInductance0x6632 = 10000;     // 10.0 mH
RotorInertia0x6633 = 1000000;      // 1000 g·cm²

// Set control parameters
ControlBandwith0x6640.Current = 1000;   // 1000 rad/s
ControlBandwith0x6640.Speed = 100;      // 100 rad/s
ControlBandwith0x6640.Position = 10;    // 10 rad/s

// Trigger parameter calculation
TriggerNewParameterCalculation0x6637 = 1;
```

### Digital I/O Configuration

```cpp
// Configure digital input polarity
DigitalInputsPolarity0x60FE = 0;  // Normal polarity

// Set up position latches
LatchPosition10x6667 = 1800;      // 180.0 degrees
LatchTriggerSource10x6668 = 1;    // Trigger source
LatchReset10x6669 = 0;            // No reset

LatchPosition20x6670 = 3600;      // 360.0 degrees
LatchTriggerSource20x6671 = 2;    // Trigger source
LatchReset20x6672 = 0;            // No reset
```

---

## Error Handling

### Error Codes (0x603F)
The device provides error codes through the standard EtherCAT error register:

- **0x0000**: No error
- **0x1000**: Generic error
- **0x2310**: Over current error
- **0x3210**: Under voltage error
- **0x3220**: Over voltage error
- **0x8611**: Position error

### Error Recovery
```cpp
// Check for errors
if (ErrorCode0x603F != 0) {
    // Perform error recovery
    Controlword0x6040 = 0x0080;  // Fault reset
}
```

---

## Performance Characteristics

### Update Rates
- **Process Data**: 1 kHz (1 ms cycle time)
- **Control Loop**: 20 kHz (50 µs cycle time)
- **Position Update**: 1 MHz (1 µs resolution)

### Resolution
- **Position**: 0.1° (tenths of degrees)
- **Velocity**: 0.01 rpm (hundredths of rpm)
- **Torque**: 0.1% of rated torque
- **Current**: 1 mA

### Latency
- **Process Data**: < 1 ms
- **Control Response**: < 50 µs
- **Parameter Update**: < 100 ms

---

## Compliance and Standards

- **EtherCAT**: ETG.1000
- **CiA 402**: Drive Profile Version 2.0
- **Safety**: SIL 2 (when configured)
- **EMC**: EN 61000-6-2, EN 61000-6-4

---

## Support and Documentation

For additional support and documentation:
- **EtherCAT Technology Group**: www.ethercat.org
- **CiA (CAN in Automation)**: www.can-cia.org
- **Device Manual**: Manual_HDrive17-etc.pdf

---

*This manual is based on firmware version 1.0.0 and object dictionary version 1.0.0.11*
