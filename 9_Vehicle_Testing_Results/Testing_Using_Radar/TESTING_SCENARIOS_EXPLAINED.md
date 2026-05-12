# MPC vs PD Controller Testing: Scenario Definitions

## Overview

Two distinct test scenarios were executed to comprehensively evaluate MPC and PD controller performance in adaptive cruise control applications:

---

## 1. CS - Completely Stop Scenario

**Definition:** Ego vehicle approaches a stationary obstacle and must decelerate smoothly to come to a complete stop.

**Test Setup:**
- Obstacle/target: **Stationary** at controlled distances
- Initial velocity: 5 kph or 10 kph
- Final velocity: 0 kph (complete stop)
- Challenge: Smooth deceleration, precise stopping, collision avoidance

**Key Metrics:**
- **Minimum Separation** — Safety margin at stop point
- **RMS Acceleration** — Smoothness of braking deceleration
- **Jerk** — Smoothness of brake application transitions
- **Overshoot/Undershoot** — Precision in reaching zero velocity (stopping)

**Test Dates & Results:**
- **Date:** March 27, 2026
- **Test Cases:** 3-4 per speed, 6 total
- **Speed Profiles:** 5 kph, 10 kph
- **Result:** MPC outperforms PD in all metrics (38-49% better safety, 35-45% lower jerk)

**Performance Focus:**
- Emergency braking capability
- Collision avoidance at stop
- Passenger comfort during deceleration
- Smooth transition to complete stop

**Location:** `9_Vehicle_Testing_Results/Testing_Using_Radar/March27_2026/`

---

## 2. FV - Following Vehicle Scenario

**Definition:** Ego vehicle follows a moving lead vehicle and must maintain optimal distance while adapting to lead vehicle velocity changes.

**Test Setup:**
- Lead vehicle: **Moving** with natural traffic patterns and obstacles
- Initial velocity: 5 kph or 10 kph
- Lead vehicle behavior: Variable acceleration/deceleration
- Challenge: Smooth tracking, maintaining safe distance, predictive control

**Key Metrics:**
- **Minimum Separation** — Safety margin during following
- **RMS Acceleration** — Smoothness of acceleration/deceleration during tracking
- **Jerk** — Smoothness of control transitions while following
- **Overshoot/Undershoot** — Precision in tracking lead vehicle velocity

**Test Dates & Results:**
- **Dates:** March 30, April 7, April 9, 2026
- **Test Cases:** 4-5 per date, ~15 total
- **Speed Profiles:** 5 kph, 10 kph
- **Result:** MPC outperforms PD in all metrics (28-33% better safety, 24-42% lower jerk)

**Performance Focus:**
- Real-time lead vehicle tracking
- Traffic flow integration
- Adaptive distance management
- Smooth velocity matching

**Location:** `9_Vehicle_Testing_Results/Testing_Using_Radar/Plot_Comparisions/`

---

## Key Differences: CS vs FV

| Aspect | CS (Completely Stop) | FV (Following Vehicle) |
|--------|---------------------|----------------------|
| **Target** | Stationary obstacle | Moving lead vehicle |
| **Final State** | Complete stop (v=0) | Steady following (maintain distance) |
| **Duration** | Short (approach & stop) | Sustained (continuous tracking) |
| **Prediction Need** | High (planned deceleration) | Very High (continuous anticipation) |
| **Typical Environment** | Traffic jams, parking | Highway, city traffic |
| **Primary Control** | Braking smoothness | Acceleration/deceleration smoothness |
| **Test Complexity** | Controlled, repeatable | Variable, naturalistic |

---

## Performance Comparison Summary

### CS Scenario (Braking Focus)
- MPC Safety: **49% better** (at 5 kph)
- MPC Smoothness: **45% better jerk** (at 10 kph)
- Primary advantage: **Planned deceleration profiles**

### FV Scenario (Following Focus)
- MPC Safety: **33% better** (at 10 kph)
- MPC Smoothness: **42% better jerk** (at 10 kph)
- Primary advantage: **Predictive lead vehicle tracking**

---

## Reports Location

1. **CS Analysis Report:** `March27_2026/MPC_vs_PD_Analysis_Report.md`
   - Braking performance analysis
   - Complete stop scenario metrics
   - Detailed stopping safety analysis

2. **FV Analysis Report:** `Plot_Comparisions/FV_Testing_Analysis_Report.md`
   - Following vehicle performance analysis
   - Lead vehicle tracking metrics
   - Real-world scenario validation

---

## Notebook Analysis Tools

1. **CS Analysis Notebook:** `March27_2026/PDC_VS_MPC_Comparison_CS.ipynb`
   - Contains metrics calculation and visualization cells
   - Generates comparison plots for CS scenarios
   - Includes overshoot/undershoot analysis

2. **FV Analysis Notebook:** `Plot_Comparisions/PDC_VS_MPC_Comparison_FV.ipynb`
   - Contains metrics calculation and visualization cells
   - Generates comparison plots for FV scenarios
   - Includes multi-date comparison analysis
