

**CAD MODEL:**

**Iteration 1:**
<img width="382" height="245" alt="image" src="https://github.com/user-attachments/assets/186ee861-bac3-4b73-9ad0-947a31e52b6c" />


**Iteration 2:**
<img width="1916" height="1001" alt="image" src="https://github.com/user-attachments/assets/8dbc3705-d551-4591-ad94-afcdd0571b7a" />


**Iteration 3:**
<img width="657" height="542" alt="image" src="https://github.com/user-attachments/assets/a18d21dc-2f6f-4c39-b54d-dda7765db8be" />

# Metal Planetary Gear System-Based Actuators

A compact, high-efficiency torque multiplication mechanism designed for integration with Brushless DC (BLDC) motors to enable high-torque, space-constrained automation and robotics applications. Developed at the **Stochastic Robots Lab, Robert Bosch Centre for Cyber Physical Systems, IISc Bangalore**.

## 📌 Overview
BLDC motors provide high speed and precise control but often lack sufficient torque under heavy loads. This project focuses on designing, assembling, and evaluating a **Compound Planetary Gearbox Actuator** that delivers a high reduction ratio, robust torque amplification, and minimal power transmission losses.

---

## ⚙️ Specifications & Technical Features

- **Target Application:** Robotics, automation, and high-load mobility systems
- **Motor:** Vector Technics 8020 BLDC Motor (Peak torque ~4 Nm)
- **Microcontroller:** O-Drive Pro board with custom 3D-printed encoder mounting
- **Compound Gear Ratio:** 11.2:1
- **Materials:**
  - **Gears:** Stainless Steel (manufactured using Wire EDM)
  - **Housing & Carriers:** Aluminium 6061
- **Bearings:** SKF precision ball bearings

---

## 📐 Mathematical Model & Working Principles

The planetary system consists of a central **Sun gear**, orbiting **Planet gears**, and an internal **Ring gear**.

### Key Formulas
1. **Module Matching Condition:**  
   $$\frac{d_1}{z_1} = \frac{d_2}{z_2}$$

2. **Geometric Relationship:**  
   $$r_R = r_S + 2r_P \implies r_P = \frac{r_R - r_S}{2}$$

3. **Gear Ratio (Single Stage):**  
   $$i = 1 + \frac{r_R}{r_S}$$

---

## 🔄 Design Evolution

1. **Iteration 1 (Single-Stage Prototype):**
   - Sun gear: 10T ($r = 10\text{ mm}$)
   - Planet gears: 40T ($r = 40\text{ mm}$)
   - Ring gear: 90T ($r = 90\text{ mm}$)
   - Gear ratio: 10:1
   - Validated via SolidWorks motion study simulations.

2. **Iteration 2 & 3 (Single stage  and Compound Planetary Gear Actuator):**
   - Upgraded the design in iteration 1 to iteration to which was an internal single stage planetary gear box with a reduction ratio of 6:1.
   - Finally transitioned to the latest version i.e. a compound planetary gearboxes with optimized sizing and a gear ratio of 11.2:1 
---

## 🧪 Testing & Performance

- **Test Bench:** Inspired by the UC Berkeley Humanoid Actuator Test Bench setup.
- **Starting Torque:** ~0.7 Nm recorded.
- **Efficiency:** Measured across varying commanded speeds (3.1 to 11.5 rad/s) and loads up to 21 Nm, achieving peak mechanical efficiency of ~50%.

---

## 🛠️ Assembly & Tools
- **CAD Software:** SolidWorks
- **Manufacturing:** Wire EDM, turning operations, press-fit assembly via arbor press
- **Control:** O-Drive Python API / closed-loop velocity and torque control

**WORKING VIDEO OF ACTUATOR:**
https://github.com/user-attachments/assets/768c27c0-2452-47af-b455-d3c3d1c2b2ac

