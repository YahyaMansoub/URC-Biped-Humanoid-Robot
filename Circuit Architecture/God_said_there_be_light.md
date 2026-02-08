# Arduino Nano + PCA9685 + LM2596 Power Architecture

This README documents a **simple, working power and control architecture** for a robot (e.g. biped) using:

* Arduino Nano (controller)
* PCA9685 (PWM / servo driver)
* LM2596 (DC‑DC buck converter)
* Battery (single source)

The goal is **one power source**, **clean wiring**, and **reliable servo operation**.

---

## Overview

* **LM2596** provides a regulated **5.0 V** rail
* That 5 V rail powers:

  * Arduino Nano
  * PCA9685 logic (VCC)
  * PCA9685 servo rail (V+)
* Arduino Nano communicates with PCA9685 via **I²C** only
* All grounds are **common**

---

## Power Flow (High Level)

```
Battery → LM2596 (5.0 V) → Nano + PCA9685 → Servos
```

---

## Step 1 — Set the LM2596

Before connecting anything else:

* Power LM2596 from the battery
* Adjust output with a multimeter
* **Set OUT = 5.0 V**

---

## Step 2 — Battery to LM2596

```
Battery +  → LM2596 IN+
Battery −  → LM2596 IN−
```

---

## Step 3 — LM2596 to PCA9685 (Power)

```
LM2596 OUT+ → PCA V+
LM2596 OUT− → PCA GND
```

This powers the **servo rail**.

---

## Step 4 — Power PCA9685 Logic (VCC)

The PCA9685 has two power pins:

* **V+**  → servos
* **VCC** → PCA logic

Using a single supply, both must be powered:

```
LM2596 OUT+ → PCA VCC
```

(Some boards have a jumper; otherwise use a wire.)

---

## Step 5 — LM2596 to Arduino Nano (Power)

Power the Nano **from the 5V pin (not VIN)**:

```
LM2596 OUT+ → Nano 5V
LM2596 OUT− → Nano GND
```

⚠️ Do not power Nano by USB at the same time.

---

## Step 6 — Arduino Nano to PCA9685 (Signals)

Only I²C signals:

```
Nano A4 (SDA) → PCA SDA
Nano A5 (SCL) → PCA SCL
Nano GND      → PCA GND
```

---

## Step 7 — Servo Connection (Channel 0 example)

On PCA9685 channel 0 header:

```
Servo brown/black → GND
Servo red         → V+
Servo yellow/white→ SIG
```

---

## Full Wiring Schema (ASCII)

```
Battery
  │
  ▼
LM2596  (OUT = 5.0 V)
  │
  ├──→ Nano 5V
  ├──→ PCA VCC
  ├──→ PCA V+
  └──→ GND (common)

Nano A4 ─────────→ PCA SDA
Nano A5 ─────────→ PCA SCL
```

---

## Critical Rules (Do Not Skip)

* PCA9685 **VCC ≠ V+** (both must be powered)
* Servo red wire goes to **V+**, not VCC
* All grounds must be **common**
* Nano powered from **5V pin**, not VIN

---

## Notes

* This single‑buck setup is fine for **testing and small robots**
* For many servos / bipeds, prefer:

  * 6 V buck for servos
  * 5 V buck for logic

---

## Status

✔ Minimal
✔ Reproducible
✔ GitHub‑ready



0 160
1 460
2 250


6 150
5 80   
4 180





#define hipLOffset 250
#define kneeLOffset 460
#define ankleLOffset 160
#define hipROffset 150
#define kneeROffset 80
#define ankleROffset 250

#define l1 6.5
#define l2 7

#define stepClearance 1
#define stepHeight 15


#define HIP_L_CH    2
#define HIP_R_CH    6
#define KNEE_L_CH   1
#define KNEE_R_CH   5
#define ANKLE_L_CH  2
#define ANKLE_R_CH  4