## 📌 Descriere

Acest modul implementează **localizarea robotului folosind Extended Kalman Filter (EKF)**.

Scopul este să combine:

* **odometria (encodere)** → pentru estimare continuă
* **măsurători din cameră (ArUco)** → pentru corecție

Rezultatul este o estimare stabilă a poziției robotului:

```text
(x, y, theta)
```

---

## 🧠 Cum funcționează

Sistemul are două etape principale:

### 🔁 1. Predict (din encodere)

Folosește viteza și rotația robotului pentru a estima poziția:

```text
x = x + v * cos(theta) * dt
y = y + v * sin(theta) * dt
theta = theta + omega * dt
```

---

### 👁 2. Update (din cameră)

Dacă există o măsurătoare validă din cameră, aceasta este folosită pentru a corecta poziția estimată.

---

## 📂 Structura proiectului

```text
math_utils.py            # funcții ajutătoare (ex: normalize_angle)
ekf.py                   # implementarea EKF
localization_system.py   # logică de integrare (predict + update)
main.py                  # testare
```

---

## 📥 Input necesar

### 🔹 Odometrie (encodere)

```python
odom = {
    "v": float,        # viteza liniară (m/s)
    "omega": float,    # viteza unghiulară (rad/s)
    "dt": float        # timp între pași (sec)
}
```

---

### 🔹 Measurement (camera)

```python
measurement = {
    "x": float,        # poziție globală X (metri)
    "y": float,        # poziție globală Y (metri)
    "theta": float,    # orientare (radiani)
    "valid": bool      # dacă măsurătoarea este validă
}
```

---

## ⚠️ IMPORTANT (camera)

Măsurătoarea de la cameră trebuie să fie:

* în **coordonate globale ale terenului**
* NU relativă la cameră

---

## 📤 Output

```python
{
    "x": ...,
    "y": ...,
    "theta": ...,
    "measurement_used": True/False
}
```

---

## 🛡️ Robustete

Sistemul:

* ignoră măsurători absurde
* normalizează unghiurile
* permite tuning prin Q și R

---

