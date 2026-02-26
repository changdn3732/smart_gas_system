<<<<<<< HEAD
Scheduler Project

This is a minimal Flet-based scheduler UI for Temperature and Gas scheduling.

Features:
- Left panel (60%) with toggle between Temperature and Gas scheduling.
- Temperature scheduling: 8 steps, each with Temp (°C) and Duration (hours). Automatically computes Rate (°/h) per step.
- Gas scheduling: simple 4-channel setpoint + duration inputs (placeholder).
- Right panel (40%) reserved for real-time trends (placeholder).

Run:

1. Create a virtual environment (recommended):

```bash
python -m venv .venv
.venv\Scripts\activate
```

2. Install requirements:

```bash
pip install -r requirements.txt
```

3. Run the app:

```bash
python main.py
```
ex. ubuntu quick

'''bash
git clone https://github.com/changdn3732/smart_gas_system.git
cd smart_gas_system
chmod +x run.sh
./run.sh
'''

Notes:
- Requires `flet` package. The UI is intentionally minimal and focuses on the scheduling elements requested.
- You can expand the gas scheduling and connect the scheduler to real devices or simulators.
=======
# smart_gas_system
>>>>>>> a37a3e7e2e547cc35f9731ed26056c634bc6ee7c
