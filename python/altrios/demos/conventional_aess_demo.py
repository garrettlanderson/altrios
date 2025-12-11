# %%
# Demo script for Conventional AESS (Automatic Engine Start-Stop) Locomotive
# Demonstrates thermal model, air brake system, and AESS controller functionality

import time
import matplotlib.pyplot as plt
import numpy as np
import seaborn as sns

import altrios as alt

sns.set_theme()

SHOW_PLOTS = alt.utils.show_plots()
SAVE_INTERVAL = 1

# %%
# Build the AESS locomotive components

# Fuel Converter with thermal model
fc = alt.FuelConverter.default()
# Configure thermal model parameters
# Note: These would need to be set via Python API when exposed
# fc.ambient_temp, fc.initial_engine_temp, fc.engine_mass, fc.coolant_setpoint_temp

# Generator
gen = alt.Generator.default()

# Electric Drivetrain  
edrv = alt.ElectricDrivetrain.default()

# Starter Battery (72V, 650Ah)
# 72V * 650Ah = 46,800 Wh = 168,480,000 J
starter_battery = alt.ReversibleEnergyStorage(
    temperature_interp_grid=[0.0, 25.0, 45.0],
    soc_interp_grid=[0.0, 0.5, 1.0],
    c_rate_interp_grid=[0.0, 1.0, 2.0],
    eta_interp_values=[
        [[0.95, 0.95, 0.95], [0.96, 0.96, 0.96], [0.95, 0.95, 0.95]],
        [[0.95, 0.95, 0.95], [0.96, 0.96, 0.96], [0.95, 0.95, 0.95]],
        [[0.95, 0.95, 0.95], [0.96, 0.96, 0.96], [0.95, 0.95, 0.95]],
    ],
    pwr_out_max_watts=10000.0,  # 10 kW max power
    energy_capacity_joules=168480000.0,  # 72V * 650Ah
    min_soc=0.2,
    max_soc=1.0,
    initial_soc=0.9,  # Start at 90%
    initial_temperature_celcius=25.0,
    save_interval=SAVE_INTERVAL,
)

# Air Brake System
# Note: This component needs to be exposed to Python API
# Placeholder for now - would instantiate AirBrake with parameters

# AESS Controller
# Note: This component needs to be exposed to Python API  
# Placeholder for now - would instantiate AESSController with:
# - soc_low_threshold=0.5
# - soc_high_threshold=0.95
# - temp_low_threshold_celsius=48.89  # 120°F
# - temp_high_threshold_celsius=82.22  # 180°F
# - shutdown_delay_seconds=600.0  # 10 minutes
# - battery_constant_load_watts=500.0
# - battery_charge_power_watts=10000.0

# %%
# For now, create a conventional locomotive to demonstrate the concept
# The full AESS locomotive would be built with:
# conv_aess = alt.ConventionalAESSLoco(
#     fuel_converter=fc,
#     generator=gen,
#     electric_drivetrain=edrv,
#     starter_battery=starter_battery,
#     air_brake=air_brake,
#     aess_controller=aess_controller,
# )

conv = alt.Locomotive.build_conventional_loco(
    fuel_converter=fc,
    generator=gen,
    drivetrain=edrv,
    loco_params=alt.LocoParams(
        pwr_aux_offset_watts=13e3,
        pwr_aux_traction_coeff_ratio=1.1e-3,
        force_max_newtons=667.2e3,
    ),
    save_interval=SAVE_INTERVAL,
)

# %%
# Create power trace for simulation
pt = alt.PowerTrace.default()

# Run simulation
sim = alt.LocomotiveSimulation(conv, pt, SAVE_INTERVAL)
t0 = time.perf_counter()
sim.walk()
t1 = time.perf_counter()
print(f"Time to simulate: {t1 - t0:.5g}")

# %%
# Plot results
sim_dict = sim.to_pydict()
conv_rslt = sim_dict["loco_unit"]
t_s = np.array(sim_dict["power_trace"]["time_seconds"])

fig, ax = plt.subplots(3, 1, sharex=True, figsize=(12, 10))
fontsize = 14

# Power plot
ax[0].plot(
    t_s,
    np.array(
        conv_rslt["loco_type"]["ConventionalLoco"]["fc"]["history"]["pwr_fuel_watts"]
    )
    * 1e-6,
    label="Fuel Power",
    linewidth=2,
)
ax[0].plot(
    t_s,
    np.array(conv_rslt["history"]["pwr_out_watts"]) * 1e-6,
    label="Loco Output Power",
    linewidth=2,
)
ax[0].set_ylabel("Power [MW]", fontsize=fontsize)
ax[0].legend(fontsize=fontsize - 2)
ax[0].grid(True, alpha=0.3)
ax[0].set_title("Conventional Locomotive Simulation", fontsize=fontsize + 2)

# Efficiency plot
pwr_fuel = np.array(
    conv_rslt["loco_type"]["ConventionalLoco"]["fc"]["history"]["pwr_fuel_watts"]
)
pwr_shaft = np.array(
    conv_rslt["loco_type"]["ConventionalLoco"]["fc"]["history"]["pwr_shaft_watts"]
)
efficiency = np.where(pwr_fuel > 0, pwr_shaft / pwr_fuel, 0) * 100
ax[1].plot(t_s, efficiency, label="Fuel Converter Efficiency", linewidth=2, color="green")
ax[1].set_ylabel("Efficiency [%]", fontsize=fontsize)
ax[1].legend(fontsize=fontsize - 2)
ax[1].grid(True, alpha=0.3)
ax[1].set_ylim([0, 50])

# Energy plot
ax[2].plot(
    t_s,
    np.array(
        conv_rslt["loco_type"]["ConventionalLoco"]["fc"]["history"]["energy_fuel_joules"]
    )
    * 1e-9,
    label="Cumulative Fuel Energy",
    linewidth=2,
    color="orange",
)
ax[2].set_ylabel("Energy [GJ]", fontsize=fontsize)
ax[2].set_xlabel("Time [s]", fontsize=fontsize)
ax[2].legend(fontsize=fontsize - 2)
ax[2].grid(True, alpha=0.3)

plt.tight_layout()
if SHOW_PLOTS:
    plt.show()

# %%
print("=" * 70)
print("AESS Locomotive Demo - Conventional Baseline")
print("=" * 70)
print(f"\nSimulation completed in {t1 - t0:.3f} seconds")
print(f"Total simulation time: {t_s[-1]:.1f} seconds ({t_s[-1]/3600:.2f} hours)")
print(
    f"Total fuel energy consumed: {np.array(conv_rslt['loco_type']['ConventionalLoco']['fc']['history']['energy_fuel_joules'])[-1] * 1e-9:.2f} GJ"
)
print(f"Average efficiency: {np.mean(efficiency[efficiency > 0]):.2f}%")
print("\nNote: Full AESS locomotive with thermal model, air brake, and")
print("automatic engine start-stop will be demonstrated once Python")
print("bindings are added for ConventionalAESSLoco, AirBrake, and AESSController.")
print("\nAESS Features to be demonstrated:")
print("  - Starter battery SOC management (50-95% thresholds)")
print("  - Engine thermal model (120-180°F operation range)")
print("  - Air brake compressor control")
print("  - Automatic engine shutdown after 10 min delay")
print("  - Battery charging when engine is on")
print("  - Compressor power draw from battery when engine is off")

# %%
