"""
falcon9_class.py
Falcon 9 Class Vehicle Configuration
All specs from public SpaceX data / CRS press kits / Falcon 9 payload user's guide.
"""

from dataclasses import dataclass


@dataclass
class Falcon9ClassVehicle:
    vehicle_id: str
    name: str = "Falcon 9 Block 5"

    # Performance (SpaceX Falcon 9 User's Guide, Rev 2)
    payload_leo_kg:     int   = 22800    # kg to LEO (28.5°)
    payload_gto_kg:     int   = 8300     # kg to GTO
    payload_mars_kg:    int   = 4020     # kg to Mars

    # Mass (kg) — public data
    liftoff_mass_kg:    int   = 549054
    first_stage_dry:    int   = 22200
    second_stage_dry:   int   = 4500
    propellant_s1_kg:   int   = 395700   # LOX/RP-1
    propellant_s2_kg:   int   = 92670

    # Dimensions
    height_m:           float = 70.0    # m (with fairing)
    diameter_m:         float = 3.66    # m
    fairing_dia_m:      float = 5.2     # m (standard)

    # Propulsion
    engine_model:       str   = "Merlin 1D+"
    engine_count:       int   = 9       # first stage
    thrust_sl_kn:       int   = 7607    # kN sea level total (9 engines)
    thrust_vac_kn:      int   = 8227    # kN vacuum total

    # Second stage
    s2_engine:          str   = "Merlin Vacuum"
    s2_thrust_vac_kn:   int   = 934
    s2_isp_s:           int   = 348     # s

    # Reusability
    reusable:           bool  = True
    landing_legs:       int   = 4
    grid_fins:          int   = 4

    # Flight constraints (structural)
    max_q_pa:           float = 35000.0  # Pa (design limit)
    max_axial_g:        float = 5.5      # g
    stage_count:        int   = 2

    def configure(self, payload_mass_kg: int = 3500):
        self.payload_mass_kg = payload_mass_kg
        return self
