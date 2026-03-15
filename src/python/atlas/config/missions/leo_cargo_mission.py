"""
leo_cargo_mission.py — LEO Cargo Mission Configuration
"""

from dataclasses import dataclass


@dataclass
class LEOCargoMission:
    mission_id:          str
    target_altitude_km:  int   = 550
    inclination_deg:     float = 28.5
    payload_mass_kg:     int   = 3500
    payload_name:        str   = "Generic Cargo"

    def configure(self, target_altitude_km=550, payload_mass_kg=3500, inclination_deg=28.5):
        self.target_altitude_km = target_altitude_km
        self.payload_mass_kg    = payload_mass_kg
        self.inclination_deg    = inclination_deg
        return self
