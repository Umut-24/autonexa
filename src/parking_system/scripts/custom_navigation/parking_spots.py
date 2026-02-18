#!/usr/bin/env python3
"""
Parking Spots Manager
Loads and manages predefined parking spots
"""

import yaml
from typing import Dict, Optional, Tuple, List


class ParkingSpotsManager:
    """Manages parking spot definitions"""
    
    def __init__(self, spots_file: str):
        """
        Initialize parking spots manager
        
        Args:
            spots_file: Path to parking_spots.yaml
        """
        self.spots_file = spots_file
        self.spots = {}
        self.load_spots()
    
    def load_spots(self):
        """Load parking spots from YAML file"""
        try:
            with open(self.spots_file, 'r') as f:
                data = yaml.safe_load(f) or {}
                self.spots = data.get('parking_spots', {})
        except FileNotFoundError:
            print(f"Warning: Spots file not found: {self.spots_file}")
            self.spots = {}
    
    def get_spot(self, spot_id: str) -> Optional[Dict]:
        """
        Get parking spot by ID
        
        Args:
            spot_id: Spot ID (e.g., 'spot_1')
        
        Returns:
            Dict with x, y, yaw, description, or None if not found
        """
        return self.spots.get(spot_id)
    
    def get_all_spots(self) -> Dict:
        """Get all parking spots"""
        return self.spots.copy()
    
    def get_spot_position(self, spot_id: str) -> Optional[Tuple[float, float]]:
        """
        Get spot position (x, y) in world coordinates
        
        Args:
            spot_id: Spot ID
        
        Returns:
            (x, y) tuple or None
        """
        spot = self.get_spot(spot_id)
        if spot:
            return (spot['x'], spot['y'])
        return None
    
    def list_spot_ids(self) -> List[str]:
        """Get list of all spot IDs"""
        return list(self.spots.keys())

