#!/usr/bin/env python3
"""
Location Manager Module
=======================
Manages named locations for the pick-and-place system.

This module loads location configurations from YAML and provides
lookup functionality to convert QR code content (location names)
into navigation coordinates.

Usage:
    from location_manager import LocationManager

    # Initialize with path to locations.yaml
    manager = LocationManager('/path/to/locations.yaml')

    # Get coordinates for a location name (from QR code)
    coords = manager.get_location('station_A')
    # Returns: {'x': 2.0, 'y': 1.0, 'theta': 1.57}

    # Get pickup location
    pickup = manager.get_pickup_location()

    # Get default fallback location
    default = manager.get_default_location()
"""

import yaml
from typing import Dict, Optional
from dataclasses import dataclass


@dataclass
class LocationCoordinates:
    """
    Represents a navigation location with position and orientation.

    Attributes:
        x: X position in map frame (meters)
        y: Y position in map frame (meters)
        theta: Orientation in map frame (radians)
        name: Location name (for reference)
    """
    x: float
    y: float
    theta: float
    name: str = ""

    def to_dict(self) -> Dict[str, float]:
        """Convert to dictionary format."""
        return {'x': self.x, 'y': self.y, 'theta': self.theta}


class LocationManager:
    """
    Manages named locations loaded from YAML configuration.

    The manager loads locations from a YAML file and provides methods
    to look up coordinates by name. Location names correspond to QR
    code content scanned during pick-and-place operations.

    Attributes:
        config_path: Path to the locations.yaml file
        locations: Dictionary of LocationCoordinates by name
    """

    def __init__(self, config_path: str):
        """
        Initialize the location manager.

        Args:
            config_path: Path to the locations.yaml configuration file

        Raises:
            FileNotFoundError: If config file doesn't exist
            yaml.YAMLError: If config file is invalid YAML
            KeyError: If required 'locations' key is missing
        """
        self.config_path = config_path
        self.locations: Dict[str, LocationCoordinates] = {}
        self._load_config()

    def _load_config(self) -> None:
        """
        Load and parse the locations configuration file.

        Reads the YAML file and creates LocationCoordinates objects
        for each defined location.
        """
        with open(self.config_path, 'r') as f:
            config = yaml.safe_load(f)

        if 'locations' not in config:
            raise KeyError("Config file must contain 'locations' key")

        for name, coords in config['locations'].items():
            self.locations[name] = LocationCoordinates(
                x=float(coords.get('x', 0.0)),
                y=float(coords.get('y', 0.0)),
                theta=float(coords.get('theta', 0.0)),
                name=name
            )

    def get_location(self, name: str) -> Optional[LocationCoordinates]:
        """
        Get coordinates for a named location.

        Args:
            name: Location name (typically from QR code content)

        Returns:
            LocationCoordinates if found, None otherwise
        """
        return self.locations.get(name)

    def get_location_or_default(self, name: str) -> LocationCoordinates:
        """
        Get coordinates for a named location, falling back to default.

        Args:
            name: Location name (typically from QR code content)

        Returns:
            LocationCoordinates for the named location, or the default
            location if name is not found.

        Raises:
            KeyError: If neither the named location nor 'default' exists
        """
        location = self.get_location(name)
        if location is not None:
            return location

        default = self.get_location('default')
        if default is not None:
            return default

        raise KeyError(f"Location '{name}' not found and no default defined")

    def get_pickup_location(self) -> LocationCoordinates:
        """
        Get the pickup station coordinates.

        Returns:
            LocationCoordinates for the 'pickup' location

        Raises:
            KeyError: If 'pickup' location is not defined
        """
        pickup = self.get_location('pickup')
        if pickup is None:
            raise KeyError("Pickup location not defined in configuration")
        return pickup

    def get_default_location(self) -> Optional[LocationCoordinates]:
        """
        Get the default fallback location.

        Returns:
            LocationCoordinates for 'default' location, or None if not defined
        """
        return self.get_location('default')

    def location_exists(self, name: str) -> bool:
        """
        Check if a location name exists in the configuration.

        Args:
            name: Location name to check

        Returns:
            True if location exists, False otherwise
        """
        return name in self.locations

    def list_locations(self) -> list:
        """
        Get list of all defined location names.

        Returns:
            List of location name strings
        """
        return list(self.locations.keys())

    def reload_config(self) -> None:
        """
        Reload the configuration file.

        Useful for updating locations at runtime without restarting.
        """
        self.locations.clear()
        self._load_config()


# =============================================================================
# Standalone testing
# =============================================================================
if __name__ == '__main__':
    import sys

    if len(sys.argv) < 2:
        print("Usage: python3 location_manager.py <path_to_locations.yaml>")
        sys.exit(1)

    config_path = sys.argv[1]
    print(f"Loading locations from: {config_path}")

    try:
        manager = LocationManager(config_path)
        print(f"\nLoaded {len(manager.locations)} locations:")

        for name in manager.list_locations():
            loc = manager.get_location(name)
            print(f"  - {name}: x={loc.x:.2f}, y={loc.y:.2f}, theta={loc.theta:.2f}")

        # Test lookup
        print("\nTesting lookups:")
        print(f"  pickup: {manager.get_pickup_location()}")
        print(f"  station_A: {manager.get_location('station_A')}")
        print(f"  unknown (with default): {manager.get_location_or_default('unknown')}")

    except Exception as e:
        print(f"Error: {e}")
        sys.exit(1)
