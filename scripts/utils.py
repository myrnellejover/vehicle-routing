## utils.py
## This module contains universal helper functions.

import os
import sys
from dotenv import dotenv_values

# Add the parent directory to sys.path
CURR_DIR: str = os.path.dirname(os.path.abspath(__file__))
HOME_DIR: str = os.path.dirname(CURR_DIR)
sys.path.insert(0, HOME_DIR)

# Data
import numpy as np
import pandas as pd
import googlemaps
import webbrowser
import folium

# Solvers
from ortools.constraint_solver import routing_enums_pb2
from ortools.constraint_solver import pywrapcp

# Classes
from typing import Optional, Union, List, Dict, Any

# Local modules
from scripts.config import BASE_CONFIG, DATA_PATH

# Paths
CONFIG_FILE: str = BASE_CONFIG.project_config
DATA_DEPOTS: str = DATA_PATH.depots
DATA_STORES: str = DATA_PATH.stores

# Global vars
GOOGLE_MAPS_API: str = dotenv_values(CONFIG_FILE).get("GOOGLE_MAPS_API_KEY")


def load_csv(file_path: str) -> pd.DataFrame:
    """Load a CSV file into a pandas DataFrame."""
    return pd.read_csv(file_path)


def get_distance_matrix(locations: pd.DataFrame) -> np.ndarray:
    """Get distance matrix from Google Maps API with chunk handling."""
    gmaps = googlemaps.Client(key=GOOGLE_MAPS_API)
    coords = locations[["latitude", "longitude"]].values.tolist()
    num_locations = len(coords)

    distance_matrix = np.zeros((num_locations, num_locations))

    # Split into chunks
    chunk_size = 10  # Adjust chunk size as needed
    for i in range(0, num_locations, chunk_size):
        origins = coords[i : i + chunk_size]
        for j in range(0, num_locations, chunk_size):
            destinations = coords[j : j + chunk_size]
            matrix = gmaps.distance_matrix(
                origins=origins,
                destinations=destinations,
                mode="driving",
                units="metric",
            )

            for m in range(len(origins)):
                for n in range(len(destinations)):
                    distance_matrix[i + m, j + n] = (
                        matrix["rows"][m]["elements"][n]["distance"]["value"] / 1000
                    )  # Convert meters to kilometres

    return distance_matrix


def get_address_from_lat_lng(
    latitude: float, longitude: float, gmaps: googlemaps.Client
) -> str:
    """Get address from latitude and longitude using Google Maps API."""
    reverse_geocode_result = gmaps.reverse_geocode((latitude, longitude))
    if reverse_geocode_result:
        return reverse_geocode_result[0]["formatted_address"]
    return "Unknown Address"


def create_data_model(df_depots: pd.DataFrame, df_stores: pd.DataFrame) -> Dict:
    """Create data model for the VRP."""
    data = {}
    locations = pd.concat([df_depots, df_stores], ignore_index=True)
    data["distance_matrix"] = get_distance_matrix(locations)
    data["num_vehicles"] = len(df_depots)
    data["depot"] = list(range(len(df_depots)))
    data["num_locations"] = len(locations)
    return data


def print_solution(
    manager: pywrapcp.RoutingIndexManager,
    routing: pywrapcp.RoutingModel,
    solution: pywrapcp.Assignment,
    locations: pd.DataFrame,
    gmaps: googlemaps.Client,
) -> List[List[int]]:
    """Print the solution with street names."""
    routes = []
    for vehicle_id in range(data["num_vehicles"]):
        index = routing.Start(vehicle_id)
        route = []
        while not routing.IsEnd(index):
            route.append(manager.IndexToNode(index))
            index = solution.Value(routing.NextVar(index))
        route.append(manager.IndexToNode(index))
        routes.append(route)

    print("Routes:")
    for route in routes:
        route_addresses = [
            get_address_from_lat_lng(
                locations.iloc[location]["latitude"],
                locations.iloc[location]["longitude"],
                gmaps,
            )
            for location in route
        ]
        print(" -> ".join(route_addresses))

    return routes


def visualise_solution(routes: List[List[int]], locations: pd.DataFrame):
    """Visualise the solution using Folium."""
    # Initialise the map centred at the first depot
    depot_location = locations.iloc[data["depot"][0]]
    m = folium.Map(
        location=[depot_location["latitude"], depot_location["longitude"]],
        zoom_start=12,
    )

    # Plot depot locations
    for depot in df_depots.itertuples():
        folium.Marker(
            location=[depot.latitude, depot.longitude],
            popup=f"Depot: {depot.name}",
            icon=folium.Icon(color="blue"),
        ).add_to(m)

    # Plot store locations
    for store in df_stores.itertuples():
        folium.Marker(
            location=[store.latitude, store.longitude],
            popup=f"Store: {store.name}",
            icon=folium.Icon(color="red"),
        ).add_to(m)

    # Plot routes
    for route in routes:
        route_coords = locations.iloc[route][["latitude", "longitude"]].values.tolist()
        folium.PolyLine(route_coords, color="green", weight=2.5, opacity=0.8).add_to(m)

    # Save map to HTML file
    m.save("vrp_solution_map.html")


def solve_vrp(data: Dict, locations: pd.DataFrame, visualise: bool = True):
    """Solve the Vehicle Routing Problem."""
    gmaps = googlemaps.Client(key=GOOGLE_MAPS_API)
    manager = pywrapcp.RoutingIndexManager(
        data["num_locations"], data["num_vehicles"], data["depot"][0]
    )
    routing = pywrapcp.RoutingModel(manager)

    def distance_callback(from_index: int, to_index: int) -> int:
        """Returns the distance between the two nodes."""
        from_node = manager.IndexToNode(from_index)
        to_node = manager.IndexToNode(to_index)
        return int(
            data["distance_matrix"][from_node][to_node] * 1000
        )  # Convert km to meters

    transit_callback_index = routing.RegisterTransitCallback(distance_callback)
    routing.SetArcCostEvaluatorOfAllVehicles(transit_callback_index)

    # Set parameters.
    search_parameters = pywrapcp.DefaultRoutingSearchParameters()
    search_parameters.first_solution_strategy = (
        routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC
    )

    # Solve the problem.
    solution = routing.SolveWithParameters(search_parameters)
    if solution:
        routes = print_solution(manager, routing, solution, locations, gmaps)
        if visualise:
            visualise_solution(routes, locations)
    else:
        print("No solution found!")
