#!/usr/bin/env python3
"""
Waypoint Generator Script

This script generates waypoints for a path and saves them to a CSV file.
Supported path types:
  - circle: points on a circle of given radius
  - line: straight line between two x positions at constant y
  - sine: sine wave between two x positions

Usage examples:
  # Circle of radius 5, 100 points
  python3 waypoint_generator.py --type circle --radius 5.0 --num_points 100 --output waypoints.csv

  # Straight line from x=0 to x=10 at y=2, 50 points
  python3 waypoint_generator.py --type line --x_start 0 --x_end 10 --y 2 --num_points 50 --output waypoints.csv

  # Sine wave from x=0 to x=10, amplitude=1, frequency=0.2 Hz, 200 points
  python3 waypoint_generator.py --type sine --x_start 0 --x_end 10 --amplitude 1.0 --frequency 0.2 --num_points 200 --output waypoints.csv
"""
import numpy as np
import argparse

def generate_circle(radius, num_points):
    angles = np.linspace(0, 2 * np.pi, num_points, endpoint=False)
    x = radius * np.cos(angles)
    y = radius * np.sin(angles)
    z = np.zeros_like(x)
    return np.stack((x, y, z), axis=1)

def generate_line(x_start, x_end, y, num_points):
    x = np.linspace(x_start, x_end, num_points)
    y_vals = np.full_like(x, y)
    z = np.zeros_like(x)
    return np.stack((x, y_vals, z), axis=1)

def generate_sine(x_start, x_end, amplitude, frequency, num_points):
    x = np.linspace(x_start, x_end, num_points)
    y = amplitude * np.sin(2 * np.pi * frequency * x)
    z = np.zeros_like(x)
    return np.stack((x, y, z), axis=1)

def main():
    parser = argparse.ArgumentParser(
        description="Generate waypoints and save to CSV"
    )
    parser.add_argument(
        '--type', choices=['circle', 'line', 'sine'], default='circle',
        help='Type of path to generate (default: circle)'
    )
    parser.add_argument(
        '--output', default='waypoints.csv',
        help='Output CSV filename (default: waypoints.csv)'
    )
    parser.add_argument(
        '--num_points', type=int, default=100,
        help='Number of waypoints to generate (default: 100)'
    )
    parser.add_argument('--radius', type=float, default=5.0, help='Radius for circle (default: 5.0)')
    parser.add_argument('--x_start', type=float, default=0.0, help='Start x for line/sine (default: 0.0)')
    parser.add_argument('--x_end', type=float, default=10.0, help='End x for line/sine (default: 10.0)')
    parser.add_argument('--y', type=float, default=0.0, help='Constant y for line (default: 0.0)')
    parser.add_argument('--amplitude', type=float, default=1.0, help='Amplitude for sine (default: 1.0)')
    parser.add_argument('--frequency', type=float, default=0.1, help='Frequency for sine (Hz, default: 0.1)')
    args = parser.parse_args()

    if args.type == 'circle':
        waypoints = generate_circle(args.radius, args.num_points)
    elif args.type == 'line':
        waypoints = generate_line(args.x_start, args.x_end, args.y, args.num_points)
    elif args.type == 'sine':
        waypoints = generate_sine(
            args.x_start, args.x_end, args.amplitude, args.frequency, args.num_points
        )
    else:
        raise ValueError(f"Unknown path type: {args.type}")

    # Save to CSV with header
    header = 'x,y,z'
    np.savetxt(
        args.output,
        waypoints,
        delimiter=',',
        header=header,
        comments=''
    )
    print(f"Generated {args.num_points} {args.type} waypoints and saved to '{args.output}'")

if __name__ == '__main__':
    main()
