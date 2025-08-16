#!/usr/bin/env python3
"""
AUV Simulation Plot Generator
============================

Simple utility script to generate plots from simulation results.
Can be used to create visualizations from any existing simulation data.

Usage:
    python generate_plots.py                    # Plot latest results
    python generate_plots.py --show             # Plot and display
    python generate_plots.py --data results/logs/simulation_data_20240101_120000.csv
"""

import argparse
import sys
from pathlib import Path

# Add visualization to path
sys.path.append(str(Path(__file__).parent / "visualization"))

from plot_results import plot_latest_results, AUVResultsPlotter


def main():
    """Main function for plot generation utility."""
    parser = argparse.ArgumentParser(description='Generate AUV simulation plots')
    parser.add_argument('--data', type=str, help='Specific CSV data file to plot')
    parser.add_argument('--metadata', type=str, help='Corresponding metadata JSON file')
    parser.add_argument('--show', action='store_true', help='Display plots after generation')
    parser.add_argument('--results-dir', type=str, default='results',
                       help='Directory containing results (default: results)')
    
    args = parser.parse_args()
    
    print("="*60)
    print("AUV SIMULATION PLOT GENERATOR")
    print("="*60)
    
    try:
        if args.data:
            # Plot specific data file
            print(f"Plotting specific data file: {args.data}")
            
            if not Path(args.data).exists():
                print(f"Error: Data file not found: {args.data}")
                return 1
            
            plotter = AUVResultsPlotter(args.data, args.metadata)
            plotter.create_all_plots(show=args.show)
            
        else:
            # Plot latest results
            print(f"Plotting latest results from: {args.results_dir}")
            
            plotter = plot_latest_results(args.results_dir, show=args.show)
            if not plotter:
                print("Error: No simulation data found")
                return 1
        
        print(f"\n✓ Plot generation completed successfully!")
        return 0
        
    except Exception as e:
        print(f"Error: Failed to generate plots: {e}")
        import traceback
        traceback.print_exc()
        return 1


if __name__ == "__main__":
    sys.exit(main())
