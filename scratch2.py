import pandas as pd
import numpy as np

df = pd.read_csv('../../lap_tests/trial128.csv')

# Find when lateral error started being large
print(df[df['lateral_error_m'] > 1.0].head(10)[['time_s', 's_m', 'x', 'y', 'v_ms', 'lateral_error_m', 'delta_rad', 'acc_ms2']])
print("---")
print(df[df['lateral_error_m'] > 0.5].head(5)[['time_s', 's_m', 'x', 'y', 'v_ms', 'lateral_error_m', 'delta_rad', 'acc_ms2']])
print("---")
# also print max speed
print(f"Max speed: {df['v_ms'].max()}")
