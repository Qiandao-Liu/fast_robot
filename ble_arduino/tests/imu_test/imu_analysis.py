"""
IMU Data Analysis Utilities

Usage:
    1. Copy CSV data from Arduino Serial Monitor into a .csv file, or
       use serial_collect() to capture data directly.
    2. Load with load_imu_csv() and use the plotting/FFT functions.

CSV columns (from imu_test.ino):
    time_ms, ax, ay, az, pitch_a, roll_a, pitch_a_lpf, roll_a_lpf,
    gx, gy, gz, pitch_g, roll_g, yaw_g, pitch_comp, roll_comp
"""

import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import serial
import time


COLUMNS = [
    "time_ms", "ax", "ay", "az",
    "pitch_a", "roll_a", "pitch_a_lpf", "roll_a_lpf",
    "gx", "gy", "gz",
    "pitch_g", "roll_g", "yaw_g",
    "pitch_comp", "roll_comp",
]


def serial_collect(port="/dev/cu.usbmodem14101", baud=115200,
                   duration_s=10, skip_lines=5):
    """Collect IMU CSV data from serial port.

    Args:
        port: Serial port (check Arduino IDE for yours).
        baud: Baud rate matching Arduino Serial.begin().
        duration_s: How many seconds to collect.
        skip_lines: Header/init lines to skip.

    Returns:
        pandas DataFrame with IMU data.
    """
    ser = serial.Serial(port, baud, timeout=1)
    time.sleep(2)  # wait for Arduino reset

    lines = []
    start = time.time()
    skipped = 0
    while time.time() - start < duration_s:
        raw = ser.readline().decode("utf-8", errors="ignore").strip()
        if not raw:
            continue
        if skipped < skip_lines:
            skipped += 1
            continue
        lines.append(raw)

    ser.close()
    print(f"Collected {len(lines)} samples in {duration_s}s")

    rows = []
    for line in lines:
        parts = line.split(",")
        if len(parts) == len(COLUMNS):
            try:
                rows.append([float(x) for x in parts])
            except ValueError:
                pass

    df = pd.DataFrame(rows, columns=COLUMNS)
    # Convert time to seconds from start
    df["time_s"] = (df["time_ms"] - df["time_ms"].iloc[0]) / 1000.0
    return df


def load_imu_csv(filepath):
    """Load a CSV file saved from serial output.

    The file should have the header line from imu_test.ino or no header
    (in which case columns are assigned automatically).
    """
    try:
        df = pd.read_csv(filepath)
        if list(df.columns[:4]) != COLUMNS[:4]:
            df = pd.read_csv(filepath, header=None, names=COLUMNS)
    except Exception:
        df = pd.read_csv(filepath, header=None, names=COLUMNS)

    df["time_s"] = (df["time_ms"] - df["time_ms"].iloc[0]) / 1000.0
    return df


def plot_time_series(df, y_cols, title="IMU Data", ylabel="degrees",
                     figsize=(12, 5)):
    """Plot one or more columns vs time.

    Args:
        df: DataFrame from load_imu_csv or serial_collect.
        y_cols: list of column names to plot.
        title: Plot title.
        ylabel: Y-axis label.
    """
    plt.figure(figsize=figsize)
    for col in y_cols:
        plt.plot(df["time_s"], df[col], label=col)
    plt.xlabel("Time (s)")
    plt.ylabel(ylabel)
    plt.title(title)
    plt.legend()
    plt.grid(True, alpha=0.3)
    plt.tight_layout()
    plt.show()


def plot_pitch_roll_comparison(df):
    """Plot accel vs LPF vs gyro vs complementary for pitch and roll."""
    fig, axes = plt.subplots(2, 1, figsize=(12, 8), sharex=True)

    axes[0].plot(df["time_s"], df["pitch_a"], alpha=0.5, label="Accel (raw)")
    axes[0].plot(df["time_s"], df["pitch_a_lpf"], label="Accel (LPF)")
    axes[0].plot(df["time_s"], df["pitch_g"], label="Gyro (integrated)")
    axes[0].plot(df["time_s"], df["pitch_comp"], label="Complementary", linewidth=2)
    axes[0].set_ylabel("Pitch (deg)")
    axes[0].set_title("Pitch Comparison")
    axes[0].legend()
    axes[0].grid(True, alpha=0.3)

    axes[1].plot(df["time_s"], df["roll_a"], alpha=0.5, label="Accel (raw)")
    axes[1].plot(df["time_s"], df["roll_a_lpf"], label="Accel (LPF)")
    axes[1].plot(df["time_s"], df["roll_g"], label="Gyro (integrated)")
    axes[1].plot(df["time_s"], df["roll_comp"], label="Complementary", linewidth=2)
    axes[1].set_ylabel("Roll (deg)")
    axes[1].set_xlabel("Time (s)")
    axes[1].set_title("Roll Comparison")
    axes[1].legend()
    axes[1].grid(True, alpha=0.3)

    plt.tight_layout()
    plt.show()


def plot_fft(df, col, title=None):
    """Compute and plot FFT of a signal column.

    Args:
        df: DataFrame with 'time_s' column.
        col: Column name to analyze.
        title: Optional plot title.

    Returns:
        (freqs, magnitudes) arrays for further analysis.
    """
    signal = df[col].values
    n = len(signal)
    # Compute sampling rate from timestamps
    dt_mean = np.mean(np.diff(df["time_s"].values))
    fs = 1.0 / dt_mean

    # FFT
    fft_vals = np.fft.rfft(signal - np.mean(signal))  # remove DC
    freqs = np.fft.rfftfreq(n, d=dt_mean)
    magnitudes = np.abs(fft_vals) * 2.0 / n

    plt.figure(figsize=(10, 4))
    plt.plot(freqs, magnitudes)
    plt.xlabel("Frequency (Hz)")
    plt.ylabel("Magnitude")
    plt.title(title or f"FFT of {col}")
    plt.grid(True, alpha=0.3)
    plt.xlim(0, fs / 2)
    plt.tight_layout()
    plt.show()

    print(f"Sampling rate: {fs:.1f} Hz, Nyquist: {fs/2:.1f} Hz")
    return freqs, magnitudes


def plot_accel_fft(df):
    """Plot FFT for all three accelerometer axes."""
    fig, axes = plt.subplots(3, 1, figsize=(10, 9), sharex=True)
    dt_mean = np.mean(np.diff(df["time_s"].values))
    fs = 1.0 / dt_mean

    for i, col in enumerate(["ax", "ay", "az"]):
        signal = df[col].values
        n = len(signal)
        fft_vals = np.fft.rfft(signal - np.mean(signal))
        freqs = np.fft.rfftfreq(n, d=dt_mean)
        magnitudes = np.abs(fft_vals) * 2.0 / n

        axes[i].plot(freqs, magnitudes)
        axes[i].set_ylabel(f"|{col}|")
        axes[i].set_title(f"FFT of {col}")
        axes[i].grid(True, alpha=0.3)
        axes[i].set_xlim(0, fs / 2)

    axes[-1].set_xlabel("Frequency (Hz)")
    plt.suptitle("Accelerometer Frequency Spectrum", y=1.01)
    plt.tight_layout()
    plt.show()
    print(f"Sampling rate: {fs:.1f} Hz")


def two_point_calibration(measured_low, measured_high,
                          expected_low=-90.0, expected_high=90.0):
    """Compute scale and offset for two-point calibration.

    Args:
        measured_low: Measured value at expected_low position.
        measured_high: Measured value at expected_high position.
        expected_low: True value at low position (default -90).
        expected_high: True value at high position (default 90).

    Returns:
        (scale, offset) such that corrected = scale * measured + offset.
    """
    scale = (expected_high - expected_low) / (measured_high - measured_low)
    offset = expected_high - scale * measured_high
    print(f"Scale: {scale:.6f}, Offset: {offset:.4f}")
    print(f"Put these in imu_test.ino: pitch_scale={scale:.6f}, pitch_offset={offset:.4f}")
    return scale, offset
