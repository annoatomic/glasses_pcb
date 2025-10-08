import socket
import threading
from collections import deque
import numpy as np
import sounddevice as sd
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import time



# ------------------ UDP-/Audio-Parameter ------------------
UDP_IP = ""           # horcht an allen Schnittstellen
UDP_PORT = 7000
MTU_PAYLOAD = 1024    # 1024 Bytes = 128 Frames × 8 Bytes (4 Kanäle × 16 Bit)
CHANNELS = 4          # auf 4 Kanäle reduziert
SR = 16000

BUFFER_LEN = 4096
FILL_SECONDS = 0.5    # Vorlauf für den Playback-Puffer

# Globale Puffer & Locks
audio_buffer = np.zeros((BUFFER_LEN, CHANNELS), dtype=np.int16)
buffer_pos = 0
lock = threading.Lock()
sound_buffer = deque(maxlen=SR * 10)

def soft_clip(x: np.ndarray, threshold: float = 0.95) -> np.ndarray:
    norm = x.astype(np.float32) / 32767.0
    clipped = np.where(
        np.abs(norm) > threshold,
        threshold * np.sign(norm)
        + (1 - threshold)
        * np.tanh((np.abs(norm) - threshold) / (1 - threshold))
        * np.sign(norm),
        norm,
    )
    return (clipped * 32767).astype(np.int16)

def udp_receiver():
    global buffer_pos
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind((UDP_IP, UDP_PORT))
    print(f"Höre auf UDP-Port {UDP_PORT}...")

    while True:
        data, _ = sock.recvfrom(MTU_PAYLOAD + 32)
        if len(data) < MTU_PAYLOAD:
            continue

        # Direkt als Big-Endian int16 lesen
        samples = np.frombuffer(data[:MTU_PAYLOAD], dtype=">i2")
        try:
            samples = samples.reshape(-1, CHANNELS)
        except ValueError:
            continue

        # Debug-Ausgabe
        print(f"Empfangen: {samples.shape[0]} Frames mit {CHANNELS} Kanälen")

        # Ringpuffer schreiben
        with lock:
            take = min(BUFFER_LEN - buffer_pos, len(samples))
            audio_buffer[buffer_pos:buffer_pos + take] = samples[:take]
            buffer_pos = (buffer_pos + take) % BUFFER_LEN
            leftover = len(samples) - take
            if leftover > 0:
                audio_buffer[0:leftover] = samples[take:take + leftover]
                buffer_pos = leftover

        sound_buffer.extend(samples.tolist())

def audio_callback(outdata, frames, time_info, status):
    if status:
        print(status)
    data = np.zeros((frames, CHANNELS), dtype=np.int16)
    for i in range(frames):
        try:
            data[i] = sound_buffer.popleft()
        except IndexError:
            data[i] = 0
    # Nur die ersten 2 Kanäle für Stereo ausgeben
    outdata[:] = soft_clip(data[:, :2])

def animate(_):
    with lock:
        if buffer_pos < BUFFER_LEN // 2:
            plot_data = np.vstack((audio_buffer[buffer_pos:], audio_buffer[:buffer_pos]))
        else:
            plot_data = audio_buffer.copy()
    for ch in range(CHANNELS):
        axs[ch].clear()
        axs[ch].plot(plot_data[:, ch], label=f"Kanal {ch+1}", alpha=0.8)
        axs[ch].set_ylim([-32768, 32767])
        axs[ch].legend(loc="upper right")
        axs[ch].set_title(f"Kanal {ch+1}")
    for ch in range(CHANNELS, len(axs)):
        axs[ch].axis("off")

if __name__ == "__main__":
    # UDP-Empfangsthread
    t = threading.Thread(target=udp_receiver, daemon=True)
    t.start()

    if FILL_SECONDS > 0:
        print(f"Fülle Puffer für {FILL_SECONDS} s…")
        time.sleep(FILL_SECONDS)

    stream = sd.OutputStream(
        samplerate=SR,
        channels=2,
        dtype="int16",
        callback=audio_callback,
        blocksize=256,
        latency="high",
    )

    fig, axs_grid = plt.subplots(2, 2, figsize=(12, 8))
    axs = axs_grid.flatten()

    with stream:
        ani = FuncAnimation(fig, animate, interval=30, cache_frame_data=False)
        plt.tight_layout()
        plt.show()
