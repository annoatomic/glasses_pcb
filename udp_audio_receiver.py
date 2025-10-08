# udp_stereo_plotter.py
# Empfängt Stereo PCM16 (LE) über UDP (z.B. vom ESP32), spielt ab und plottet live.

import argparse
import socket
import threading
import time
from collections import deque

import numpy as np
import sounddevice as sd
from matplotlib.animation import FuncAnimation
import matplotlib.pyplot as plt


# --- CLI ---
def parse_args():
    p = argparse.ArgumentParser(description="UDP Stereo PCM16 (LE) Receiver: Play + Plot")
    p.add_argument("--port", type=int, default=7000, help="UDP-Port zum Empfangen (default: 7000)")
    p.add_argument("--sr", type=int, default=48000, help="Samplerate (Hz) (default: 48000)")
    p.add_argument("--prefill", type=float, default=0.5, help="Start-Puffer in Sekunden (default: 0.5)")
    p.add_argument("--plot-sec", type=float, default=0.25, help="Plotfenster in Sekunden (default: 0.25)")
    p.add_argument("--block", type=int, default=256, help="Audio-Blockgröße (Frames) (default: 256)")
    p.add_argument("--device", type=str, default=None, help="Ausgabegerät-Name/Index (optional)")
    return p.parse_args()


# --- Globale (per args) ---
args = parse_args()
PORT = args.port
SR = args.sr
CHANNELS = 2
BYTES_PER_FRAME = CHANNELS * 2  # 16-bit stereo, little-endian
PREFILL_SEC = args.prefill
PLOT_SECONDS = args.plot_sec
AUDIO_BLOCK = args.block
AUDIO_DEVICE = args.device

# --- Speicher/Queues ---
audio_queue = deque()  # Blöcke für Audio (wird geleert)
plot_len = int(SR * PLOT_SECONDS)
plot_buf = np.zeros((plot_len, CHANNELS), dtype=np.int16)  # Ringpuffer nur für Plot (bleibt immer gefüllt)

lock = threading.Lock()
queued_frames = 0
running = True

# Matplotlib Animation-Referenz festhalten, damit sie nicht von GC zerstört wird
_ani_ref = None


def udp_receiver():
    """Empfängt UDP-Pakete (roh, LE PCM16 Stereo) und füttert Audio-Queue + Plot-Ringpuffer.
       Nebenbei: Durchsatz-Monitor (Bytes/s) zur Rate-Diagnose."""
    global queued_frames, running

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    # Größeren Kernel-Rx-Puffer versuchen (hilft bei Bursts)
    try:
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 1 << 20)
    except Exception:
        pass
    sock.bind(("", PORT))
    print(f"[UDP] Empfang auf Port {PORT}")

    # --- Durchsatz-Monitor ---
    recv_bytes = 0
    last_t = time.time()

    while running:
        data, _ = sock.recvfrom(65536)  # groß genug für 1-2 UDP-Pakete
        if not data:
            continue
        if len(data) % BYTES_PER_FRAME != 0:
            # Falsche Paketgröße -> ignorieren
            continue

        # Durchsatz zählen
        recv_bytes += len(data)
        now = time.time()
        if now - last_t >= 1.0:
            exp = SR * BYTES_PER_FRAME  # erwartete Bytes/s
            pct = (recv_bytes / exp * 100.0) if exp > 0 else 0.0
            print(f"In: {recv_bytes:7d} B/s  ({pct:5.1f}% von erwartet)")
            recv_bytes = 0
            last_t = now

        frames = len(data) // BYTES_PER_FRAME

        # In numpy-Block umwandeln (Little-Endian 16-bit)
        block = np.frombuffer(data, dtype="<i2").reshape(frames, CHANNELS).copy()

        # In Audio-Queue + Plot-Ringpuffer
        with lock:
            audio_queue.append(block)
            queued_frames += frames

            # Plot-Ringpuffer aktualisieren
            n = len(block)
            if n >= plot_len:
                plot_buf[:] = block[-plot_len:]
            else:
                plot_buf[:-n] = plot_buf[n:]
                plot_buf[-n:] = block


def audio_writer():
    """Spielt Audio aus der Queue. Bei Leerlauf wird Stille geschrieben (verhindert hörbare Underruns)."""
    global queued_frames, running
    silence = np.zeros((AUDIO_BLOCK, CHANNELS), dtype=np.int16)
    stream_kwargs = dict(samplerate=SR, channels=CHANNELS, dtype="int16", blocksize=AUDIO_BLOCK)
    if AUDIO_DEVICE is not None:
        stream_kwargs["device"] = AUDIO_DEVICE

    with sd.OutputStream(**stream_kwargs) as stream:
        while running:
            with lock:
                blk = audio_queue.popleft() if audio_queue else None
                if blk is not None:
                    queued_frames -= len(blk)

            if blk is None:
                stream.write(silence)
            else:
                # Blöcke ggf. in AUDIO_BLOCK-Häppchen zerlegen
                i = 0
                while i < len(blk):
                    chunk = blk[i:i + AUDIO_BLOCK]
                    i += len(chunk)
                    if len(chunk) < AUDIO_BLOCK:
                        # Rest am Ende mit Stille auffüllen
                        pad = np.zeros((AUDIO_BLOCK - len(chunk), CHANNELS), dtype=np.int16)
                        chunk = np.vstack((chunk, pad))
                    stream.write(chunk)


def start_plot():
    """Startet das Live-Plot-Fenster (2 Kanäle)."""
    global _ani_ref

    fig, (axL, axR) = plt.subplots(2, 1, sharex=True, figsize=(10, 5))
    x = np.arange(plot_len)
    lineL, = axL.plot(x, plot_buf[:, 0])
    lineR, = axR.plot(x, plot_buf[:, 1])
    for ax in (axL, axR):
        ax.set_ylim([-32768, 32767])
    axR.set_xlabel("Samples")
    axL.set_title("Left")
    axR.set_title("Right")
    fig.tight_layout()

    def update(_):
        with lock:
            lineL.set_ydata(plot_buf[:, 0])
            lineR.set_ydata(plot_buf[:, 1])
            qf = queued_frames
        fig.suptitle(f"Queue ~{qf / SR:.3f}s")
        return lineL, lineR

    _ani_ref = FuncAnimation(fig, update, interval=33, blit=False, cache_frame_data=False)
    plt.show()


def main():
    # Receiver starten
    t_rx = threading.Thread(target=udp_receiver, daemon=True)
    t_rx.start()

    # Prefill abwarten
    target = int(PREFILL_SEC * SR)
    print(f"Fülle Puffer bis ~{PREFILL_SEC:.2f}s…")
    try:
        while True:
            time.sleep(0.01)
            with lock:
                if queued_frames >= target:
                    break
    except KeyboardInterrupt:
        return

    print("Starte Audio & Plot…")

    # Audio starten
    t_audio = threading.Thread(target=audio_writer, daemon=True)
    t_audio.start()

    # Plot starten (blockiert bis Fenster zu)
    try:
        start_plot()
    finally:
        global running
        running = False
        time.sleep(0.1)


if __name__ == "__main__":
    main()
