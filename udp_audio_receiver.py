# udp_multich_rx.py
# Empfängt N-kanaliges PCM16 (LE) über UDP (z.B. vom ESP32),
# plottet alle 6 Kanäle live und spielt 2 wählbare Kanäle ab.
#
# Keys im Plot-Fenster:
#   [ / ]   : linken Play-Kanal - / +
#   ; / '   : rechten Play-Kanal - / +
#   TAB     : L/R tauschen
#   m       : Mono (Downmix der zwei gewählten)
#   p       : Stereo (zwei gewählte Kanäle)
#   0       : Mute (keine Wiedergabe)

import argparse
import socket
import threading
import time
from collections import deque

import numpy as np
import sounddevice as sd
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

# ---------- CLI ----------
def parse_args():
    p = argparse.ArgumentParser(description="UDP Multichannel PCM16 (LE) Receiver: Plot 6ch + switchable playback")
    p.add_argument("--port", type=int, default=7000, help="UDP-Port (default: 7000)")
    p.add_argument("--sr", type=int, default=48000, help="Samplerate Hz (default: 48000)")
    p.add_argument("--ch", type=int, default=6, help="Empfangene Kanäle (default: 6)")
    p.add_argument("--pick", type=str, default="0,1", help="Start-Playkanäle 'L,R' (z.B. '0,1')")
    p.add_argument("--prefill", type=float, default=0, help="Startpuffer Sekunden (default: 0)")
    p.add_argument("--plot-sec", type=float, default=0.5, help="Plotfenster Sekunden (default: 0.5)")
    p.add_argument("--block", type=int, default=256, help="Audio-Blocksize Frames (default: 256)")
    p.add_argument("--device", type=str, default=None, help="Sounddevice Name/Index (optional)")
    return p.parse_args()

args = parse_args()

PORT            = args.port
SR              = args.sr
RX_CHANNELS     = args.ch
PREFILL_SEC     = args.prefill
PLOT_SECONDS    = args.plot_sec
AUDIO_BLOCK     = args.block
AUDIO_DEVICE    = args.device
BYTES_PER_FRAME = RX_CHANNELS * 2  # 16-bit * N

# Play-Status (wird per Tastatur geändert)
pickL, pickR = (int(x) for x in args.pick.split(","))
pickL = max(0, min(RX_CHANNELS - 1, pickL))
pickR = max(0, min(RX_CHANNELS - 1, pickR))
play_mode = "stereo"  # "stereo" | "mono" | "mute"

# Queues & Buffer
raw_queue = deque()  # enthält Blöcke in Form (frames, RX_CHANNELS), dtype int16
plot_len  = int(SR * PLOT_SECONDS)
plot_buf  = np.zeros((plot_len, RX_CHANNELS), dtype=np.int16)  # Ringpuffer für alle Kanäle

lock = threading.Lock()
queued_frames = 0
running = True

# Für Matplotlib, damit Animation nicht vom GC zerstört wird
_ani_ref = None

# ---------- UDP Receiver ----------
def udp_receiver():
    """Empfängt UDP-Pakete (int16 LE, interleaved, RX_CHANNELS Kanäle),
       füttert raw_queue und aktualisiert den Plot-Ringpuffer."""
    global queued_frames, running

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 1 << 20)
    except Exception:
        pass
    sock.bind(("", PORT))
    print(f"[UDP] Listening on :{PORT} | expecting {RX_CHANNELS}ch int16 LE @ {SR} Hz")

    recv_bytes = 0
    last_t = time.time()

    while running:
        data, _ = sock.recvfrom(65536)
        if not data:
            continue
        if len(data) % BYTES_PER_FRAME != 0:
            # Paket nicht frame-aligned -> verwerfen
            continue

        frames = len(data) // BYTES_PER_FRAME
        blockN = np.frombuffer(data, dtype="<i2").reshape(frames, RX_CHANNELS).copy()

        with lock:
            raw_queue.append(blockN)
            queued_frames += frames

            # Plot-Ringpuffer aktualisieren (alle Kanäle)
            n = len(blockN)
            if n >= plot_len:
                plot_buf[:] = blockN[-plot_len:]
            else:
                plot_buf[:-n] = plot_buf[n:]
                plot_buf[-n:] = blockN

        # Durchsatz-Log 1x/s
        recv_bytes += len(data)
        now = time.time()
        if now - last_t >= 1.0:
            exp = SR * BYTES_PER_FRAME
            pct = (recv_bytes / exp * 100.0) if exp > 0 else 0.0
            print(f"In: {recv_bytes:7d} B/s  ({pct:5.1f}% of expected)")
            recv_bytes = 0
            last_t = now

# ---------- Audio Writer ----------
def audio_writer():
    """Callback-getriebene Audioausgabe: Gerät zieht exakt SR-Frames; wir liefern aus raw_queue."""
    global queued_frames, running, pickL, pickR, play_mode

    # Lokaler FIFO-Puffer für 2ch (int16), damit wir im Callback ohne Locks arbeiten können
    fifo = np.zeros((0, 2), dtype=np.int16)
    fifo_lock = threading.Lock()

    # Feeder-Thread: holt Nch-Blöcke aus raw_queue, mappt auf 2ch (stereo/mono/mute) und hängt an fifo an
    def feeder():
        nonlocal fifo
        global queued_frames
        while running:
            with lock:
                blk = raw_queue.popleft() if raw_queue else None
                mode = play_mode
                l_idx, r_idx = pickL, pickR
            if blk is None:
                time.sleep(0.001)
                continue

            if mode == "mute":
                out = np.zeros((len(blk), 2), dtype=np.int16)
            elif mode == "mono":
                mix = (blk[:, l_idx].astype(np.int32) + blk[:, r_idx].astype(np.int32)) // 2
                out = np.stack([mix, mix], axis=1).astype(np.int16)
            else:  # stereo
                out = blk[:, [l_idx, r_idx]]

            with fifo_lock:
                fifo = np.vstack((fifo, out))
            with lock:
                queued_frames -= len(blk)  # jetzt korrekt: wir zählen ab, wenn in den 2ch-FIFO überführt

    # Audio-Callback: füllt das Devicepuffer exakt mit SR-Frames
    def callback(outdata, frames, time_info, status):
        nonlocal fifo
        if status:
            # z.B. underflow/overflow Hinweise
            # print(status)
            pass
        with fifo_lock:
            have = len(fifo)
            if have >= frames:
                out = fifo[:frames]
                fifo = fifo[frames:]
            else:
                # nicht genug -> Rest mit Stille auffüllen (verhindert Knackser)
                out = np.zeros((frames, 2), dtype=np.int16)
                if have > 0:
                    out[:have] = fifo
                    fifo = fifo[have:]
        outdata[:] = out

    stream_kwargs = dict(samplerate=SR, channels=2, dtype="int16", blocksize=0, callback=callback)
    if AUDIO_DEVICE is not None:
        stream_kwargs["device"] = AUDIO_DEVICE

    feeder_thread = threading.Thread(target=feeder, daemon=True)
    feeder_thread.start()

    import sounddevice as sd
    with sd.OutputStream(**stream_kwargs) as stream:
        print("Actual device SR:", stream.samplerate)
        while running:
            time.sleep(0.1)  # Hauptthread am Leben halten

# ---------- Plot ----------
def start_plot():
    """Startet 6-Kanal-Plot (3×2) + Keyboard-Shortcuts zum Umschalten der Playback-Kanäle/Modi."""
    global _ani_ref, pickL, pickR, play_mode

    # 3×2 Layout (6 Kanäle)
    fig, axes = plt.subplots(3, 2, sharex=True, figsize=(11, 6))
    axes = axes.flatten()
    x = np.arange(plot_len)
    lines = []
    for ch in range(RX_CHANNELS):
        line, = axes[ch].plot(x, plot_buf[:, ch])
        axes[ch].set_ylim([-32768, 32767])
        axes[ch].set_title(f"Ch {ch}")
        lines.append(line)
    for ax in axes[-2:]:
        ax.set_xlabel("Samples")
    fig.tight_layout()

    def update(_):
        with lock:
            for ch in range(RX_CHANNELS):
                lines[ch].set_ydata(plot_buf[:, ch])
            qf = queued_frames
            l_sel, r_sel, mode = pickL, pickR, play_mode
        fig.suptitle(f"Queue ~{qf / SR:.3f}s | Play: {mode} | L={l_sel} R={r_sel} | {SR} Hz, {RX_CHANNELS}ch", fontsize=10)
        return lines

    def clamp(idx):
        return (idx + RX_CHANNELS) % RX_CHANNELS

    def on_key(ev):
        global pickL, pickR, play_mode
        if ev.key == '[':
            with lock:
                pickL = clamp(pickL - 1)
        elif ev.key == ']':
            with lock:
                pickL = clamp(pickL + 1)
        elif ev.key == ';':
            with lock:
                pickR = clamp(pickR - 1)
        elif ev.key == "'":
            with lock:
                pickR = clamp(pickR + 1)
        elif ev.key == 'tab':
            with lock:
                pickL, pickR = pickR, pickL
        elif ev.key == 'm':
            with lock:
                play_mode = "mono"
        elif ev.key == 'p':
            with lock:
                play_mode = "stereo"
        elif ev.key == '0':
            with lock:
                play_mode = "mute"

    fig.canvas.mpl_connect('key_press_event', on_key)
    _ani_ref = FuncAnimation(fig, update, interval=33, blit=False, cache_frame_data=False)
    plt.show()

# ---------- Main ----------
def main():
    t_rx = threading.Thread(target=udp_receiver, daemon=True)
    t_rx.start()

    # Prefill
    target = int(PREFILL_SEC * SR)
    print(f"Prefill ~{PREFILL_SEC:.2f}s …")
    try:
        while True:
            time.sleep(0.01)
            with lock:
                if queued_frames >= target:
                    break
    except KeyboardInterrupt:
        return

    # Audio-Thread
    t_audio = threading.Thread(target=audio_writer, daemon=True)
    t_audio.start()

    # Plot (blockiert bis Fenster zu)
    try:
        start_plot()
    finally:
        global running
        running = False
        time.sleep(0.1)

if __name__ == "__main__":
    main()
