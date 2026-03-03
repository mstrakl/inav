import socket
import time
import math
import argparse
import threading


HOST = "127.0.0.1"
PORT = 2323
RATE_HZ = 200.0

_tx_state = {
    "enable": False,
    "swA": 0,
    "swB": 0,
    "swC": 0,
    "swD": 0,
    "potS1": 0.0,
    "potS2": 0.0,
}
_tx_lock = threading.Lock()


def main(args):

    inav_connect = False
    
    if args.sim == "inav":
        inav_connect = True
        
    if args.joy:

        t = threading.Thread(
            target=read_tx_thread,
            daemon=True
        )
        t.start()


    if inav_connect:
        server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        
        server.bind((HOST, PORT))
        server.listen(1)
        print(f"Listening on {HOST}:{PORT}")

        conn, addr = server.accept()
        print(f"Client connected from {addr}")
    
        

    from jsb_sim import JsbSimulation
    Inav = JsbSimulation(1.0/RATE_HZ)
    
    start_time = time.time()
    t = 0
    try:
        while t < 500:
            
            t = time.time() - start_time  # relative time

            if args.joy:
                tx = get_tx_state()
                
                if (t < 2.0 and tx["swD"] != 0):
                    while tx["swD"] != 0:
                        print("Waiting for disarm...")
                        time.sleep(0.5)
                        tx = get_tx_state()
                
                #print(
                #    f"swA={tx['swA']} "
                #    f"swB={tx['swB']} "
                #    f"swC={tx['swC']} "
                #    f"swD={tx['swD']} "
                #    f"potS1={tx['potS1']:.2f} "
                #    f"potS2={tx['potS2']:.2f}"
                #)
            else:
                tx = _tx_state


            if inav_connect:
                Inav.rx(conn)
            
            Inav.update(tx)
            
            if inav_connect:
                Inav.tx(conn)
            
            time.sleep(Inav.getDt())
            
    except KeyboardInterrupt:
        print("\nCtrl+C pressed! Exiting gracefully...")
    
    if inav_connect:
        server.close()




def read_tx_thread(device="/dev/input/js0"):
    import sys
    if sys.platform == "darwin":
        _read_tx_thread_pygame()
    else:
        _read_tx_thread_linux(device)


def _read_tx_thread_linux(device="/dev/input/js0"):
    import struct

    JS_EVENT_FORMAT = "IhBB"
    JS_EVENT_SIZE = struct.calcsize(JS_EVENT_FORMAT)

    try:
        open(device, "rb").close()
        _tx_state["enable"] = True
    except FileNotFoundError:
        print(f"Joystick device {device} not found. Joystick input will be disabled.")
        return

    with open(device, "rb") as js:
        while True:
            event = js.read(JS_EVENT_SIZE)
            if not event:
                break

            _, value, event_type, number = struct.unpack(JS_EVENT_FORMAT, event)

            with _tx_lock:
                if event_type & 0x01:   # button
                    if number == 3:
                        _tx_state["swA"] = value
                elif event_type & 0x02:  # axis
                    if number == 1:
                        _tx_state["potS1"] = value / 32767.0
                    elif number == 3:
                        _tx_state["potS2"] = value / 32767.0
                    elif number == 4:
                        _tx_state["swD"] = read_sw(value)
                    elif number == 5:
                        _tx_state["swC"] = read_sw(value)
                    elif number == 6:
                        _tx_state["swB"] = read_sw(value)


def _read_tx_thread_pygame(joystick_index=0):
    import os
    import pygame

    # Prevent SDL from touching Cocoa/AppKit (creating a window or menu bar).
    # On macOS those calls must happen on the main thread; we run on a daemon
    # thread, so using the dummy video/audio driver avoids the crash entirely.
    # Joystick input does not need a display.
    os.environ.setdefault("SDL_VIDEODRIVER", "dummy")
    os.environ.setdefault("SDL_AUDIODRIVER", "dummy")

    pygame.init()
    pygame.joystick.init()

    if pygame.joystick.get_count() == 0:
        print("No joystick found by pygame. Joystick input will be disabled.")
        return

    joy = pygame.joystick.Joystick(joystick_index)
    joy.init()
    print(f"pygame joystick: {joy.get_name()}  "
          f"({joy.get_numaxes()} axes, {joy.get_numbuttons()} buttons)")

    _tx_state["enable"] = True

    # Axis/button indices match the Linux /dev/input mapping for the TX16S:
    #   axis 1  -> potS1   (right vertical, normalised -1..1)
    #   axis 3  -> potS2   (right horizontal, normalised -1..1)
    #   axis 4  -> swD     (3-pos switch, -1 / 0 / +1)
    #   axis 5  -> swC
    #   axis 6  -> swB
    #   button 3 -> swA    (momentary, 0/1)
    POLL_HZ = 100
    import time
    while True:
        pygame.event.pump()
        with _tx_lock:
            _tx_state["swA"]   = joy.get_button(3)
            _tx_state["potS1"] = joy.get_axis(1)
            _tx_state["potS2"] = joy.get_axis(3)
            # get_axis returns -1..1; scale to -32767..32767 for read_sw
            _tx_state["swD"]  = read_sw(int(joy.get_axis(4) * 32767))
            _tx_state["swC"]  = read_sw(int(joy.get_axis(5) * 32767))
            _tx_state["swB"]  = read_sw(int(joy.get_axis(6) * 32767))

            _tx_state["swF"]  = joy.get_button(2)
            _tx_state["swG"]  = read_sw(int(joy.get_axis(7) * 32767))


            _tx_state["axAil"] = joy.get_axis(0)
            _tx_state["axEle"] = joy.get_axis(1)
            _tx_state["axThr"] = joy.get_axis(2)
            _tx_state["axRud"] = joy.get_axis(3)

        if False:
            # Print all raw axes and buttons, then the mapped TX state
            axes_str    = "  ".join(f"a{i}={joy.get_axis(i):+.3f}"    for i in range(joy.get_numaxes()))
            buttons_str = "  ".join(f"b{i}={joy.get_button(i)}"       for i in range(joy.get_numbuttons()))
            print(f"AXES:    {axes_str}")
            print(f"BUTTONS: {buttons_str}")
            with _tx_lock:
                s = _tx_state
                print(f"MAPPED:  swA={s['swA']}  swB={s['swB']}  swC={s['swC']}  swD={s['swD']}"
                    f"  swF={s['swF']}  swG={s['swG']}")
            print()


        time.sleep(1.0 / POLL_HZ)




def get_tx_state():
    with _tx_lock:
        return _tx_state.copy()
    
def read_sw(value):
    
    if value < -10000:
        return 0
    elif value < 10000:
        return 1
    else:
        return 2


if __name__ == "__main__":
    
    parser = argparse.ArgumentParser(description="Simulator entry point")
    parser.add_argument(
        "--sim",
        type=str,
        default=None,
        choices=["inav"],
        help="Select simulator backend (e.g. inav)"
    )
    

    parser.add_argument(
        "--joy",
        action="store_true",
        help="Use joystick"
    )
    
    args = parser.parse_args()
    
    main(args)