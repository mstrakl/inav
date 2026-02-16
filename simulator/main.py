import socket
import time
import math
import argparse
import threading

from src.inav_interface import InavSimulate

HOST = "127.0.0.1"
PORT = 2323
RATE_HZ = 60.0

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
    
        
    Inav = InavSimulate(run_isolated=(not inav_connect))
     
    dt = 1.0 / RATE_HZ
    start_time = time.time()

    try:
        while True:
            
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
            
            Inav.update(t, tx)
            
            if inav_connect:
                Inav.tx(conn)

            time.sleep(dt)
    except KeyboardInterrupt:
        print("\nCtrl+C pressed! Exiting gracefully...")
    
    if inav_connect:
        server.close()




def read_tx_thread(device="/dev/input/js0"):
    import struct

    JS_EVENT_FORMAT = "IhBB"
    JS_EVENT_SIZE = struct.calcsize(JS_EVENT_FORMAT)
    
    try:
        js = open(device, "rb")
        _tx_state["enable"] = True
    except FileNotFoundError:
        print(f"Joystick device {device} not found. Joystick input will be disabled.")
        return

    with open(device, "rb") as js:
        while True:
            event = js.read(JS_EVENT_SIZE)
            if not event:
                break

            _, value, event_type, number = struct.unpack(
                JS_EVENT_FORMAT, event
            )

            with _tx_lock:
                if event_type & 0x01:  # button
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