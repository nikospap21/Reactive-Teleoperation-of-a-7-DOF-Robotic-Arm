import socket
import json

UDP_IP = "0.0.0.0"
UDP_PORT = 5005

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
sock.bind((UDP_IP, UDP_PORT))

print(f"WSL Receiver έτοιμος! Αναμονή στην πόρτα {UDP_PORT}...")

try:
    while True:
        data, _ = sock.recvfrom(2048)
        state = json.loads(data.decode('utf-8'))
        
        # Εκτύπωση αξόνων και πατημένων κουμπιών στην ίδια γραμμή
        axes_str = " | ".join([f"A{i}: {v:+0.2f}" for i, v in enumerate(state["axes"])])
        active_buttons = [i for i, v in enumerate(state["buttons"]) if v == 1]
        
        print(f"\r{axes_str} | Buttons: {active_buttons}   ", end="", flush=True)

except KeyboardInterrupt:
    print("\nΈξοδος.")
finally:
    sock.close()