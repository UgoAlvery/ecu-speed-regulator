import os
import serial
import struct
import time
import random
import threading
from datetime import datetime

# --- CONFIGURATION ---
SERIAL_PORT = '/dev/ttyUSB0'  # À adapter
#SERIAL_PORT = '/tmp/ttyV0'
BAUD_RATE = 115200
TIMEOUT = 0.1
SPEED_TX_PERIOD_S = 0.1
MOTOR_GAIN = 1.65
DRAG_COEFF = 0.9
MODEL_RESPONSE = 1.2
MAX_SPEED = 220.0

# Codes des messages
MSG_SETPOINT = 0x01
MSG_SPEED    = 0x02
MSG_MODE_SET = 0x05
MSG_OUTPUT   = 0x80
MSG_STATS    = 0x83
MSG_ALARM    = 0x85

class ECUTester:
    def __init__(self):
        self.ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=TIMEOUT)
        self.current_speed = 50.0  # Vitesse initiale
        self.target_setpoint = 90.0
        self.last_output = 0.0
        self.running = True
        self.stats_count = 0
        self.start_time = time.time()

    def log(self, message):
        timestamp = datetime.now().strftime('%H:%M:%S.%f')[:-3]
        print(f"[{timestamp}] {message}")

    def update_vehicle_model(self, dt):
        """Modèle 1er ordre: accel = moteur - traînée."""
        dt = max(0.0, min(dt, 0.2))
        acceleration = MODEL_RESPONSE * ((MOTOR_GAIN * self.last_output) - (DRAG_COEFF * self.current_speed))
        self.current_speed += acceleration * dt
        self.current_speed = max(0.0, min(MAX_SPEED, self.current_speed))

    def compute_crc(self, data_bytes):
        """CRC-16/CCITT (poly 0x1021, init 0x0000) sur les octets hors START."""
        crc = 0x0000
        for b in data_bytes:
            crc ^= b << 8
            for _ in range(8):
                crc = (crc << 1) ^ 0x1021 if crc & 0x8000 else crc << 1
            crc &= 0xFFFF
        return crc

    def send_frame(self, msg_type, payload=b''):
        """Construit et envoie une trame: [0xAA][LEN(2)][TYPE][PAYLOAD][CRC16(2 LE)]"""
        length = len(payload) + 1  # LEN: Taille de (TYPE + PAYLOAD)
        header = struct.pack('<HB', length, msg_type)
        full_msg = header + payload
        crc = self.compute_crc(full_msg)
        frame = struct.pack('B', 0xAA) + full_msg + struct.pack('<H', crc)
        self.ser.write(frame)

    def handle_non_protocol_byte(self, first_byte):
        """Traite les octets hors protocole binaire (logs ESP texte, bruit)."""
        if not first_byte:
            return

        b = first_byte[0]
        if b in (0x0A, 0x0D):
            return

        # Les logs ESP (ESP_LOGI/W/E) arrivent en ASCII sur UART0.
        if 32 <= b <= 126:
            tail = self.ser.read_until(b'\n', 256)
            line = (first_byte + tail).decode('utf-8', errors='ignore').strip()
            if line:
                self.log(f"\033[35m[ESP] {line}\033[0m")
            return

        self.log(f"Byte inattendu: {first_byte} - resync en cours...")

    def receive_feedback(self):
        """Lit et décode les messages venant de l'ECU (Output et Télémétrie)"""
        while self.running:
            if self.ser.in_waiting > 0:
                start_byte = self.ser.read(1)
                if start_byte == b'\xaa':
                    len_bytes = self.ser.read(2)
                    if len(len_bytes) < 2: continue
                    length = struct.unpack('<H', len_bytes)[0]

                    data = self.ser.read(length)
                    crc_received = self.ser.read(2)

                    if len(data) == length and len(crc_received) == 2:
                        # Vérification CRC-16 little-endian
                        if self.compute_crc(len_bytes + data) == struct.unpack('<H', crc_received)[0]:
                            msg_type = data[0]
                            payload = data[1:]

                            if msg_type == MSG_OUTPUT:
                                self.last_output = struct.unpack('<f', payload)[0]
                                self.log(f"[SPEED] ({self.current_speed:.2f} km/h) | Commande: {self.last_output:.2f}")

                            elif msg_type == MSG_STATS:
                                self.stats_count += 1
                                self.log(f"[TELEMETRIE] Reçue ({self.stats_count}s)")

                            elif msg_type == MSG_ALARM:
                                self.log(f"\n[ALERTE ECU] : {payload.decode(errors='ignore')}")
                else:
                    self.handle_non_protocol_byte(start_byte)

    def run_normal_operation(self, duration):
        """Phase de fonctionnement normal (100ms cycle) """
        self.log(f"--- DÉBUT PHASE NORMALE ({duration}s) ---")
        self.send_frame(MSG_SETPOINT, struct.pack('<f', self.target_setpoint))
        self.send_frame(MSG_MODE_SET, struct.pack('B', 2)) # Mode AUTO

        end_time = time.time() + duration
        last_tick = time.time()
        while time.time() < end_time:
            now = time.time()
            dt = now - last_tick
            last_tick = now

            self.update_vehicle_model(dt)
            # Envoi de la vitesse mesurée toutes les 100ms
            self.send_frame(MSG_SPEED, struct.pack('<f', self.current_speed))
            time.sleep(SPEED_TX_PERIOD_S)

    def run_stress_test(self):
        """Phase de stress : messages corrompus, fragmentés et flood"""
        self.log("\n--- DÉBUT PHASE DE STRESS ---")

        # 1. Flood de messages (Surcharge CPU)
        self.log("Action: Surcharge CPU (Flood)...")
        for _ in range(50):
            self.send_frame(random.randint(0, 0xFF), os.urandom(4))

        # 2. Trames fragmentées
        self.log("Action: Envoi de trame fragmentée...")
        partial_frame = struct.pack('B', 0xAA) + struct.pack('<HB', 5, MSG_SPEED)
        self.ser.write(partial_frame)
        time.sleep(0.5) # Pause au milieu du message
        self.ser.write(struct.pack('<f', 100.0) + b'\x00') # Fin de trame

        # 3. Erreur de CRC16
        self.log("Action: Envoi erreur CRC...")
        # MSG_SPEED (LEN=5, float 0.0) avec CRC16 intentionnellement invalide
        self.ser.write(b'\xAA\x05\x00\x02\x00\x00\x00\x00\xFF\xFF')

    def test_failsafe(self):
        """Vérifie si l'ECU passe en Failsafe après 2s de silence"""
        self.log("\n--- TEST FAILSAFE (Silence radio 2s) ---")
        time.sleep(2.5)
        if abs(self.last_output) < 1e-3:
            self.log("VÉRIFICATION RÉUSSIE : L'ECU a coupé la commande moteur (Output=0)")
        else:
            self.log("ÉCHEC : L'ECU n'est pas passé en mode Failsafe")

    def start(self):
        # Lancement du thread de lecture
        reader = threading.Thread(target=self.receive_feedback, daemon=True)
        reader.start()

        try:
            self.run_normal_operation(30)  # 30s de fonctionnement normal
            self.run_stress_test()
            self.run_normal_operation(5)
            self.test_failsafe()
        except KeyboardInterrupt:
            pass
        finally:
            self.running = False
            self.ser.close()
            self.log("\nTest terminé.")

if __name__ == "__main__":
    tester = ECUTester()
    tester.start()
