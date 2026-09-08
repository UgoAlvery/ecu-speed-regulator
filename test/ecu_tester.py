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
        self.tx_nonce = 0          # anti-rejeu sortant, incrémenté à chaque trame émise
        self.last_rx_nonce = None  # anti-rejeu entrant, pour vérifier les trames de l'ECU
        self.replay_dropped = 0    # trames locales rejetées côté script (nonce ECU non croissant)
        self.last_speed_frame = None  # dernière trame SPEED envoyée, pour le test de rejeu

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

    def build_frame(self, msg_type, payload=b'', nonce=None):
        """Construit une trame: [0xAA][LEN(2)][TYPE][NONCE(2 LE)][PAYLOAD][CRC16(2 LE)]
        sans l'envoyer — utile pour rejouer volontairement un nonce donné (test anti-rejeu)."""
        if nonce is None:
            self.tx_nonce = (self.tx_nonce + 1) & 0xFFFF
            nonce = self.tx_nonce
        length = len(payload) + 1 + 2  # LEN: Taille de (TYPE + NONCE + PAYLOAD)
        header = struct.pack('<HBH', length, msg_type, nonce)
        full_msg = header + payload
        crc = self.compute_crc(full_msg)
        return struct.pack('B', 0xAA) + full_msg + struct.pack('<H', crc)

    def send_frame(self, msg_type, payload=b'', nonce=None):
        """Construit et envoie une trame (cf. build_frame)."""
        frame = self.build_frame(msg_type, payload, nonce)
        self.ser.write(frame)
        return frame

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

    def is_nonce_newer(self, nonce):
        """Même règle de comparaison modulaire que protocol_nonce_check_and_update()
        côté firmware : accepte ssi nonce est postérieur au dernier accepté, au
        sens circulaire (tolère le wraparound 16 bits)."""
        if self.last_rx_nonce is None:
            return True
        delta = (nonce - self.last_rx_nonce) & 0xFFFF
        if delta >= 0x8000:
            delta -= 0x10000
        return delta > 0

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
                            nonce = struct.unpack('<H', data[1:3])[0]
                            payload = data[3:]

                            if not self.is_nonce_newer(nonce):
                                self.replay_dropped += 1
                                self.log(f"[ANTI-REJEU] trame ECU ignorée "
                                         f"(nonce={nonce} <= dernier={self.last_rx_nonce})")
                                continue
                            self.last_rx_nonce = nonce

                            if msg_type == MSG_OUTPUT:
                                self.last_output = struct.unpack('<f', payload)[0]
                                self.log(f"[SPEED] ({self.current_speed:.2f} km/h) | Commande: {self.last_output:.2f}")

                            elif msg_type == MSG_STATS:
                                self.stats_count += 1
                                if len(payload) >= 24:
                                    rx_valid, rx_crc_err, rx_dropped, rx_replay, tx_output, uptime_s = \
                                        struct.unpack('<6I', payload[:24])
                                    self.log(f"[TELEMETRIE] ({self.stats_count}s) "
                                             f"valid={rx_valid} crc_err={rx_crc_err} "
                                             f"dropped={rx_dropped} replay={rx_replay} "
                                             f"tx_output={tx_output} uptime={uptime_s}s")
                                else:
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
            # (frame gardée pour le test de rejeu de la phase de stress)
            self.last_speed_frame = self.send_frame(MSG_SPEED, struct.pack('<f', self.current_speed))
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
        # LEN=7 : TYPE(1) + NONCE(2) + PAYLOAD(4 B, float)
        partial_frame = struct.pack('B', 0xAA) + struct.pack('<HB', 7, MSG_SPEED)
        self.ser.write(partial_frame)
        time.sleep(0.5)  # Pause au milieu du message → doit déclencher le timeout parser (50ms)
        # Reste de la trame envoyé après coup ; CRC volontairement omis, l'objectif
        # est de vérifier que le parser s'est déjà reset sur timeout, pas de
        # produire une trame valide.
        self.ser.write(struct.pack('<H', 0) + struct.pack('<f', 100.0))

        # 3. Erreur de CRC16
        self.log("Action: Envoi erreur CRC...")
        bad_crc_frame = bytearray(self.build_frame(MSG_SPEED, struct.pack('<f', 0.0)))
        bad_crc_frame[-2:] = b'\xFF\xFF'  # CRC intentionnellement invalide
        self.ser.write(bytes(bad_crc_frame))

        # 4. Rejeu (replay) d'une trame déjà acceptée précédemment
        self.log("Action: Rejeu d'une trame SPEED déjà envoyée (test anti-rejeu)...")
        if self.last_speed_frame is not None:
            self.ser.write(self.last_speed_frame)
            self.log("  -> attendu : rejet silencieux côté ECU, compteur "
                     "'replay' incrémenté dans la prochaine trame STATS")

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
