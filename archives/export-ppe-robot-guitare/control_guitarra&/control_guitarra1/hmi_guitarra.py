import curses
import serial
import threading
import time
import re
import math

# Notas y frecuencias base (A4 = 440 Hz)
NOTE_NAMES = ['C', 'C#', 'D', 'D#', 'E', 'F', 'F#', 'G', 'G#', 'A', 'A#', 'B']
A4_INDEX = NOTE_NAMES.index('A') + 12 * 4  # MIDI index for A4

# Calcular frecuencia de cualquier nota/octava
def note_freq(note, octave):
    n = NOTE_NAMES.index(note) + 12 * (octave)
    return 440.0 * (2 ** ((n - A4_INDEX) / 12))

def get_next_frets(note, octave, num_frets=12):
    idx = NOTE_NAMES.index(note) + 12 * octave
    frets = []
    for i in range(num_frets+1):
        nidx = idx + i
        nname = NOTE_NAMES[nidx % 12]
        noct = nidx // 12
        frets.append(f"{i}: {nname}{noct}")
    return frets

# Diccionario de errores y estados
ERROR_CODES = {
    'X1': 'Valeur invalide dans la séquence',
    'X2': 'Frette hors plage',
    'X3': 'Format BPM invalide',
    'X4': 'Valeurs BPM invalides',
    'X5': 'Format de frette invalide',
    'X6': 'Séquence de frettes pleine',
    'X7': 'Combinaison de frettes invalide',
    'X8': 'Homing non effectué',
    'X9': 'Aucune chanson chargée',
    'X10': 'Aucun BPM',
    'X11': 'Le chargement de la chanson nécessite un homing',
    'X12': 'Numéro de chanson invalide',
}
STATE_CODES = {
    'EN': 'Moteur activé',
    'ED': 'Moteur désactivé',
    'N': 'Note jouée',
    'A': 'Mode accordage activé',
    'AR': 'Mode accordage désactivé',
    'G': 'Lecture de la chanson commencée',
    'GR': 'Lecture de la chanson terminée',
    'C': 'Chanson chargée',
    'T': 'Frette ajoutée',
}

SONG_TITLES = [
    "Joyeux Anniversaire (exemple)",
    "Smoke on the Water (exemple)",
    "Come As You Are (exemple)",
    "Chanson Test (exemple)"
]

# Información de afinación para cada canción (nota, octava)
SONG_TUNING = [
    ("E", 4),  # Joyeux Anniversaire
    ("E", 4),  # Smoke on the Water
    ("E", 4),  # Come As You Are
    ("E", 4)   # Chanson Test
]

SELECTED_SONG_IDX = 0
SELECTED_SONG_NAME = SONG_TITLES[0]

class SerialReader(threading.Thread):
    def __init__(self, ser):
        super().__init__()
        self.ser = ser
        self.freq = None
        self.last_code = None
        self.running = True
        self.lock = threading.Lock()
    def run(self):
        freq_pattern = re.compile(r"^\s*:F([0-9.]+)")
        code_pattern = re.compile(r":([A-Z]+[0-9]*)")
        while self.running:
            try:
                line = self.ser.readline().decode(errors='ignore').strip()
                m = freq_pattern.search(line)
                if m:
                    with self.lock:
                        self.freq = float(m.group(1))
                else:
                    m2 = code_pattern.search(line)
                    if m2:
                        with self.lock:
                            self.last_code = m2.group(1)
            except Exception:
                pass
    def get_freq(self):
        with self.lock:
            return self.freq
    def get_code(self):
        with self.lock:
            code = self.last_code
            self.last_code = None
            return code
    def stop(self):
        self.running = False

def safe_addstr(stdscr, y, x, string, *args):
    h, w = stdscr.getmaxyx()
    if 0 <= y < h and 0 <= x < w:
        # Coupe la chaîne si elle dépasse la largeur
        max_len = w - x
        stdscr.addstr(y, x, string[:max_len], *args)

def draw_menu(stdscr, selected, options, title, status_msg=None):
    stdscr.clear()
    h, w = stdscr.getmaxyx()
    safe_addstr(stdscr, 1, w//2 - len(title)//2, title, curses.A_BOLD)
    for idx, opt in enumerate(options):
        x = w//2 - len(opt)//2
        y = 3 + idx
        if idx == selected:
            stdscr.attron(curses.color_pair(1))
            safe_addstr(stdscr, y, x, opt)
            stdscr.attroff(curses.color_pair(1))
        else:
            safe_addstr(stdscr, y, x, opt)
    if status_msg:
        safe_addstr(stdscr, h-3, 2, status_msg, curses.A_BOLD)
    stdscr.refresh()

def draw_tuning(stdscr, note_idx, octave, freq_obj, ser, status_msg=None):
    stdscr.clear()
    h, w = stdscr.getmaxyx()
    note = NOTE_NAMES[note_idx]
    freq_target = note_freq(note, octave)
    safe_addstr(stdscr, 1, w//2 - 12, "Accordage de la corde", curses.A_BOLD)
    safe_addstr(stdscr, 3, 4, f"Note : [{note}]   Octave : [{octave}]   Fréq. cible : {freq_target:.2f} Hz")
    freq = freq_obj.get_freq()
    bar_y = 5
    bar_margin = 8
    bar_len = w - 2*bar_margin - 2
    if bar_len < 21:
        bar_len = 21
    center = bar_len // 2
    if freq is not None:
        diff = freq - freq_target
        visual_range = 200.0  # ±200 Hz
        norm = max(-1.0, min(1.0, diff / visual_range))
        pos = int(center + norm * (bar_len / 2))
        pos = max(0, min(bar_len-1, pos))
        bar = ['-'] * bar_len
        bar[center] = '+'
        bar[pos] = '|'
        bar_str = ''.join(bar)
        safe_addstr(stdscr, bar_y, bar_margin, bar_str)
        if abs(diff) <= 5:
            action = "Accordé !"
            color = curses.color_pair(2)
        elif diff < 0:
            action = "Tendre la corde"
            color = curses.color_pair(3)
        else:
            action = "Détendre la corde"
            color = curses.color_pair(4)
        freq_str = f"{freq:.2f} Hz"
        safe_addstr(stdscr, bar_y+1, w//2 - len(freq_str)//2, freq_str, color)
        safe_addstr(stdscr, bar_y+2, w//2 - len(action)//2, action, color)
    else:
        safe_addstr(stdscr, bar_y, bar_margin, "En attente de fréquence...")
    frets = get_next_frets(note, octave)
    safe_addstr(stdscr, bar_y+4, 4, "Frettes suivantes :")
    fret_line = ''
    row = bar_y+5
    for f in frets:
        fret_line += f + '   '
        if len(fret_line) > w-10:
            safe_addstr(stdscr, row, 4, fret_line)
            fret_line = ''
            row += 1
    if fret_line:
        if row < h:
            max_len = w - 8 if w > 8 else 0
            safe_addstr(stdscr, row, 4, fret_line[:max_len])
    safe_addstr(stdscr, h-2, 4, "← → note, ↑ ↓ octave, Q pour revenir")
    if status_msg:
        safe_addstr(stdscr, h-3, 4, status_msg, curses.A_BOLD)
    stdscr.refresh()

def draw_progress_bar(stdscr, progress, label="Homing en cours..."):
    h, w = stdscr.getmaxyx()
    bar_len = w - 20
    filled = int(bar_len * progress)
    bar = '[' + '='*filled + ' '*(bar_len-filled) + ']'
    stdscr.clear()
    safe_addstr(stdscr, h//2-1, w//2 - len(label)//2, label, curses.A_BOLD)
    safe_addstr(stdscr, h//2, w//2 - bar_len//2, bar)
    stdscr.refresh()

def send_serial_slow(ser, data, delay=0.01):
    """Envía una cadena por serial, byte a byte, con un pequeño delay entre cada byte."""
    if isinstance(data, str):
        data = data.encode()
    for b in data:
        ser.write(bytes([b]))
        ser.flush()
        time.sleep(delay)

def draw_tuning_info(stdscr, song_idx):
    """Affiche les informations d'accordage pour une chanson spécifique."""
    stdscr.clear()
    h, w = stdscr.getmaxyx()
    note, octave = SONG_TUNING[song_idx]
    freq = note_freq(note, octave)
    
    title = f"Informations d'accordage - {SONG_TITLES[song_idx]}"
    safe_addstr(stdscr, 1, w//2 - len(title)//2, title, curses.A_BOLD)
    
    tuning_info = [
        f"Note à vide : {note}{octave}",
        f"Fréquence cible : {freq:.2f} Hz",
        "",
        "Instructions :",
        "1. Sélectionnez la corde correcte",
        "2. Utilisez le mode accordage pour ajuster la fréquence",
        "3. Assurez-vous que la fréquence correspond à la cible",
        "",
        "Appuyez sur ENTRER pour continuer ou Q pour revenir"
    ]
    
    for idx, line in enumerate(tuning_info):
        y = 3 + idx
        x = w//2 - len(line)//2
        safe_addstr(stdscr, y, x, line)
    
    stdscr.refresh()

def draw_tuning_confirmation(stdscr):
    """Affiche l'écran de confirmation avant de jouer la chanson."""
    stdscr.clear()
    h, w = stdscr.getmaxyx()
    
    title = "Confirmation d'accordage"
    safe_addstr(stdscr, 1, w//2 - len(title)//2, title, curses.A_BOLD)
    
    note, octave = SONG_TUNING[SELECTED_SONG_IDX]
    freq = note_freq(note, octave)
    
    messages = [
        f"Chanson : {SELECTED_SONG_NAME}",
        f"Note à vide : {note}{octave}",
        f"Fréquence cible : {freq:.2f} Hz",
        "",
        "Êtes-vous sûr que la corde est correctement accordée ?",
        "",
        "Appuyez sur ENTRER pour continuer ou Q pour revenir"
    ]
    
    for idx, line in enumerate(messages):
        y = 3 + idx
        x = w//2 - len(line)//2
        safe_addstr(stdscr, y, x, line)
    
    stdscr.refresh()

def main(stdscr):
    curses.curs_set(0)
    curses.start_color()
    curses.init_pair(1, curses.COLOR_BLACK, curses.COLOR_CYAN)
    curses.init_pair(2, curses.COLOR_GREEN, curses.COLOR_BLACK)
    curses.init_pair(3, curses.COLOR_YELLOW, curses.COLOR_BLACK)
    curses.init_pair(4, curses.COLOR_RED, curses.COLOR_BLACK)
    # Activer le mode keypad pour que les touches spéciales (flèches, etc.)
    # soient retournées comme constantes curses.KEY_* par getch().
    stdscr.keypad(True)
    # Demander le port série
    stdscr.addstr(2, 2, "Entrez le port série (ex : COM9) :    ")
    curses.echo()
    port = stdscr.getstr(2, 36, 10).decode()
    curses.noecho()
    try:
        ser = serial.Serial(
            port=port,
            baudrate=115200,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            timeout=0.1
        )
        # Nettoyer les buffers série après ouverture
        ser.reset_input_buffer()
        ser.reset_output_buffer()
    except Exception as e:
        stdscr.addstr(4, 2, f"Erreur lors de l'ouverture du port : {e}")
        stdscr.getch()
        return
    freq_obj = SerialReader(ser)
    freq_obj.start()
    motor_enabled = False
    global SELECTED_SONG_IDX, SELECTED_SONG_NAME
    menu_base = ["Activer le moteur", "Accorder", "Homing", "Choisir une chanson", "Jouer la chanson", "Réinitialiser le microcontrôleur", "Quitter", "osciller le mediator", "jouer le mediator", "arreter le mediator"]
    selected = 0
    status_msg = None
    while True:
        code = freq_obj.get_code()
        if code:
            if code in ERROR_CODES:
                status_msg = f"Erreur : {ERROR_CODES[code]}"
            elif code in STATE_CODES:
                status_msg = f"État : {STATE_CODES[code]}"
                if code == 'EN':
                    motor_enabled = True
                # Si se deshabilita el motor, no hay código explícito, pero podrías agregar uno en el firmware
            elif code == 'HR':
                status_msg = "Homing terminé"
            else:
                status_msg = f"Code reçu : {code}"
        # Actualiza el menú según el estado del motor
        menu = menu_base.copy()
        if motor_enabled:
            menu[0] = "Désactiver le moteur"
        # Muestra el estado del motor en el menú
        etat_moteur = "Moteur : ACTIVÉ" if motor_enabled else "Moteur : DÉSACTIVÉ"
        titre = f"Guitare Robot  |  {etat_moteur}  |  Chanson : {SELECTED_SONG_NAME}"
        draw_menu(stdscr, selected, menu, titre, status_msg)
        key = stdscr.getch()
        # Gérer navigation: accepter Z/S (AZERTY) ET les flèches ↑/↓ via curses.KEY_*
        if key in (ord('z'), ord('Z'), curses.KEY_UP) and selected > 0:
            selected -= 1
        elif key in (ord('s'), ord('S'), curses.KEY_DOWN) and selected < len(menu)-1:
            selected += 1
        elif key in [curses.KEY_ENTER, 10, 13]:
            if menu[selected] == "Quitter":
                send_serial_slow(ser, ":R\r\n")
                time.sleep(0.5)
                ser.reset_input_buffer()
                ser.reset_output_buffer()
                break
            elif menu[selected] == "Réinitialiser le microcontrôleur":
                send_serial_slow(ser, ":R\r\n")
                ser.reset_input_buffer()
                ser.reset_output_buffer()
                motor_enabled = False
                SELECTED_SONG_IDX = 0
                SELECTED_SONG_NAME = SONG_TITLES[0]
                status_msg = "Commande de réinitialisation envoyée. Interface réinitialisée."
            elif menu[selected] == "Activer le moteur":
                send_serial_slow(ser, ":E1\r\n")
                # Esperar respuesta del micro
                timeout = time.time() + 2.0
                reponse = None
                while time.time() < timeout:
                    code = freq_obj.get_code()
                    if code:
                        if code == 'EN':
                            motor_enabled = True
                            status_msg = "Moteur activé."
                            reponse = 'ok'
                            break
                        elif code.startswith('X'):
                            # status_msg = f"Erreur : {ERROR_CODES.get(code, "Erreur l activation du moteur\")}"
                            reponse = 'error'
                            break
                    time.sleep(0.05)
                if reponse is None:
                    status_msg = "Pas de réponse du microcontrôleur lors de l'activation du moteur."
            elif menu[selected] == "Désactiver le moteur":
                send_serial_slow(ser, ":E0\r\n")
                # Esperar respuesta del micro (no hay código explícito, pero podrías agregar uno en el firmware)
                timeout = time.time() + 2.0
                reponse = None
                while time.time() < timeout:
                    code = freq_obj.get_code()
                    if code:
                        if code.startswith('X'):
                            status_msg = f"Erreur : {ERROR_CODES.get(code, 'Erreur lors de la désactivation du moteur')}"
                            reponse = 'error'
                            break
                    time.sleep(0.05)
                if reponse is None:
                    # Asumimos éxito si no hay error explícito
                    motor_enabled = False
                    status_msg = "Moteur désactivé."
            elif menu[selected] == "Accorder":
                # Utiliser la note et l'octave de la chanson sélectionnée
                note, octave = SONG_TUNING[SELECTED_SONG_IDX]
                note_idx = NOTE_NAMES.index(note)
                last_note_idx = None
                last_octave = None
                stdscr.timeout(50)  # Rafraîchit toutes les 50 ms
                msg_accord = None
                send_serial_slow(ser, ":A\r\n")  # Envoyer la commande pour démarrer l'envoi de fréquence
                while True:
                    code = freq_obj.get_code()
                    if code:
                        if code in ERROR_CODES:
                            msg_accord = f"Erreur : {ERROR_CODES[code]}"
                        elif code in STATE_CODES:
                            msg_accord = f"État : {STATE_CODES[code]}"
                        elif code == 'HR':
                            msg_accord = "Homing terminé"
                        else:
                            msg_accord = f"Code reçu : {code}"
                    draw_tuning(stdscr, note_idx, octave, freq_obj, ser, msg_accord)
                    k = stdscr.getch()
                    if k in (curses.KEY_LEFT, ord('a'), ord('A')):
                        note_idx = (note_idx - 1) % len(NOTE_NAMES)
                    elif k in (curses.KEY_RIGHT, ord('e'), ord('E')):
                        note_idx = (note_idx + 1) % len(NOTE_NAMES)
                    elif k in (curses.KEY_UP, ord('z'), ord('Z')) and octave < 6:
                        octave += 1
                    elif k in (curses.KEY_DOWN, ord('s'), ord('S')) and octave > 2:
                        octave -= 1
                    elif k in (ord('q'), ord('Q')):
                        send_serial_slow(ser, ":A\r\n")
                        stdscr.timeout(-1)  # Retour au mode attente normal
                        break
            elif menu[selected] == "Homing":
                if not motor_enabled:
                    status_msg = "Activez d'abord le moteur !"
                    continue
                send_serial_slow(ser, ":H\r\n")
                # Mostrar barra de progreso y esperar respuesta en pantalla de homing
                progress = 0.0
                start = time.time()
                stdscr.timeout(100)
                homing_done = False
                homing_error = None
                while not homing_done and not homing_error:
                    code = freq_obj.get_code()
                    if code:
                        if code == 'HR':
                            status_msg = "Homing terminé."
                            homing_done = True
                        elif code.startswith('X'):
                            status_msg = f"Erreur : {ERROR_CODES.get(code, 'Erreur lors du homing')}"
                            homing_error = True
                    elapsed = time.time() - start
                    progress = min(1.0, elapsed/10.0)  # Simula barra de hasta 10 segundos
                    draw_progress_bar(stdscr, progress, label="Homing en cours...")
                    k = stdscr.getch()
                    if k in [ord('q'), ord('Q')]:
                        break
                stdscr.timeout(-1)
            elif menu[selected] == "Choisir une chanson":
                song_menu_idx = 0
                while True:
                    stdscr.clear()
                    h, w = stdscr.getmaxyx()
                    safe_addstr(stdscr, 1, w//2 - 10, "Choisir une chanson", curses.A_BOLD)
                    for idx, title in enumerate(SONG_TITLES):
                        prefix = f"{idx} : "
                        x = w//2 - len(prefix + title)//2
                        y = 3 + idx
                        if idx == song_menu_idx:
                            stdscr.attron(curses.color_pair(1))
                            safe_addstr(stdscr, y, x, prefix + title)
                            stdscr.attroff(curses.color_pair(1))
                        else:
                            safe_addstr(stdscr, y, x, prefix + title)
                    safe_addstr(stdscr, h-2, 4, "ENTRER pour choisir, Q pour revenir")
                    stdscr.refresh()
                    k = stdscr.getch()
                    if k in (curses.KEY_UP, ord('z'), ord('Z')) and song_menu_idx > 0:
                        song_menu_idx -= 1
                    elif k in (curses.KEY_DOWN, ord('s'), ord('S')) and song_menu_idx < len(SONG_TITLES)-1:
                        song_menu_idx += 1
                    elif k in [curses.KEY_ENTER, 10, 13]:
                        send_serial_slow(ser, f":C{song_menu_idx}\r\n")
                        # Esperar respuesta del micro
                        timeout = time.time() + 2.0  # 2 segundos de timeout
                        reponse = None
                        while time.time() < timeout:
                            code = freq_obj.get_code()
                            if code:
                                if code == 'C':
                                    SELECTED_SONG_IDX = song_menu_idx
                                    SELECTED_SONG_NAME = SONG_TITLES[song_menu_idx]
                                    status_msg = f"Chanson sélectionnée : {SELECTED_SONG_NAME}"
                                    reponse = 'ok'
                                    break
                                elif code.startswith('X'):
                                    status_msg = f"Erreur : {ERROR_CODES.get(code, 'Erreur lors du choix de la chanson')}"
                                    reponse = 'error'
                                    break
                            time.sleep(0.05)
                        if reponse is None:
                            status_msg = "Pas de réponse du microcontrôleur lors du choix de la chanson."
                        if reponse == 'ok':
                            # Mostrar información de afinación
                            draw_tuning_info(stdscr, song_menu_idx)
                            while True:
                                k = stdscr.getch()
                                if k in [curses.KEY_ENTER, 10, 13]:
                                    break
                                elif k in [ord('q'), ord('Q')]:
                                    break
                        break
                    elif k in [ord('q'), ord('Q')]:
                        break
            elif menu[selected] == "Jouer la chanson":
                if not motor_enabled:
                    status_msg = "Activez d'abord le moteur !"
                    continue
                # Mostrar pantalla de confirmación
                draw_tuning_confirmation(stdscr)
                while True:
                    k = stdscr.getch()
                    if k in [curses.KEY_ENTER, 10, 13]:
                        send_serial_slow(ser, ":G\r\n")
                        # Esperar respuesta de inicio
                        timeout = time.time() + 2.0
                        reponse = None
                        while time.time() < timeout:
                            code = freq_obj.get_code()
                            if code:
                                if code == 'G':
                                    status_msg = "Lecture de la chanson..."
                                    reponse = 'ok'
                                    break
                                elif code.startswith('X'):
                                    status_msg = f"Erreur : {ERROR_CODES.get(code, 'Erreur lors de la lecture de la chanson')}"
                                    reponse = 'error'
                                    break
                            time.sleep(0.05)
                        if reponse is None:
                            status_msg = "Pas de réponse du microcontrôleur lors de la lecture de la chanson."
                        # Esperar :GR para saber que terminó
                        if reponse == 'ok':
                            stdscr.timeout(100)
                            while True:
                                code = freq_obj.get_code()
                                if code == 'GR':
                                    status_msg = "Chanson terminée"
                                    break
                                draw_menu(stdscr, selected, menu, titre, status_msg)
                                k = stdscr.getch()
                                if k in [ord('q'), ord('Q')]:
                                    break
                            stdscr.timeout(-1)
                        break
            elif menu[selected] == "osciller le mediator":
                if not motor_enabled:
                    status_msg = "Activez d'abord le moteur !"
                    continue
                send_serial_slow(ser, ":I\r\n")
                # Mostrar barra de progreso y esperar respuesta en pantalla de homing
                progress = 0.0
                start = time.time()
                stdscr.timeout(100)
                homing_done = False
                homing_error = None
                while not homing_done and not homing_error:
                    code = freq_obj.get_code()
                    if code:
                        if code == 'I':
                            status_msg = "grattage terminé."
                            homing_done = True
                        elif code.startswith('X'):
                            status_msg = f"Erreur : {ERROR_CODES.get(code, 'Erreur lors du grattage')}"
                            homing_error = True
                    elapsed = time.time() - start
                    progress = min(1.0, elapsed/10.0)  # Simula barra de hasta 10 segundos
                    draw_progress_bar(stdscr, progress, label="grattage en cours...")
                    k = stdscr.getch()
                    if k in [ord('q'), ord('Q')]:
                        break
                stdscr.timeout(-1)
            elif menu[selected] == "jouer le mediator":
                if not motor_enabled:
                    status_msg = "Activez d'abord le moteur !"
                    continue
                send_serial_slow(ser, ":J\r\n")
                # Mostrar barra de progreso y esperar respuesta en pantalla de homing
                progress = 0.0
                start = time.time()
                stdscr.timeout(100)
                homing_done = False
                homing_error = None
                while not homing_done and not homing_error:
                    code = freq_obj.get_code()
                    if code:
                        if code == 'J':
                            status_msg = "grattage en cours..."
                            homing_done = True
                        elif code.startswith('X'):
                            status_msg = f"Erreur : {ERROR_CODES.get(code, 'Erreur lors du grattage')}"
                            homing_error = True
                    elapsed = time.time() - start
                    progress = min(1.0, elapsed/10.0)  # Simula barra de hasta 10 segundos
                    draw_progress_bar(stdscr, progress, label="grattage en cours...")
                    k = stdscr.getch()
                    if k in [ord('q'), ord('Q')]:
                        break
                stdscr.timeout(-1)
            elif menu[selected] == "arreter le mediator":
                if not motor_enabled:
                    status_msg = "Activez d'abord le moteur !"
                    continue
                send_serial_slow(ser, ":K\r\n")
                # Mostrar barra de progreso y esperar respuesta en pantalla de homing
                progress = 0.0
                start = time.time()
                stdscr.timeout(100)
                homing_done = False
                homing_error = None
                while not homing_done and not homing_error:
                    code = freq_obj.get_code()
                    if code:
                        if code == 'K':
                            status_msg = "grattage arreté."
                            homing_done = True
                        elif code.startswith('X'):
                            status_msg = f"Erreur : {ERROR_CODES.get(code, 'Erreur lors du grattage')}"
                            homing_error = True
                    elapsed = time.time() - start
                    progress = min(1.0, elapsed/10.0)  # Simula barra de hasta 10 segundos
                    draw_progress_bar(stdscr, progress, label="grattage en cours...")
                    k = stdscr.getch()
                    if k in [ord('q'), ord('Q')]:
                        break
                stdscr.timeout(-1)
            elif k in [ord('q'), ord('Q')]:
                break
    freq_obj.stop()
    ser.close()

if __name__ == "__main__":
    curses.wrapper(main) 