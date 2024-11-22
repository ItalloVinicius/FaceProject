import dlib
import cv2
import numpy as np
import time
import tkinter as tk
from PIL import Image, ImageTk
from threading import Thread
import sys
from collections import deque  # Import necessário para a média móvel

# Adicione o caminho para a API do CoppeliaSim
sys.path.append('Caminho/para/a/pasta/do/CoppeliaSim/programming/remoteApiBindings/python/python')  # Substitua pelo caminho real

# Importa a API do CoppeliaSim
try:
    import sim
except Exception as e:
    print("Erro ao importar a API do CoppeliaSim:", e)
    sys.exit(1)

# Inicializa a conexão com o CoppeliaSim
def initialize_coppelia():
    sim.simxFinish(-1)  # Fecha conexões anteriores
    clientID = sim.simxStart('127.0.0.1', 19997, True, True, 5000, 5)  # Conectar ao CoppeliaSim
    if clientID != -1:
        print("Conectado ao CoppeliaSim")
        return clientID
    else:
        print("Falha ao conectar ao CoppeliaSim")
        sys.exit(1)

clientID = initialize_coppelia()

# Função para enviar comando ao CoppeliaSim
def send_coppelia_command(command):
    try:
        # Enviar o comando como sinal de string
        sim.simxSetStringSignal(clientID, 'robot_command', command, sim.simx_opmode_oneshot)
        print(f"Comando enviado para CoppeliaSim: {command}")
    except Exception as e:
        print(f"Erro ao enviar comando para CoppeliaSim: {e}")

initial_nose_y = None

# Filas para armazenar os últimos valores de pitch e yaw
pitch_values = deque(maxlen=5)  # Média móvel dos últimos 5 frames
yaw_values = deque(maxlen=5)

# Função para calcular o pitch usando o nariz
def calculate_pitch_using_nose(landmarks):
    global initial_nose_y
    nose = np.array(landmarks[30])

    if initial_nose_y is None:
        initial_nose_y = nose[1]

    nose_displacement = nose[1] - initial_nose_y
    return nose_displacement

# Função para calcular o yaw (giro) usando os olhos
def calculate_yaw_using_eyes(landmarks):
    left_eye = np.array(landmarks[36])
    right_eye = np.array(landmarks[45])
    eye_vector = right_eye - left_eye
    yaw_angle = np.degrees(np.arctan2(eye_vector[1], eye_vector[0]))
    return yaw_angle

# Inicializando Tkinter para a interface gráfica
root = tk.Tk()
root.title("Detecção de Movimentos de Cabeça e Comandos com Setas")
root.geometry("800x600")
root.configure(bg="#282828")

# Frame principal para posicionar a câmera e as setas
frame_main = tk.Frame(root, bg="#282828")
frame_main.pack(expand=True)

# Rótulo da câmera
camera_label = tk.Label(frame_main, bg="#282828", bd=2, relief="solid")

# Função para desenhar setas com Canvas (sem imagens externas)
def create_arrow(canvas, direction, color):
    canvas.delete("all")
    if direction == 'up':
        canvas.create_polygon([50, 10, 90, 90, 10, 90], fill=color)
    elif direction == 'down':
        canvas.create_polygon([50, 90, 90, 10, 10, 10], fill=color)
    elif direction == 'left':
        canvas.create_polygon([10, 50, 90, 10, 90, 90], fill=color)
    elif direction == 'right':
        canvas.create_polygon([90, 50, 10, 10, 10, 90], fill=color)

# Criação dos canvas para setas
canvas_up = tk.Canvas(frame_main, width=100, height=100, bg="#282828", highlightthickness=0)
create_arrow(canvas_up, 'up', 'white')

canvas_left = tk.Canvas(frame_main, width=100, height=100, bg="#282828", highlightthickness=0)
create_arrow(canvas_left, 'left', 'white')

canvas_right = tk.Canvas(frame_main, width=100, height=100, bg="#282828", highlightthickness=0)
create_arrow(canvas_right, 'right', 'white')

canvas_down = tk.Canvas(frame_main, width=100, height=100, bg="#282828", highlightthickness=0)
create_arrow(canvas_down, 'down', 'white')

# Posicionamento dos elementos usando grid
canvas_up.grid(row=0, column=1, pady=5)
canvas_left.grid(row=1, column=0, padx=5)
camera_label.grid(row=1, column=1)
canvas_right.grid(row=1, column=2, padx=5)
canvas_down.grid(row=2, column=1, pady=5)

# Texto de comando com fonte e cor modificadas
command_label = tk.Label(root, text="Aguardando detecção...", font=("Helvetica", 18, "bold"), fg="#00FF00", bg="#000000")
command_label.pack(pady=10)

# Função para atualizar as setas com base no comando detectado
def update_arrows_based_on_command(command):
    if command == "1":  # Cabeça inclinada para baixo
        create_arrow(canvas_up, 'up', 'white')
        create_arrow(canvas_down, 'down', 'red')  # Acende a seta para baixo
        create_arrow(canvas_left, 'left', 'white')
        create_arrow(canvas_right, 'right', 'white')
    elif command == "2":  # Cabeça inclinada para cima
        create_arrow(canvas_up, 'up', 'red')  # Acende a seta para cima
        create_arrow(canvas_down, 'down', 'white')
        create_arrow(canvas_left, 'left', 'white')
        create_arrow(canvas_right, 'right', 'white')
    elif command == "3":  # Cabeça virada para a direita
        create_arrow(canvas_up, 'up', 'white')
        create_arrow(canvas_down, 'down', 'white')
        create_arrow(canvas_left, 'left', 'white')
        create_arrow(canvas_right, 'right', 'red')  # Acende a seta para direita
    elif command == "4":  # Cabeça virada para a esquerda
        create_arrow(canvas_up, 'up', 'white')
        create_arrow(canvas_down, 'down', 'white')
        create_arrow(canvas_left, 'left', 'red')  # Acende a seta para esquerda
        create_arrow(canvas_right, 'right', 'white')
    else:  # Parar, apaga todas as setas
        create_arrow(canvas_up, 'up', 'white')
        create_arrow(canvas_down, 'down', 'white')
        create_arrow(canvas_left, 'left', 'white')
        create_arrow(canvas_right, 'right', 'white')

def update_camera_frame(frame):
    frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
    img_pil = Image.fromarray(frame_rgb)
    img_tk = ImageTk.PhotoImage(image=img_pil)
    camera_label.imgtk = img_tk
    camera_label.config(image=img_tk)

# Função para desenhar landmarks e vetores de direção com cor e espessura alteradas
def draw_landmarks_and_direction(frame, landmarks_points, yaw, pitch):
    for (x, y) in landmarks_points:
        cv2.circle(frame, (x, y), 3, (0, 255, 0), -1)  # Pontos verdes maiores

    left_eye = landmarks_points[36]
    right_eye = landmarks_points[45]
    nose = landmarks_points[30]
    cv2.line(frame, left_eye, right_eye, (255, 255, 0), 3)  # Linha entre olhos amarela
    cv2.line(frame, ((left_eye[0] + right_eye[0]) // 2, (left_eye[1] + right_eye[1]) // 2), nose, (255, 0, 255), 3)  # Linha para o nariz magenta

    # Desenhar contorno branco para o texto
    cv2.putText(frame, f"Yaw: {int(yaw)}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 4)
    cv2.putText(frame, f"Pitch: {int(pitch)}", (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 4)

    # Desenhar texto preto por cima
    cv2.putText(frame, f"Yaw: {int(yaw)}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 0), 2)
    cv2.putText(frame, f"Pitch: {int(pitch)}", (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 0), 2)

def detect_head_movements():
    detector = dlib.get_frontal_face_detector()
    predictor = dlib.shape_predictor("shape_predictor_68_face_landmarks.dat")

    cap = cv2.VideoCapture(0)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)  # Alterado para 640
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)  # Alterado para 480

    while True:
        ret, frame = cap.read()
        if not ret:
            break

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        faces = detector(gray, 1)

        comando = "Parar"  # Comando padrão

        if len(faces) > 0:
            for face in faces:
                landmarks = predictor(gray, face)
                landmarks_points = [(landmarks.part(n).x, landmarks.part(n).y) for n in range(68)]

                # Calcula o yaw e pitch atuais
                yaw_current = calculate_yaw_using_eyes(landmarks_points)
                pitch_current = calculate_pitch_using_nose(landmarks_points)

                # Adiciona os valores atuais às filas
                yaw_values.append(yaw_current)
                pitch_values.append(pitch_current)

                # Calcula a média móvel
                yaw_avg = np.mean(yaw_values)
                pitch_avg = np.mean(pitch_values)

                # Definir thresholds ajustados
                yaw_threshold = 15  # Limiar reduzido para maior sensibilidade após suavização
                pitch_threshold = 20

                # Determina o comando baseado nos valores suavizados
                if pitch_avg > pitch_threshold:
                    comando = "1"  # Andar para trás
                elif pitch_avg < -pitch_threshold:
                    comando = "2"  # Andar para frente
                elif yaw_avg > yaw_threshold:
                    comando = "3"  # Virar à direita
                elif yaw_avg < -yaw_threshold:
                    comando = "4"  # Virar à esquerda
                else:
                    comando = "Parar"

                # Mapeamento dos comandos para ações
                if comando == "1":
                    action = "Andar para trás"
                elif comando == "2":
                    action = "Andar para frente"
                elif comando == "3":
                    action = "Virar à direita"
                elif comando == "4":
                    action = "Virar à esquerda"
                else:
                    action = "Parar"

                # Envia o comando e atualiza a UI
                send_coppelia_command(comando)
                command_label.config(text=action)
                update_arrows_based_on_command(comando)
                draw_landmarks_and_direction(frame, landmarks_points, yaw_avg, pitch_avg)
        else:
            # Nenhuma face detectada
            comando = "Parar"
            send_coppelia_command(comando)
            command_label.config(text="Parar")
            update_arrows_based_on_command(comando)
            # Resetar a posição inicial do nariz e limpar as filas
            global initial_nose_y
            initial_nose_y = None
            pitch_values.clear()
            yaw_values.clear()

        update_camera_frame(frame)
        time.sleep(0.0001)

    cap.release()

# Thread para detecção de movimento de cabeça
detection_thread = Thread(target=detect_head_movements)
detection_thread.daemon = True
detection_thread.start()

# Inicializa a interface gráfica
root.mainloop()

# Fechar a conexão com o CoppeliaSim
sim.simxFinish(clientID)