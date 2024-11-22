import dlib
import cv2
import numpy as np
import time
import tkinter as tk
from PIL import Image, ImageTk
from threading import Thread
import socket
import sys

# Tentativa de importar a API do CoppeliaSim
try:
    import sim
except:
    print("Erro ao importar a API do CoppeliaSim")

# Endereço do servidor
server_address = ('127.0.0.1', 65432)

# Função de inicialização do socket
def initialize_socket():
    client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    try:
        time.sleep(1)  # Espera um segundo antes de tentar conectar
        client_socket.connect(server_address)
        print("Conectado ao servidor.")
        return client_socket
    except Exception as e:
        print(f"Erro ao conectar ao servidor: {e}")
        sys.exit(1)  # Fecha o programa se não conseguir conectar

# Inicializa o socket para comunicação com o servidor
client_socket = initialize_socket()

# Função para enviar comando ao servidor socket
def send_socket_command(command):
    try:
        client_socket.sendall(command.encode('utf-8'))
        print(f"Comando enviado via socket: {command}")
    except Exception as e:
        print(f"Erro ao enviar comando via socket: {e}")

# Inicializa a conexão com o CoppeliaSim
def initialize_coppelia():
    sim.simxFinish(-1)  # Fecha conexões anteriores
    clientID = sim.simxStart('127.0.0.1', 19999, True, True, 5000, 5)  # Conectar ao CoppeliaSim
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
        sim.simxAddStatusbarMessage(clientID, f'Comando: {command}', sim.simx_opmode_oneshot)
        print(f"Comando enviado para CoppeliaSim: {command}")
    except Exception as e:
        print(f"Erro ao enviar comando para CoppeliaSim: {e}")

initial_nose_y = None

# Função para calcular o pitch usando o nariz
def calculate_pitch_using_nose(landmarks):
    global initial_nose_y
    nose = np.array(landmarks[30])

    if initial_nose_y is None:
        initial_nose_y = nose[1]

    nose_displacement = nose[1] - initial_nose_y
    threshold = 20  # Ajustado para maior range da posição neutra

    if nose_displacement > threshold:
        return "1", nose_displacement
    elif nose_displacement < -threshold:
        return "2", nose_displacement
    else:
        return "Cabeça neutra", nose_displacement

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
root.geometry("1280x720")
root.configure(bg="#282828")

def resize_image(image_path, size=(100, 100)):
    img = Image.open(image_path)
    img = img.resize(size, Image.Resampling.LANCZOS)
    return ImageTk.PhotoImage(img)

# Carregar as imagens das setas
up_off_img = resize_image("up_off.png")
up_on_img = resize_image("up_on.png")
down_off_img = resize_image("down_off.png")
down_on_img = resize_image("down_on.jpg")
left_off_img = resize_image("left_off.jpg")
left_on_img = resize_image("left_on.jpg")
right_off_img = resize_image("right_off.jpg")
right_on_img = resize_image("right_on.png")

# Layout para as setas
frame_arrows = tk.Frame(root, bg="#282828")
frame_arrows.pack(pady=20)

up_arrow = tk.Label(frame_arrows, image=up_off_img, bg="#282828")
down_arrow = tk.Label(frame_arrows, image=down_off_img, bg="#282828")
left_arrow = tk.Label(frame_arrows, image=left_off_img, bg="#282828")
right_arrow = tk.Label(frame_arrows, image=right_off_img, bg="#282828")

up_arrow.grid(row=0, column=1, pady=10)
left_arrow.grid(row=1, column=0, padx=50)
right_arrow.grid(row=1, column=2, padx=50)
down_arrow.grid(row=2, column=1, pady=10)

command_label = tk.Label(root, text="Aguardando detecção...", font=("Helvetica", 16), fg="white", bg="#282828")
command_label.pack(pady=20)

camera_label = tk.Label(root, bg="#282828")
camera_label.pack(pady=10)

def update_camera_frame(frame):
    frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
    img_pil = Image.fromarray(frame_rgb)
    img_tk = ImageTk.PhotoImage(image=img_pil)
    camera_label.imgtk = img_tk
    camera_label.config(image=img_tk)

def draw_landmarks_and_direction(frame, landmarks_points, yaw, pitch):
    for (x, y) in landmarks_points:
        cv2.circle(frame, (x, y), 2, (255, 0, 0), -1)

    left_eye = landmarks_points[36]
    right_eye = landmarks_points[45]
    nose = landmarks_points[30]
    cv2.line(frame, left_eye, right_eye, (0, 255, 0), 2)
    cv2.line(frame, ((left_eye[0] + right_eye[0]) // 2, (left_eye[1] + right_eye[1]) // 2), nose, (0, 255, 0), 2)

    cv2.putText(frame, f"Yaw: {int(yaw)}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
    cv2.putText(frame, f"Pitch: {int(pitch)}", (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)

def detect_head_movements():
    detector = dlib.get_frontal_face_detector()
    predictor = dlib.shape_predictor("shape_predictor_68_face_landmarks.dat")

    cap = cv2.VideoCapture(0)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)

    while True:
        ret, frame = cap.read()
        if not ret:
            break

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        faces = detector(gray, 1)

        for face in faces:
            landmarks = predictor(gray, face)
            landmarks_points = [(landmarks.part(n).x, landmarks.part(n).y) for n in range(68)]

            yaw = calculate_yaw_using_eyes(landmarks_points)
            comando_pitch, pitch_value = calculate_pitch_using_nose(landmarks_points)

            comando = comando_pitch
            yaw_threshold = 15

            if yaw < -yaw_threshold:
                comando = "4"
            elif yaw > yaw_threshold:
                comando = "3"

            command_label.config(text=comando)
            send_socket_command(comando)  # Enviar comando via socket
            send_coppelia_command(comando)  # Enviar comando para CoppeliaSim

            draw_landmarks_and_direction(frame, landmarks_points, yaw, pitch_value)

        update_camera_frame(frame)
        time.sleep(0.0001)

    cap.release()

# Thread para detecção de movimento de cabeça
detection_thread = Thread(target=detect_head_movements)
detection_thread.daemon = True
detection_thread.start()

# Inicializa a interface gráfica
root.mainloop()

# Fechar as conexões após a execução
client_socket.close()
sim.simxFinish(clientID)
