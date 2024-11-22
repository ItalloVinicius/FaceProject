import dlib
import cv2
import numpy as np
import time
import tkinter as tk
from PIL import Image, ImageTk
from threading import Thread
import serial  # Adiciona o módulo pySerial

# Configurar a porta serial (substitua pelo nome correto da porta e a taxa de transmissão que você está usando)
#ser = serial.Serial('COM4', 9600, timeout=1)  # Substitua 'COM3' pela porta correta no seu sistema

# Função para calcular a inclinação da cabeça (yaw, pitch) usando os olhos e o queixo
def calculate_head_orientation(landmarks):
    nose = np.array(landmarks[30])
    left_eye = np.array(landmarks[36])
    right_eye = np.array(landmarks[45])
    chin = np.array(landmarks[57])

    # Vetor horizontal entre os olhos
    eye_vector = right_eye - left_eye

    # Cálculo do yaw (movimento horizontal) com base nos olhos
    yaw_angle = np.degrees(np.arctan2(eye_vector[1], eye_vector[0]))

    # Cálculo do pitch usando o ângulo entre nariz e queixo
    face_vector = nose - chin
    pitch_angle = np.degrees(np.arctan2(face_vector[1], np.linalg.norm(face_vector)))

    # Aplicar valor absoluto no pitch
    pitch_angle = abs(pitch_angle)

    return yaw_angle, pitch_angle

# Função para atualizar a seta visual com o comando detectado
def update_arrows(up, down, left, right, command):
    if command == "Comando 1: Andar para frente":
        up.config(image=up_on_img)
        down.config(image=down_off_img)
        left.config(image=left_off_img)
        right.config(image=right_off_img)
    elif command == "Comando 2: Andar para trás":
        up.config(image=up_off_img)
        down.config(image=down_on_img)
        left.config(image=left_off_img)
        right.config(image=right_off_img)
    elif command == "Comando 3: Virar à esquerda":
        up.config(image=up_off_img)
        down.config(image=down_off_img)
        left.config(image=left_on_img)
        right.config(image=right_off_img)
    elif command == "Comando 4: Virar à direita":
        up.config(image=up_off_img)
        down.config(image=down_off_img)
        left.config(image=left_off_img)
        right.config(image=right_on_img)
    else:
        up.config(image=up_off_img)
        down.config(image=down_off_img)
        left.config(image=left_off_img)
        right.config(image=right_off_img)

# Função para enviar comandos pela porta serial
def send_serial_command(command):
    try:
        ser.write(command.encode())  # Envia o comando como uma string codificada em bytes
        print(f"Comando enviado: {command}")
    except Exception as e:
        print(f"Erro ao enviar comando: {e}")

# Inicializando Tkinter
root = tk.Tk()
root.title("Detecção de Movimentos de Cabeça e Comandos com Setas")
root.geometry("1280x720")

# Função para redimensionar as imagens das setas
def resize_image(image_path, size=(100, 100)):
    img = Image.open(image_path)
    img = img.resize(size, Image.Resampling.LANCZOS)
    return ImageTk.PhotoImage(img)

# Carregando e redimensionando as imagens das setas
up_off_img = resize_image("up_off.png")
up_on_img = resize_image("up_on.png")
down_off_img = resize_image("down_off.png")
down_on_img = resize_image("down_on.jpg")
left_off_img = resize_image("left_off.jpg")
left_on_img = resize_image("left_on.jpg")
right_off_img = resize_image("right_off.jpg")
right_on_img = resize_image("right_on.png")

# Criando os widgets das setas
up_arrow = tk.Label(root, image=up_off_img)
down_arrow = tk.Label(root, image=down_off_img)
left_arrow = tk.Label(root, image=left_off_img)
right_arrow = tk.Label(root, image=right_off_img)

# Posicionando as setas na interface
up_arrow.pack(pady=10)
left_arrow.pack(side="left", padx=50)
right_arrow.pack(side="right", padx=50)
down_arrow.pack(pady=10)

# Label para exibir o comando atual
command_label = tk.Label(root, text="Aguardando detecção...")
command_label.pack(pady=20)

# Criando widget para exibir o vídeo da câmera
camera_label = tk.Label(root)
camera_label.pack(pady=10)

# Função que atualiza a janela com os frames da câmera
def update_camera_frame(frame):
    frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
    img_pil = Image.fromarray(frame_rgb)
    img_tk = ImageTk.PhotoImage(image=img_pil)
    camera_label.imgtk = img_tk
    camera_label.config(image=img_tk)

# Função que desenha os landmarks no rosto e uma linha para mostrar a direção da cabeça
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

# Função que rodará a detecção de movimentos da cabeça e atualizará o frame da câmera
def detect_head_movements():
    detector = dlib.get_frontal_face_detector()
    predictor = dlib.shape_predictor("shape_predictor_68_face_landmarks.dat")

    cap = cv2.VideoCapture(0)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
    cap.set(cv2.CAP_PROP_GAIN, 0)
    cap.set(cv2.CAP_PROP_BRIGHTNESS, 75)
    cap.set(cv2.CAP_PROP_CONTRAST, 50)

    while True:
        ret, frame = cap.read()

        if not ret:
            break

        frame = cv2.GaussianBlur(frame, (1, 1), 0)
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        faces = detector(gray, 1)

        for face in faces:
            landmarks = predictor(gray, face)
            landmarks_points = [(landmarks.part(n).x, landmarks.part(n).y) for n in range(68)]

            # Calcular a orientação da cabeça
            yaw, pitch = calculate_head_orientation(landmarks_points)

            # Lógica para comandos com base na inclinação da cabeça
            comando = "Aguardando detecção..."
            pitch_threshold_up = 44
            pitch_threshold_down = 15

            if yaw < -15:
                comando = "Comando 4: Virar à direita"
            elif yaw > 15:
                comando = "Comando 3: Virar à esquerda"
            elif pitch > pitch_threshold_down:
                comando = "Comando 2: Andar para trás"
            elif pitch < pitch_threshold_up:
                comando = "Comando 1: Andar para frente"

            command_label.config(text=comando)
            update_arrows(up_arrow, down_arrow, left_arrow, right_arrow, comando)

            # Enviar o comando via serial
            send_serial_command(comando)

            draw_landmarks_and_direction(frame, landmarks_points, yaw, pitch)

        update_camera_frame(frame)
        time.sleep(0.0001)

    cap.release()

# Criar uma thread para rodar a detecção sem bloquear a interface gráfica
detect_thread = Thread(target=detect_head_movements)
detect_thread.daemon = True
detect_thread.start()

# Rodar o loop da interface
root.mainloop()
