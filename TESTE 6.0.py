import dlib
import cv2
import numpy as np
import time
import tkinter as tk
from PIL import Image, ImageTk
from threading import Thread


# Função para calcular a inclinação da cabeça (yaw, pitch)
def calculate_head_orientation(landmarks):
    nose = np.array(landmarks[30])
    chin = np.array(landmarks[8])  # Pode ser afetado pela barba
    left_eye = np.array(landmarks[36])
    right_eye = np.array(landmarks[45])

    eye_vector = right_eye - left_eye
    nose_chin_vector = chin - nose

    # Cálculo do yaw (movimento horizontal) e pitch (movimento vertical)
    yaw_angle = np.degrees(np.arctan2(eye_vector[1], eye_vector[0]))

    # Ajuste para compensar a barba grande (recalibração da inclinação vertical)
    chin_adjustment_factor = 1.0  # Fator de ajuste para a posição do queixo devido à barba
    pitch_angle = np.degrees(np.arctan2(nose_chin_vector[1] * chin_adjustment_factor, np.linalg.norm(nose_chin_vector)))

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
    # Convertemos o frame do OpenCV (BGR) para o formato RGB
    frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

    # Convertendo o frame do OpenCV para imagem PIL
    img_pil = Image.fromarray(frame_rgb)

    # Convertendo a imagem PIL para ImageTk para exibir no Tkinter
    img_tk = ImageTk.PhotoImage(image=img_pil)

    # Atualizando o widget da câmera com a nova imagem
    camera_label.imgtk = img_tk
    camera_label.config(image=img_tk)


# Função que desenha os landmarks no rosto e uma linha para mostrar a direção da cabeça
def draw_landmarks_and_direction(frame, landmarks_points, yaw, pitch):
    # Desenhar os pontos faciais
    for (x, y) in landmarks_points:
        cv2.circle(frame, (x, y), 2, (255, 0, 0), -1)

    # Desenhar uma linha vertical do nariz até o queixo para mostrar a orientação
    nose = landmarks_points[30]
    chin = landmarks_points[8]
    cv2.line(frame, nose, chin, (0, 255, 0), 2)

    # Adicionar texto com o valor do yaw e pitch
    cv2.putText(frame, f"Yaw: {int(yaw)}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
    cv2.putText(frame, f"Pitch: {int(pitch)}", (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)


# Função que rodará a detecção de movimentos da cabeça e atualizará o frame da câmera
def detect_head_movements():
    detector = dlib.get_frontal_face_detector()
    predictor = dlib.shape_predictor("shape_predictor_68_face_landmarks.dat")

    cap = cv2.VideoCapture(0)

    # Ajuste de parâmetros da câmera
    cap.set(cv2.CAP_PROP_GAIN, 0)
    cap.set(cv2.CAP_PROP_BRIGHTNESS, 150)
    cap.set(cv2.CAP_PROP_CONTRAST, 50)

    while True:
        ret, frame = cap.read()

        if not ret:
            break

        # Aplicar filtro Gaussiano para reduzir o ruído
        frame = cv2.GaussianBlur(frame, (5, 5), 0)

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        faces = detector(gray)

        for face in faces:
            landmarks = predictor(gray, face)

            # Extraindo as coordenadas das landmarks relevantes
            landmarks_points = [(landmarks.part(n).x, landmarks.part(n).y) for n in range(68)]

            # Calcular a orientação da cabeça
            yaw, pitch = calculate_head_orientation(landmarks_points)

            # Lógica para comandos com base na inclinação da cabeça
            comando = "Aguardando detecção..."

            # Definindo uma zona neutra para a cabeça reta
            pitch_threshold_up = 100  # Inclinada para frente
            pitch_threshold_down = -60 # Inclinada para trás

            if yaw < -15:  # Cabeça virada à direita
                comando = "Comando 4: Virar à direita"
            elif yaw > 15:  # Cabeça virada à esquerda
                comando = "Comando 3: Virar à esquerda"
            elif pitch > pitch_threshold_down:  # Cabeça inclinada para trás
                comando = "Comando 2: Andar para trás"
            elif pitch < pitch_threshold_up:  # Cabeça inclinada para frente
                comando = "Comando 1: Andar para frente"

            # Atualizar a interface gráfica com o comando detectado
            command_label.config(text=comando)
            update_arrows(up_arrow, down_arrow, left_arrow, right_arrow, comando)

            # Desenhar landmarks e direção no frame
            draw_landmarks_and_direction(frame, landmarks_points, yaw, pitch)

        # Atualizar o frame da câmera no Tkinter
        update_camera_frame(frame)

        # Reduzir o atraso para melhorar a resposta
        time.sleep(0.1)

    cap.release()


# Criar uma thread para rodar a detecção sem bloquear a interface gráfica
detect_thread = Thread(target=detect_head_movements)
detect_thread.daemon = True
detect_thread.start()

# Rodar o loop da interface
root.mainloop()




