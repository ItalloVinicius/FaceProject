import dlib
import cv2
import numpy as np
import time
import tkinter as tk
from PIL import Image, ImageTk
from threading import Thread
import serial  # Adiciona o módulo pySerial

# Configurar a porta serial
ser = serial.Serial('COM3', 9600, timeout=1)

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
        return "Comando 2: Andar para trás", nose_displacement
    elif nose_displacement < -threshold:
        return "Comando 1: Andar para frente", nose_displacement
    else:
        return "Cabeça neutra", nose_displacement


def calculate_yaw_using_eyes(landmarks):
    left_eye = np.array(landmarks[36])
    right_eye = np.array(landmarks[45])
    eye_vector = right_eye - left_eye
    yaw_angle = np.degrees(np.arctan2(eye_vector[1], eye_vector[0]))
    return yaw_angle


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


def send_serial_command(command):
    try:
        ser.write(command.encode())
        print(f"Comando enviado: {command}")
    except Exception as e:
        print(f"Erro ao enviar comando: {e}")


# Inicializando Tkinter
root = tk.Tk()
root.title("Detecção de Movimentos de Cabeça e Comandos com Setas")
root.geometry("1280x720")
root.configure(bg="#282828")  # Fundo escuro


def resize_image(image_path, size=(100, 100)):
    img = Image.open(image_path)
    img = img.resize(size, Image.Resampling.LANCZOS)
    return ImageTk.PhotoImage(img)


up_off_img = resize_image("up_off.png")
up_on_img = resize_image("up_on.png")
down_off_img = resize_image("down_off.png")
down_on_img = resize_image("down_on.jpg")
left_off_img = resize_image("left_off.jpg")
left_on_img = resize_image("left_on.jpg")
right_off_img = resize_image("right_off.jpg")
right_on_img = resize_image("right_on.png")

# Centralizando layout e aplicando padding
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


# Melhorando visualização da câmera
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

            yaw = calculate_yaw_using_eyes(landmarks_points)
            comando_pitch, pitch_value = calculate_pitch_using_nose(landmarks_points)

            comando = comando_pitch
            yaw_threshold = 15

            if yaw < -yaw_threshold:
                comando = "Comando 4: Virar à direita"
            elif yaw > yaw_threshold:
                comando = "Comando 3: Virar à esquerda"

            command_label.config(text=comando)
            update_arrows(up_arrow, down_arrow, left_arrow, right_arrow, comando)

            send_serial_command(comando)

            draw_landmarks_and_direction(frame, landmarks_points, yaw, pitch_value)

        update_camera_frame(frame)
        time.sleep(0.0001)

    cap.release()


detection_thread = Thread(target=detect_head_movements)
detection_thread.daemon = True
detection_thread.start()

root.mainloop()
ser.close()
