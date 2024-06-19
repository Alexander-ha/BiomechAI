import cv2
import mediapipe as mp
import numpy as np
from models.Inverted_Pendulum_Cart_Walk_COM_control import Inverted_Pendulum_Cart_Walk_COM_control
from models.Classical_Pendulum import Classic_Pendulum_EK_control
from algorithms.REAM import REAM


def process_gait(input_file, output_file):
    mp_drawing = mp.solutions.drawing_utils
    mp_holistic = mp.solutions.holistic
    cap = cv2.VideoCapture(input_file)
    frame_width = int(cap.get(3))
    frame_height = int(cap.get(4))
    fps = cap.get(cv2.CAP_PROP_FPS)
    frame_count = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
    print(frame_count)
    duration = frame_count/fps
    dt = 1/fps
    out = cv2.VideoWriter(output_file, cv2.VideoWriter_fourcc(*'MP4V'), 10, (frame_width,frame_height))
    mp_pose = mp.solutions.pose
    pose = mp_pose.Pose(min_detection_confidence=0.5, min_tracking_confidence=0.5)
    iter = 0
    Tmax = 0
    trajectory = []
    while cap.isOpened():
     ret, image = cap.read()
     if not ret:
      break

     image = cv2.cvtColor(cv2.flip(image, 1), cv2.COLOR_BGR2RGB)
     image.flags.writeable = False
     results = pose.process(image)

     image.flags.writeable = True
     image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
     mp_drawing.draw_landmarks(
     image, results.pose_landmarks, mp_pose.POSE_CONNECTIONS)

     knee_joint = np.array([results.pose_landmarks.landmark[mp_pose.PoseLandmark(26).value].x, results.pose_landmarks.landmark[mp_pose.PoseLandmark(26).value].y])
     foot_joint = np.array([results.pose_landmarks.landmark[mp_pose.PoseLandmark(28).value].x, results.pose_landmarks.landmark[mp_pose.PoseLandmark(28).value].y])

     if iter == 0:
      Ream_Alg = REAM(70, results.pose_landmarks, iter, dt)

     else:
      Ream_Alg.Update_Data(results.pose_landmarks, iter)
      x = Ream_Alg.Barycenter_Get()

      if iter == 1:
       Leg_len = Ream_Alg.get_Leg_Len()
       Y, th, x, x0, Z = Ream_Alg.get_Init_Data()
       Inverted_Pendulum_Model = Inverted_Pendulum_Cart_Walk_COM_control(start = np.array([0, 0, th, 0]))
       print('Inverted pendulum generated')
       xs = Inverted_Pendulum_Model.make_model()

      _x = (x[0], x[1])
      x = (round(x[0]*frame_width), round(x[1]*frame_height))
      image = cv2.circle(image, x, 10, (255, 0, 0), 3)
      if iter>=1:
       xi = xs[iter]
       ths = xi[0]
       xcentre = xi[2]
       pxs = (Leg_len/10)*np.sin(ths) + xcentre/5 + _x[0]
       pys = (Leg_len/13)*np.cos(ths) + _x[1]
       pxs = round(pxs*frame_width)
       pys = round(pys*frame_height)
       image = cv2.circle(image, (pxs, pys), 3, (43, 255, 0), 3)
       image = cv2.circle(image, (round((results.pose_landmarks.landmark[mp_pose.PoseLandmark(25).value].x + xcentre/10)*frame_width), round(results.pose_landmarks.landmark[mp_pose.PoseLandmark(25).value].y*frame_height)), 3, (43, 255, 0), 3)
       image = cv2.circle(image, (round(results.pose_landmarks.landmark[mp_pose.PoseLandmark(30).value].x)*frame_width, round(results.pose_landmarks.landmark[mp_pose.PoseLandmark(30).value].y)*frame_height), 10, (215, 10, 10), 3)
       image = cv2.line(image, (pxs, pys), (round((results.pose_landmarks.landmark[mp_pose.PoseLandmark(25).value].x + xcentre/10)*frame_width), round(results.pose_landmarks.landmark[mp_pose.PoseLandmark(25).value].y*frame_height)), [0, 255, 0], 2)
       image = cv2.line(image, (round((results.pose_landmarks.landmark[mp_pose.PoseLandmark(25).value].x + xcentre/10)*frame_width), round(results.pose_landmarks.landmark[mp_pose.PoseLandmark(25).value].y*frame_height)), (round((results.pose_landmarks.landmark[27].x + xcentre/10)*frame_width), round(results.pose_landmarks.landmark[27].y*frame_height)), [0, 255, 0], 2)
       image = cv2.line(image, (pxs, pys), (round((results.pose_landmarks.landmark[mp_pose.PoseLandmark(26).value].x + xcentre/10)*frame_width), round(results.pose_landmarks.landmark[mp_pose.PoseLandmark(26).value].y*frame_height)), [0, 255, 0], 2)
       image = cv2.line(image, (round((results.pose_landmarks.landmark[mp_pose.PoseLandmark(26).value].x + xcentre/10)*frame_width), round(results.pose_landmarks.landmark[mp_pose.PoseLandmark(26).value].y*frame_height)), (round((results.pose_landmarks.landmark[28].x + xcentre/10)*frame_width), round(results.pose_landmarks.landmark[28].y*frame_height)), [0, 255, 0], 2)
     out.write(image)
     iter = iter + 1
    pose.close()
    cap.release()
    out.release()
