import cv2
import mediapipe as mp
import numpy as np
from models.Inverted_Pendulum_Cart_Walk_COM_control import Inverted_Pendulum_Cart_Walk_COM_control
from models.Classical_Pendulum import Classic_Pendulum_EK_control

def process_knee(input_file, output_file):
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

     image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
     image.flags.writeable = False
     results = pose.process(image)

     image.flags.writeable = True
     image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
     mp_drawing.draw_landmarks(
     image, results.pose_landmarks, mp_pose.POSE_CONNECTIONS)
     if results.pose_landmarks is not None:
      knee_joint = np.array([results.pose_landmarks.landmark[mp_pose.PoseLandmark(26).value].x, results.pose_landmarks.landmark[mp_pose.PoseLandmark(26).value].y])
      foot_joint = np.array([results.pose_landmarks.landmark[mp_pose.PoseLandmark(28).value].x, results.pose_landmarks.landmark[mp_pose.PoseLandmark(28).value].y])
 
     if iter == 0:
      leg_vector = foot_joint - knee_joint
      start_angle = np.arccos(np.dot([1, 0], leg_vector)/np.linalg.norm(leg_vector))
      pendulum = Classic_Pendulum_EK_control(70, 1, np.linalg.norm(leg_vector), [start_angle, 0], 3*np.pi/2)
      x = pendulum.make_model()
      th = x[:, 0]
      px = np.linalg.norm(leg_vector)*np.cos(th) + knee_joint[0]
      py = np.linalg.norm(leg_vector)*np.sin(th) + knee_joint[1]

     image = cv2.circle(image, (round(px[iter]*frame_width), round(py[iter]*frame_height)), 3, (210, 255, 0), 3)
     image = cv2.line(image, (round(px[iter]*frame_width), round(py[iter]*frame_height)), (round((knee_joint[0])*frame_width), round(knee_joint[1]*frame_height)), [0, 255, 0], 2)
     if iter + 1 < len(px):
      iter = iter + 1

     out.write(image)

    pose.close()
    cap.release()
    out.release()