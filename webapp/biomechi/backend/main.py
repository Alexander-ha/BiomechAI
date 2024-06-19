from flask import Flask, request, jsonify, send_from_directory, redirect, url_for, send_file
from flask_cors import CORS
import os
import cv2
import mediapipe as mp
import numpy as np
from models.Classical_Pendulum import Classic_Pendulum_EK_control
from models.Inverted_Pendulum_Cart_Walk_COM_control import Inverted_Pendulum_Cart_Walk_COM_control
from algorithms.process_knee import process_knee
from algorithms.REAM import REAM
from algorithms.process_gait import process_gait
import io


app = Flask(__name__, static_folder='static')
CORS(app)
filename = ''
UPLOAD_FOLDER = 'static/uploads'
OUTPUT_FOLDER = 'output'
app.config['UPLOAD_FOLDER'] = UPLOAD_FOLDER
app.config['OUTPUT_FOLDER'] = OUTPUT_FOLDER


@app.route('/displayvideo/<filename>')
def display_video(filename):
    video_path = f'static/output/{filename}'
    return send_file(video_path, as_attachment=True)


@app.route('/get_video', methods=['GET'])
def get_video():
    return send_file('path_to_your_video_file/video.mp4', as_attachment=True)


@app.route('/downloadvideo/<filename>', methods=['GET'])
def download_video(filename):
    filename = str(filename)
    return send_from_directory(app.config['OUTPUT_FOLDER'], 'processed_' + filename, as_attachment=True)


@app.route('/upload-video', methods=['POST'])
def upload_video():
    if 'file' not in request.files:
        return jsonify({'error': 'No file part'})

    file = request.files['file']
    option1 = request.form['option1']
    option2 = request.form['option2']
    print(option1, option2)
    if file.filename == '':
        return jsonify({'error': 'No selected file'})

    if file and file.filename.endswith('.mp4'):
        filename = file.filename
        file_path = os.path.join(app.config['UPLOAD_FOLDER'], file.filename)
        file.save(file_path)
        output_file_path = os.path.join(app.config['OUTPUT_FOLDER'], 'processed_' + file.filename)
        if option2 == 'true':
         process_knee(file_path, output_file_path)
        else:
         process_gait(file_path, output_file_path)
        return send_from_directory(app.config['OUTPUT_FOLDER'], 'processed_' + file.filename, as_attachment=True)
    else:
        return jsonify({'error': 'Invalid file format'})

if __name__ == '__main__':
    app.run(debug=True)