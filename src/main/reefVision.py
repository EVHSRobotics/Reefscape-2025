import cv2
import numpy as np
import tflite_runtime.interpreter as tflite
from networktables import NetworkTables
import time


NetworkTables.initialize(server='10.2854.2')  
stage_table = NetworkTables.getTable("StageDetection")
priorityLevel =["L4", "L3", "L2", "L1"]
prioritySide = ["LEFT", "RIGHT"]

interpreter = tflite.Interpreter(model_path="locationasfjasdifw")
interpreter.allocate_tensors()

input_details = interpreter.get_input_details()
output_details = interpreter.get_output_details()
height = input_details[0]['shape'][1]
width = input_details[0]['shape'][2]

cap = cv2.VideoCapture(0) 

positions = {
    "L1_CLOSE": 0,
    "L1_FAR": 1,
    "L2_LEFT": 2,
    "L2_RIGHT": 3,
    "L3_LEFT": 4,
    "L3_RIGHT": 5,
    "L4_LEFT": 6,
    "L4_RIGHT": 7
}

while True:
    ret, frame = cap.read()
    if not ret:
        continue
    
    img_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
    img_resized = cv2.resize(img_rgb, (width, height))
    input_data = np.expand_dims(img_resized, axis=0).astype(np.float32) / 255.0
    
    interpreter.set_tensor(input_details[0]['index'], input_data)
    interpreter.invoke()
    
    output_data = interpreter.get_tensor(output_details[0]['index'])
    
# 1 is filled 0 not
    filled_positions = []
    for i, prob in enumerate(output_data[0]):
        position_name = list(positions.keys())[i]
        if prob > 0.9:  # confidence test out
            filled_positions.append(position_name)

            cv2.putText(frame, f"{position_name}: FILLED", (10, 30 + i*30), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        else:
            cv2.putText(frame, f"{position_name}: EMPTY", (10, 30 + i*30), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
    
    all_positions = list(positions.keys())
    available_positions = [pos for pos in all_positions if pos not in filled_positions]
    
    next_position = None
    if available_positions:

        for level in priorityLevel:
            for side in prioritySide:
                check_pos = f"{level}_{side}"
                if check_pos in available_positions:
                    next_position = check_pos
                    break
            if next_position:
                break
    

    if next_position:
        stage_table.putString("next_position", next_position)
        cv2.putText(frame, f"NEXT: {next_position}", (10, height - 30), 
                    cv2.FONT_HERSHEY_SIMPLEX, 1.0, (255, 0, 0), 2)
    else:
        stage_table.putString("next_position", "FULL")
        cv2.putText(frame, "STAGE FULL", (10, height - 30), 
                    cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 0, 255), 2)
 
    time.sleep(0.05)  # 20 FPS processing

