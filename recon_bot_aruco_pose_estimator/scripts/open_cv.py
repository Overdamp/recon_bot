# -*- coding: utf-8 -*-
import cv2
import numpy as np

# นำเข้าข้อมูล camera_matrix และ dist_coeff
camera_matrix = np.array([[1628.241678181888, 0.0, 1003.6452346168742],
                          [0.0, 1620.5433007003242, 496.96109096520547],
                          [0.0, 0.0, 1.0]])
dist_coeff = np.array([-0.09312356072026592, 0.33806553912219084, -0.02321020264601232, 0.003718081308981547, -1.2224078994046312])

# เริ่มต้นการเชื่อมต่อกล้อง (คุณอาจต้องระบุดัชนีของกล้อง)
cap = cv2.VideoCapture(0)

while True:
    # อ่านภาพจากกล้อง
    ret, frame = cap.read()

    # ใช้ camera_matrix และ dist_coeff ในการปรับแก้ภาพด้วย OpenCV
    undistorted_frame = cv2.undistort(frame, camera_matrix, dist_coeff)

    # ทำตามขั้นตอนการตรวจจับ ArUco Marker หรือประมวลผลภาพต่อไปได้

    # แสดงภาพที่ได้หลังจากการปรับแก้
    cv2.imshow("Undistorted Frame", undistorted_frame)

    # รอการกดปุ่ม 'q' เพื่อออกจากโปรแกรม
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# ปิดการเชื่อมต่อกล้องและหยุดการแสดงผล
cap.release()
cv2.destroyAllWindows()
