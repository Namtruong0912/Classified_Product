# ..............................thư viện
import cv2
import numpy as np
import serial
import threading
import struct
import time

# ..............................Các biến
# Biến để kiểm soát luồng
running = True

# Biến để lưu giữ tín hiệu từ cổng COM
com_signal = None

color_result = None

color_result_number = None

# Biến lưu trạng thái hiển thị video hoặc ảnh
display_state = "video"

# ..............................Các hàm
# Hàm đọc dữ liệu từ cổng COM
def read_com_data():
    global com_signal
    while running:
        try:
            com_signal = ser.read().decode('utf-8')
            # com_signal = ser.readline().decode('utf-8').strip()
        except serial.SerialException:
            pass

# Hàm gửi dữ liệu ra qua cổng com
def send_result_to_com(ser, results):
    # Convert the results list to a formatted string
    result_text = ''.join([text for (text,) in results])
    # Gửi dữ liệu kết quả
    ser.write(result_text.encode('utf-8'))

# ................................Setup

# Mở kết nối đến cổng COM (điều chỉnh tên cổng COM tương ứng của bạn)
ser = serial.Serial('COM11', 9600, timeout=1)

# Bắt đầu luồng đọc dữ liệu từ cổng COM
com_thread = threading.Thread(target=read_com_data)
com_thread.start()

# Mở kết nối đến video stream
# cap = cv2.VideoCapture(video_url)
cap = cv2.VideoCapture(1, cv2.CAP_DSHOW)

# .....................................loop
while True:
    ret, frame = cap.read()

    # Kiểm tra xem việc đọc frame có thành công không
    if not ret:
        print("Error reading frame")
        break

    cv2.imshow("ket qua1", frame)

    # So sánh tín hiệu từ cổng COM
    if com_signal == "1":
        print("nhận dữ liệu từ vi điều khiển")

        # Gọi hàm phân loại sản phẩm
        # Đọc ảnh từ đường dẫn
        image = frame

        # Chuyển đổi không gian màu từ BGR sang HSV để dễ so sánh và nhận diện màu hơn
        hsv_image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        # Đặt ngưỡng màu cho màu đỏ
        lower_red = np.array([0, 100, 100])
        upper_red = np.array([10, 255, 255])
        # lower_red = np.array([0, 198, 76])
        # upper_red = np.array([0, 198, 25])
        red_mask = cv2.inRange(hsv_image, lower_red, upper_red)

        # Đặt ngưỡng màu cho màu lam
        # lower_blue = np.array([93, 79, 2])
        # upper_blue = np.array([122, 255, 255])
        lower_blue = np.array([100, 150, 50])
        upper_blue = np.array([140, 255, 255])
        blue_mask = cv2.inRange(hsv_image, lower_blue, upper_blue)

        # Kết hợp mask của đỏ và lam
        combined_mask = cv2.bitwise_or(red_mask, blue_mask)

        # Find contours in the mask
        contours, _ = cv2.findContours(combined_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        # Áp dụng mask để lấy các pixel cần thiết
        result_image = cv2.bitwise_and(image, image, mask=combined_mask)

        # Xác định màu dựa trên mask
        if (cv2.countNonZero(red_mask) > cv2.countNonZero(blue_mask)) & (cv2.countNonZero(red_mask) > 5000):
            color_result = "DO"
            color_result_number = 1
            # print("DO")
        else:
            if (cv2.countNonZero(blue_mask) > cv2.countNonZero(red_mask)) & (cv2.countNonZero(blue_mask) > 10000):
                color_result = "XANH"
                color_result_number = 2
                # print("XANH")
            else:
                color_result = "KHONG XAC DINH"
                color_result_number = 0
                # print("KHONG XAC DINH")

        # vẽ viền cho vật
        for contour in contours:
            area = cv2.contourArea(contour)
            if area > 1000:
                x, y, w, h = cv2.boundingRect(contour)
                cv2.rectangle(result_image, (x, y), (x + w, y + h), (0, 255, 0), 2)

                # Hiển thị thông tin kết quả lên frame
                cv2.putText(result_image, f'{color_result}', (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)


        cv2.imshow("Ket qua2", result_image)

        if color_result_number == 1:
            send_result_to_com(ser, "1")
            print("DO")
        if color_result_number == 2:
            send_result_to_com(ser, "2")
            print("XANH")
        if color_result_number == 0:
            send_result_to_com(ser, "0")
            print("Khong xac dinh")
        color_result_number = 0
        com_signal = None

    # Kiểm tra phím nhấn để thoát
    if cv2.waitKey(1) & 0xFF == ord('o'):
        break

# Giải phóng tài nguyên và đóng cửa sổ
cap.release()
cv2.destroyAllWindows()
