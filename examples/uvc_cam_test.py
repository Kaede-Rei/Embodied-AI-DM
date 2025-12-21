import cv2

def main():
    cam_index = 0  # Change this if your camera is not at index 0
    cap = cv2.VideoCapture(cam_index)

    if not cap.isOpened():
        print(f"Cannot open camera at index {cam_index}")
        return

    print("Press 'q' to quit.")
    while True:
        ret, frame = cap.read()
        if not ret:
            print("Failed to grab frame")
            break

        cv2.imshow('USB Camera Test', frame)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
