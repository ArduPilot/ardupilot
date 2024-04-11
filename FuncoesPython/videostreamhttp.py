import cv2
from flask import Flask, Response
import argparse



app = Flask(__name__)

def generate_frames():
    camera = cv2.VideoCapture(0)  # Use 0 para a primeira câmera conectada

    while True:
        success, frame = camera.read()  # Ler o frame da câmera
        if not success:
            break
        else:
            ret, buffer = cv2.imencode('.jpg', frame)
            frame = buffer.tobytes()
            yield (b'--frame\r\n'
                   b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n')

@app.route('/video')
def video():
    return Response(generate_frames(), mimetype='multipart/x-mixed-replace; boundary=frame')

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Descricao dos parametros de configuracao')

    parser.add_argument('--addr_server', type=str, help='endereco servidor', default='192.168.0.187')
    parser.add_argument('--server_port', type=int, help='porta', default=5000)
    args = parser.parse_args()
    
    app.run(host=args.addr_server, port=args.server_port)


