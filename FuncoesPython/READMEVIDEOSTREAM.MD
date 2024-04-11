#Configurar o RP para Strem de video


para o stream de video stremserver e stremclient.py
no console ssh do raspberry pi

```commandline
sudo apt install python3-pip
```
```commandline
pip install --force-reinstall numpy==1.19.3
```

```commandline
pip install opencv-python
```
Trocar o Leo pela sua pasta local
```commandline
export PATH=$PATH:/home/leo/.local/bin
```

```commandline
git clone --recurse-submodules https://github.com/lmhonorio/ardupilot.git
sudo apt update
pip3 install flask opencv-python-headless
sudo apt install libgl1-mesa-glx
sudo apt-get install xvfb
xvfb-run python3 streamserver.py --addr_server_socket 192.168.0.187 --show_image False
```

rodar no PC o cliente


outra opcao e rodar usando o videostreamhttp.py

Isso iniciará o servidor Flask no Raspberry Pi. Certifique-se de que o Raspberry Pi e o celular estejam na mesma rede Wi-Fi.

Em seu celular, abra um navegador e vá para http://<IP-DO-RASPBERRY-PI>:5000/video, substituindo <IP-DO-RASPBERRY-PI> pelo endereço IP do seu Raspberry Pi. 
Você deve ser capaz de ver o stream de vídeo ao vivo da câmera conectada ao Raspberry Pi.

para conectar a mais e um cliente:

```
pip3 install gunicorn
```

e depois (trocando o ip e a porta)

```
gunicorn -w 4 videostreamhttp:app --bind 192.168.0.187:5000
```
