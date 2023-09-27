#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

def camera1_publisher():
    #inicializa o node
    rospy.init_node('camera1_publisher', anonymous=True)
    
    #cria um objeto Publisher para publicar imagens no tópico 'camera1/image_raw'
    image_pub = rospy.Publisher('camera1/image_raw', Image, queue_size=10)
    
    #cria um objeto bridge da classe CvBridge para converter entre imagens OpenCV e mensagens utilizadas pela biblioteca do ROS usb_cam
    bridge = CvBridge()
    
    #abre a câmera 1 (TODO: índice da câmera 1 é realmente 0?)
    cap = cv2.VideoCapture(0)
    
    #define a taxa de publicação em Heartz (a ser ajustado)
    rate = rospy.Rate(10)  # 10 Hz

    while not rospy.is_shutdown():
        #lê um quadro da câmera
        ret, frame = cap.read()
        
        if ret:
            #converte o quadro para uma mensagem ROS Image
            image_msg = bridge.cv2_to_imgmsg(frame, "bgr8")
            
            #publica a mensagem no tópico
            image_pub.publish(image_msg)
        
        #controla a taxa de publicação
        rate.sleep()

if __name__ == '__main__':
    try:
        #chama a função para publicar imagens da câmera 1
        camera1_publisher()
    except rospy.ROSInterruptException:
        pass
