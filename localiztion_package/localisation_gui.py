import sys
import subprocess
from PyQt5.QtWidgets import QApplication, QWidget, QVBoxLayout, QPushButton, QMessageBox

class ROSCommandGUI(QWidget):
    def __init__(self):
        super().__init__()

        self.initUI()

    def initUI(self):
        layout = QVBoxLayout()

        btn0 = QPushButton('Run roscore', self)
        btn0.clicked.connect(self.run_roscore)
        layout.addWidget(btn0)

        btn1 = QPushButton('Launch Ouster Sensor', self)
        btn1.clicked.connect(self.launch_ouster_sensor)
        layout.addWidget(btn1)

        btn2 = QPushButton('Launch Direct Lidar Odometry', self)
        btn2.clicked.connect(self.launch_dlo)
        layout.addWidget(btn2)

        btn3 = QPushButton('Save PCD Map', self)
        btn3.clicked.connect(self.save_pcd_map)
        layout.addWidget(btn3)

        btn4 = QPushButton('Run HW5 Node', self)
        btn4.clicked.connect(self.run_hw5_node)
        layout.addWidget(btn4)
        
        btn5 = QPushButton('Launch RViz', self)
        btn5.clicked.connect(self.launch_rviz)
        layout.addWidget(btn5)

        self.setLayout(layout)
        self.setWindowTitle('ROS Command GUI')
        self.show()

    def run_roscore(self):
        self.run_command('roscore')

    def launch_ouster_sensor(self):
        self.run_command('roslaunch ouster_ros sensor.launch sensor_hostname:=os-122125000885.local')

    def launch_dlo(self):
        self.run_command('roslaunch direct_lidar_odometry dlo.launch')

    def save_pcd_map(self):
        self.run_command('rosservice call /robot/dlo_map/save_pcd 0.05 /home/ranjitray/ws/src')

    def run_hw5_node(self):
        self.run_command('rosrun hw5_0751081 hw5_0751081_node')
        
    def launch_rviz(self):
        self.run_command('rviz')

    def run_command(self, command):
        try:
            subprocess.Popen(['gnome-terminal', '--', 'bash', '-c', f'source ~/ws/devel/setup.bash && {command}; exec bash'])
            QMessageBox.information(self, 'Success', f'Command executed: {command}')
        except Exception as e:
            QMessageBox.critical(self, 'Error', f'Failed to execute command: {command}\nError: {str(e)}')

if __name__ == '__main__':
    app = QApplication(sys.argv)
    ex = ROSCommandGUI()
    sys.exit(app.exec_())
