# This Python file uses the following encoding: utf-8
import sys, logging
from time import sleep
from PyQt5 import QtWidgets, QtGui
from PyQt5 import uic
import subprocess, re

logfile = 'gui.log'
open(logfile, 'w').close()
logging.basicConfig(filename=logfile, 
                    filemode='a', 
                    format='%(name)s - %(levelname)s - %(message)s',
                    level=logging.DEBUG)

class QTextEditLogger(logging.Handler):
    def __init__(self, parent):
        super().__init__()
        self.widget = QtWidgets.QPlainTextEdit(parent)
        self.widget.setReadOnly(True)

    def emit(self, record):
        msg = self.format(record)
        self.widget.appendPlainText(msg)

class RoboSimITWindow(QtWidgets.QMainWindow):
    def __init__(self):
        super().__init__()
        uic.loadUi("robosimit.ui", self)
        self.logTextBox = QTextEditLogger(self)
        # You can format what is printed to text box
        self.logTextBox.setFormatter(logging.Formatter('%(asctime)s - %(name)s - %(levelname)s - %(message)s'))
        logging.getLogger().addHandler(self.logTextBox)
        # You can control the logging level
        logging.getLogger().setLevel(logging.DEBUG)
        self.logTextBox.widget.setGeometry(10, 550, 1000, 200)
        self.dockerready = False
        self.imageready = False
        self.containerRunning = False
        self.pushButton.clicked.connect(self.clickme)
        self.prechecks()
        self.check_container()
    
    def clickme(self):
        logging.info("Trying to start a simulation container.")
        command = ['/bin/sh','sc.sh']
        logging.debug(command)
        running = subprocess.run(command, check=False, stderr=subprocess.STDOUT)
        sleep(5)


    def prechecks(self):
        command = ['/usr/bin/docker', 'info']
        logging.debug(command)
        res = subprocess.check_output(command)
        string = res.decode("utf-8")
        logging.debug(string)
        regex = re.compile('Server Version.*\n')
        m = regex.search(string)
        if m is not None:
            self.dockerready = True
            self.DockerReadyResultLabel.setText('Ready ☑️' + '\n' + m.group(0))
            logging.debug(m.group(0))
        
        command = ['docker', 'images', 'valu3s:robosimit']
        res = subprocess.check_output(command)
        string = res.decode("utf-8")
        print(string)
        regex = re.compile('valu3s')
        m = regex.search(string)
        if m is not None:
            self.imageready = True
            self.ContainerImageReadyResultLabel.setText('Ready ☑️ [values:robosimit]')

    def check_container(self):
        command = ['/usr/bin/docker', 'ps', '|',  '/usr/bin/grep', 'valu3s:robosimit']
        logging.debug(command)
        try:
            res = subprocess.check_output(command)
            string = res.decode("utf-8")
            logging.debug(string)
            self.containerRunning = True
        except:
            self.containerRunning = False


if __name__ == "__main__":
    app = QtWidgets.QApplication([])
    window = RoboSimITWindow()
    window.show()
    sys.exit(app.exec_())
