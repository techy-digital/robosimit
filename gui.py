# This Python file uses the following encoding: utf-8
import sys, logging
from time import sleep
from PyQt5 import QtWidgets, QtGui, uic
from PyQt5.QtCore import QThread, pyqtSignal, QObject
import subprocess, re

logfile = 'gui.log'
open(logfile, 'w').close()
logging.basicConfig(filename=logfile, 
                    filemode='a', 
                    format='%(name)s - %(levelname)s - %(message)s',
                    level=logging.DEBUG)

class Worker(QObject):
    finished = pyqtSignal()
    progress = pyqtSignal(int)
    def run(self):
        running = subprocess.run("/bin/sh start_container.sh", stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, check=False, shell=True)
        self.finished.emit()

class CheckContainer(QObject):
    containerRunning = pyqtSignal(bool)
    command = ['/usr/bin/docker', 'ps', '--filter', 'label=etiket=valu3s']
    containerID = pyqtSignal(str)
    def run(self):
        while True:
            try:
                res = subprocess.check_output(self.command)
                string = res.decode("utf-8")
                regex = re.compile('([a-z0-9]{12})')
                m = regex.search(string)
                if m is not None:
                    containerID = m.group(0)
                    self.containerRunning.emit(True)
                    self.containerID.emit(containerID)
                else:
                    self.containerRunning.emit(False)
                    self.containerID.emit("")
            except:
                self.containerRunning.emit(False)
            sleep(5)
    
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
        self.runningContainerID = ""
        self.startContainerButton.clicked.connect(self.startContainer)
        self.stopContainerButton.clicked.connect(self.stopContainer)
        self.prechecks()
        self.startChecker()

    def updateContainerRunning(self, isRunning):
        if isRunning:
            if self.runningContainerID:
                self.containerRunning = True
                self.ContainerRunningLabel.setText('Container Running, ID: ' + self.runningContainerID)
            else:
                self.containerRunning = False
                self.ContainerRunningLabel.setText('Container Running, ID: Unknown')
        else:
            self.containerRunning = False
            self.ContainerRunningLabel.setText('Container Not Running!')
    
    def updateContainerID(self, ID):
        self.runningContainerID = ID

    def startContainer(self):
        logging.info("Trying to start a simulation container.")
        self.runthread = QThread()
        self.worker = Worker()
        self.worker.moveToThread(self.runthread)
        self.runthread.started.connect(self.worker.run)
        self.worker.finished.connect(self.runthread.quit)
        self.worker.finished.connect(self.worker.deleteLater)
        self.runthread.finished.connect(self.runthread.deleteLater)
        self.runthread.start()

    def startChecker(self):
        logging.info("Starting the container checker thread.")
        self.checkthread = QThread()
        self.checkcontainerworker = CheckContainer()
        self.checkcontainerworker.moveToThread(self.checkthread)
        self.checkthread.started.connect(self.checkcontainerworker.run)
        #self.checkcontainerworker.finished.connect(self.checkthread.quit)
        #self.checkcontainerworker.finished.connect(self.checkcontainerworker.deleteLater)
        #self.checkthread.finished.connect(self.checkthread.deleteLater)
        self.checkcontainerworker.containerRunning.connect(self.updateContainerRunning)
        self.checkcontainerworker.containerID.connect(self.updateContainerID)
        self.checkthread.start()
         
    def stopContainer(self):
        if not self.containerRunning:
            logging.info("No container is running!")
        else:
            if self.runningContainerID:
                logging.info("Trying to stop a running container with ID: " + self.runningContainerID)
                command = ['/usr/bin/docker', 'stop', self.runningContainerID]
                res = subprocess.check_output(command)
                self.runthread.quit()
                self.runningContainerID = ""
            else:
                logging.info("Trying to stop a running container with no ID")

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
        regex = re.compile('valu3s')
        m = regex.search(string)
        if m is not None:
            self.imageready = True
            self.ContainerImageReadyResultLabel.setText('Ready ☑️ [values:robosimit]')

if __name__ == "__main__":
    app = QtWidgets.QApplication([])
    window = RoboSimITWindow()
    window.show()
    sys.exit(app.exec_())