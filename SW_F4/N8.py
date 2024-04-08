from PyQt5 import QtCore, QtGui, QtWidgets
from PyQt5.QtGui import QTextCursor
from PyQt5.QtWidgets import QApplication, QTextEdit, QVBoxLayout, QWidget
from PyQt5.QtCore import Qt
from PyQt5.QtWidgets import (
    QAbstractButton,
    QAbstractItemView,
    QHeaderView,
    QMessageBox,
    QRadioButton,
    QFrame,
    QTableWidgetItem,
    QTextEdit,
    QApplication
)
from typing import *
from enum import Enum
from main_window import *
from compress_data import *
from serial import Serial
import serial
import sys
import string
import threading
import binascii

class positionLed(Enum):
    led_1 = 0
    led_2 = 1
    led_3 = 2
    led_4 = 3

class buildTable(Ui_MainWindow):
    def __int__(self) -> None:
        super().__init__()

    def init_slave1_table(self) -> None:
        self.slave1_table.setEditTriggers(QAbstractItemView.NoEditTriggers)
        self.slave1_table.setSelectionMode(QAbstractItemView.NoSelection)
        # Set default column witdh
        self.slave1_table.setColumnWidth(0, 195)
        self.slave1_table.setColumnWidth(1, 195)
        self.slave1_table.setColumnWidth(2, 296)
        self.slave1_table.horizontalHeader().setSectionResizeMode(QHeaderView.Fixed)
        # Add row
        self.slave1_table.verticalHeader().setSectionsClickable(False)
        self.slave1_table.verticalHeader().setDefaultSectionSize(15)
        self.slave1_table.verticalHeader().setVisible(False)

    def init_slave2_table(self) -> None:
        self.slave2_table.setEditTriggers(QAbstractItemView.NoEditTriggers)
        self.slave2_table.setSelectionMode(QAbstractItemView.NoSelection)
        # Set default column witdh
        self.slave2_table.setColumnWidth(0, 195)
        self.slave2_table.setColumnWidth(1, 195)
        self.slave2_table.setColumnWidth(2, 296)
        self.slave2_table.horizontalHeader().setSectionResizeMode(QHeaderView.Fixed)
        # Add row
        self.slave2_table.verticalHeader().setSectionsClickable(False)
        self.slave2_table.verticalHeader().setDefaultSectionSize(15)
        self.slave2_table.verticalHeader().setVisible(False)

class GUI(buildTable, compressData):
    def __init__(self, MainWindow) -> None:
        super(GUI, self).__init__()
        self.setupUi(MainWindow)
        self.init_variable()
        self.init_default_config_window()
        self.init_callback()
        self.init_timer()
        self.init_slave1_table()
        self.init_slave2_table()

    @staticmethod
    def set_bit(data, value, bit):
        return data | (value << bit)

    def init_variable(self) -> None:
        self.com_connect_status = False

        # Status of device
        self.status_CAN_connect = False

        # Timer call for loop process
        self.loop_RX = 100
        self.loop_TX = 600

        self.TX_Flag = False
        self.Rx_cnt = 0
        self.Rx_TIM_cnt = 0

        self.RX_alert = False

        self.max_lines = 10
        self.cnt_fr = 0
        self.cnt_data = 0

    def init_timer(self) -> None:
        self.TIM_RX = QtCore.QTimer()
        self.TIM_RX.setInterval(self.loop_RX)
        self.TIM_RX.timeout.connect(self.RX_Timeout_Handler)

        self.TIM_TX = QtCore.QTimer()
        self.TIM_TX.setInterval(self.loop_TX)
        self.TIM_TX.timeout.connect(self.TX_Timeout_Handler)

        self.TIM_check = QtCore.QTimer()
        self.TIM_check.timeout.connect(self.auto_clear)
        self.TIM_check.start(500)

    def init_default_config_window(self) -> None:
        # Connect button
        self.connect_button.setText("CONNECT")
        self.connect_button.setStyleSheet("QPushButton {color: green;}")

        # Radio button default select
        self.led1_off.setChecked(True)
        self.led2_off.setChecked(True)
        self.led3_off.setChecked(True)
        self.led4_off.setChecked(True)

        # Status button
        self.status_button.setStyleSheet("background-color: white; color: black;")

        # Default value
        validator = QtGui.QIntValidator(0, 5000)
        self.adc_value.setText("0")
        self.adc_value.setValidator(validator)

        # Generate 20 COM in com_select
        for x in range(20):
            com_name = "COM" + str(x + 1)
            self.com_select.addItem(com_name)

    def init_callback(self) -> None:
        # Callback connect button
        self.connect_button.clicked.connect(self.connect_button_callback)

        # Callback set_leds_button
        self.set_leds_button.clicked.connect(self.set_leds_button_callback)

    def connect_button_callback(self) -> None:
        NameCOM = self.com_select.currentText()
        try:
            if self.com_connect_status == False:
                self.connect_com(NameCOM)
            else:
                self.disconnect_com()
        except IOError:
            status_bar = "Serial port " + NameCOM
            if self.com_connect_status == False:
                status_bar += " Error when open port !"
            else:
                status_bar += " Error when close port !"
            self.statusbar.showMessage(status_bar)

    def connect_com(self, com) -> None:
        self.transmit = serial.Serial(com, 115200, timeout=2.5)
        self.com_select.setEnabled(False)
        self.connect_button.setText("DISCONNECT")
        self.connect_button.setStyleSheet("QPushButton {color: red;}")
        self.com_connect_status = True
        self.com_choice = com

        status_bar = "Serial port " + self.com_choice + " opened"
        self.statusbar.showMessage(status_bar)

        self.receive_data_thread = threading.Thread(target=self.receive_uart_data_thread, daemon=True)
        self.receive_data_thread.start()

        # start Timer for Rx unplugged handler
        self.TIM_RX.start()

    def disconnect_com(self) -> None:
        self.com_select.setEnabled(True)
        self.transmit.close()
        self.connect_button.setText("CONNECT")
        self.connect_button.setStyleSheet("QPushButton {color: green;}")
        self.com_connect_status = False

        status_bar = "Serial port " + self.com_choice + " closed"

        self.statusbar.showMessage(status_bar)

    def receive_uart_data_thread(self):
        while self.com_connect_status:

            bytetoread = []
            try:
                bytetoread = self.transmit.inWaiting()
                if bytetoread > 0:
                    data = self.transmit.read(10)
                    if data:
                        if self.Rx_TIM_cnt <= 6 and self.Rx_TIM_cnt >= 0:
                            self.Rx_cnt += 1
                        hex_data = binascii.hexlify(data).decode('utf-8')
                        RxFullFrame = hex_data.upper()
                        formatted_RxFullFrame = f'<font color="red">{RxFullFrame}</font>'
                        STX = RxFullFrame[0:2]
                        CMD = RxFullFrame[2:10]
                        OPT = RxFullFrame[10:12]
                        DATA = RxFullFrame[12:16]
                        ACK = RxFullFrame[16:18]
                        ETX = RxFullFrame[18:20]
                        self.receive_uart_data(formatted_RxFullFrame)
                        ''' 
                        data_frame:
                        024F4E4C440A03001603
                        02564144430B0FFF0603
                        025242544E0C01000603
                        '''
                        if OPT == "0A":
                            self.TX_Flag = False
                        elif OPT == "0B":
                            self.view_adc_value(DATA)
                            action_1: str = "Transmit"
                            row_position = self.slave1_table.rowCount()
                            self.slave1_table.insertRow(row_position)

                            opt_item1 = QtWidgets.QTableWidgetItem(OPT)
                            opt_item1.setTextAlignment(Qt.AlignCenter)

                            data_item1 = QtWidgets.QTableWidgetItem(DATA)
                            data_item1.setTextAlignment(Qt.AlignCenter)

                            action_item1 = QtWidgets.QTableWidgetItem(action_1)
                            action_item1.setTextAlignment(Qt.AlignCenter)

                            self.slave1_table.setItem(row_position, 0, QtWidgets.QTableWidgetItem(opt_item1))
                            self.slave1_table.setItem(row_position, 1, QtWidgets.QTableWidgetItem(data_item1))
                            self.slave1_table.setItem(row_position, 2, QtWidgets.QTableWidgetItem(action_item1))

                            self.slave1_table.scrollToBottom()

                            action_2: str = "Receive"
                            row_position = self.slave2_table.rowCount()
                            self.slave2_table.insertRow(row_position)

                            opt_item2 = QtWidgets.QTableWidgetItem(OPT)
                            opt_item2.setTextAlignment(Qt.AlignCenter)

                            data_item2 = QtWidgets.QTableWidgetItem(DATA)
                            data_item2.setTextAlignment(Qt.AlignCenter)

                            action_item2 = QtWidgets.QTableWidgetItem(action_2)
                            action_item2.setTextAlignment(Qt.AlignCenter)

                            self.slave2_table.setItem(row_position, 0, QtWidgets.QTableWidgetItem(opt_item2))
                            self.slave2_table.setItem(row_position, 1, QtWidgets.QTableWidgetItem(data_item2))
                            self.slave2_table.setItem(row_position, 2, QtWidgets.QTableWidgetItem(action_item2))

                            self.slave2_table.scrollToBottom()
                        elif OPT == "0C":
                            if DATA == "0100":
                                self.status_button.setStyleSheet("background-color: red; color: black;")
                            else:
                                self.status_button.setStyleSheet("background-color: white; color: black;")
                            action: str = "Transmit"
                            row_position = self.slave2_table.rowCount()
                            self.slave2_table.insertRow(row_position)

                            opt_item = QtWidgets.QTableWidgetItem(OPT)
                            opt_item.setTextAlignment(Qt.AlignCenter)

                            data_item = QtWidgets.QTableWidgetItem(DATA)
                            data_item.setTextAlignment(Qt.AlignCenter)

                            action_item = QtWidgets.QTableWidgetItem(action)
                            action_item.setTextAlignment(Qt.AlignCenter)

                            self.slave2_table.setItem(row_position, 0, QtWidgets.QTableWidgetItem(opt_item))
                            self.slave2_table.setItem(row_position, 1, QtWidgets.QTableWidgetItem(data_item))
                            self.slave2_table.setItem(row_position, 2, QtWidgets.QTableWidgetItem(action_item))

                            self.slave2_table.scrollToBottom()
            except IOError:
                self.disconnect_com()
                self.statusbar.showMessage("COM unplugged !")

    def receive_uart_data(self, data):
        self.uart_frame.append(data)
        self.cnt_fr += 1
        self.cnt_data += 1

    def auto_clear(self):
        if self.cnt_fr > self.max_lines and self.cnt_data < 100:
            cursor = self.uart_frame.textCursor()
            cursor.movePosition(QTextCursor.Start)
            cursor.movePosition(QTextCursor.Down, QTextCursor.KeepAnchor, 10)
            cursor.removeSelectedText()
            self.cnt_fr = 0

            for i in range(10):
                self.slave1_table.removeRow(i)
                self.slave2_table.removeRow(i)

        elif self.cnt_data > 100:
            cursor = self.uart_frame.textCursor()
            cursor.movePosition(QTextCursor.Start)
            cursor.movePosition(QTextCursor.Down, QTextCursor.KeepAnchor, 10)
            cursor.removeSelectedText()
            self.cnt_data = 0

            for i in range(10):
                self.slave1_table.removeRow(i)
                self.slave2_table.removeRow(i)

    def view_adc_value(self, adc_data):
        adc_val = int(adc_data, 16)
        adc_value = int(adc_val * 3000 / 4095)
        self.adc_value.setText(str(adc_value))

    def get_led_value(self, led_builtin: QRadioButton) -> int:
        if led_builtin.isChecked() == True:
            return 1
        else:
            return 0

    def RX_Timeout_Handler(self):
        if self.TIM_RX.isActive():
            self.TIM_RX.stop()

        self.Rx_TIM_cnt += 1

        if self.Rx_TIM_cnt == 6:
            if self.Rx_cnt < 2 and self.com_connect_status == True:
                # RX unplugged
                self.statusbar.showMessage("RX unplugged !")
                self.RX_alert = True
            else:
                if self.RX_alert == True and self.com_connect_status == True:
                    self.statusbar.showMessage("Rx re-plugged", msecs=2000)
                    self.RX_alert = False

            self.Rx_cnt = 0
            self.Rx_TIM_cnt = 0

        if not self.TIM_RX.isActive():
            self.TIM_RX.start()

    def TX_Timeout_Handler(self):
        if self.TIM_TX.isActive():
            self.TIM_TX.stop()
        if self.TX_Flag == True:
            # TX unplugged
            self.statusbar.showMessage("TX unplugged !", msecs=5000)
            self.TX_Flag = False

    def set_leds_button_callback(self):
        data: bytearray = bytearray()
        led_static = 0
        led_static = GUI.set_bit(led_static, self.get_led_value(self.led1_on), positionLed.led_1.value)
        led_static = GUI.set_bit(led_static, self.get_led_value(self.led2_on), positionLed.led_2.value)
        led_static = GUI.set_bit(led_static, self.get_led_value(self.led3_on), positionLed.led_3.value)
        led_static = GUI.set_bit(led_static, self.get_led_value(self.led4_on), positionLed.led_4.value)
        data = compressData.compress_data_A(led_static)

        OPT = "0A"
        led_data = hex(led_static)[2:].upper()
        data_led = "0" + led_data + "00"
        action: str = "Receive"
        row_position = self.slave1_table.rowCount()
        self.slave1_table.insertRow(row_position)

        opt_item = QtWidgets.QTableWidgetItem(OPT)
        opt_item.setTextAlignment(Qt.AlignCenter)

        data_item = QtWidgets.QTableWidgetItem(data_led)
        data_item.setTextAlignment(Qt.AlignCenter)

        action_item = QtWidgets.QTableWidgetItem(action)
        action_item.setTextAlignment(Qt.AlignCenter)

        self.slave1_table.setItem(row_position, 0, QtWidgets.QTableWidgetItem(opt_item))
        self.slave1_table.setItem(row_position, 1, QtWidgets.QTableWidgetItem(data_item))
        self.slave1_table.setItem(row_position, 2, QtWidgets.QTableWidgetItem(action_item))
        self.slave1_table.scrollToBottom()

        if self.com_connect_status == True:
            self.transmit.write(data)
            """
            # Turn on flag
            # Start Timer for Tx unplugged handler
            """
            self.TX_Flag = True
            self.TIM_TX.start()

            data_led_item = binascii.hexlify(data).decode('utf-8').upper()
            formatted_data_led = f'<font color="green">{data_led_item}</font>'
            self.uart_frame.append(formatted_data_led)
        else:
            status_bar = "COM is not connected"
            self.statusbar.showMessage(status_bar)

def UIbuild():
    app = QtWidgets.QApplication(sys.argv)
    MainWindow = QtWidgets.QMainWindow()
    ui = GUI(MainWindow)
    MainWindow.show()
    sys.exit(app.exec_())

if __name__ == "__main__":
    UIbuild()