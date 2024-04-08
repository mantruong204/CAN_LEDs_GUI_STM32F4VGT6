from operator import mod
from typing import *

class compressData:
    def __init__(self) -> None:
        self.start_flag = 0x02
        self.end_flag = 0x03
        self.cmd_ONLD = bytearray([0x4F, 0x4E, 0x4C, 0x44])
        self.cmd_OFLD = bytearray([0x4F, 0x46, 0x4C, 0x44])
        self.opt_A = 0x0A
        self.data1 = 0x00
        self.sync = 0x16
        '''
        Transmit data frame:
        -------------------------------------------------------------------------------------------------------
        |  STX(1) |           CMD(4)         |         OPT(1)         |    DATA(2)   |  SYNC/ACK(1) |  ETX(1) |
        |  0x02   |  0x4F  0x4E  0x4C  0x44  |  0x0A or 0x0B or 0x0C  |  0xXX  0x00  |     0x16     |   0x03  |
        -------------------------------------------------------------------------------------------------------
        '''
    @staticmethod
    def compress_data_A(led: int) -> bytearray:
        transmit_data: List[int] = [compressData().start_flag, compressData().cmd_ONLD[0], compressData().cmd_ONLD[1], compressData().cmd_ONLD[2], compressData().cmd_ONLD[3], compressData().opt_A]
        transmit_data.append(led)
        transmit_data.append(0x00)
        transmit_data.append(compressData().sync)
        transmit_data.append(compressData().end_flag)
        return bytearray(transmit_data)