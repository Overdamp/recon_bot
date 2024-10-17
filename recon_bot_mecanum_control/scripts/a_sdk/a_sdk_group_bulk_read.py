#!/usr/bin/env python
# -*- coding: utf-8 -*-

################################################################################
# Copyright 2017 ROBOTIS CO., LTD.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
################################################################################

# Author: Ryu Woon Jung (Leon)

#!/usr/bin/env python
# -*- coding: utf-8 -*-

################################################################################
# Copyright 2017 ROBOTIS CO., LTD.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
################################################################################

# Author: Ryu Woon Jung (Leon)

BROADCAST_ID = 0xFE  # 254
MAX_ID = 0xFC  # 252

# Instruction for DXL Protocol
INST_PING = 1
INST_READ = 2
INST_WRITE = 3
INST_REG_WRITE = 4
INST_ACTION = 5
INST_FACTORY_RESET = 6
INST_CLEAR = 16
INST_SYNC_WRITE = 131  # 0x83
INST_BULK_READ = 146  # 0x92
# --- Only for 2.0 ---
INST_REBOOT = 8
INST_STATUS = 85  # 0x55
INST_SYNC_READ = 130  # 0x82
INST_BULK_WRITE = 147  # 0x93

# Communication Result
COMM_SUCCESS = 0  # tx or rx packet communication success
COMM_PORT_BUSY = -1000  # Port is busy (in use)
COMM_TX_FAIL = -1001  # Failed transmit instruction packet
COMM_RX_FAIL = -1002  # Failed get status packet
COMM_TX_ERROR = -2000  # Incorrect instruction packet
COMM_RX_WAITING = -3000  # Now recieving status packet
COMM_RX_TIMEOUT = -3001  # There is no status packet
COMM_RX_CORRUPT = -3002  # Incorrect status packet
COMM_NOT_AVAILABLE = -9000  #


# Macro for Control Table Value
def DXL_MAKEWORD(a, b):
    return (a & 0xFF) | ((b & 0xFF) << 8)


def DXL_MAKEDWORD(a, b):
    return (a & 0xFFFF) | (b & 0xFFFF) << 16


def DXL_LOWORD(l):
    return l & 0xFFFF


def DXL_HIWORD(l):
    return (l >> 16) & 0xFFFF


def DXL_LOBYTE(w):
    return w & 0xFF


def DXL_HIBYTE(w):
    return (w >> 8) & 0xFF


PARAM_NUM_DATA = 0
PARAM_NUM_ADDRESS = 1
PARAM_NUM_LENGTH = 2


class GroupBulkRead:
    def __init__(self, port, ph):
        self.port = port
        self.ph = ph

        self.last_result = False
        self.is_param_changed = False
        self.param = []
        self.data_dict = {}

        self.clearParam()

    def makeParam(self):
        if not self.data_dict:
            return

        self.param = []

        for dxl_id in self.data_dict:
            if self.ph.getProtocolVersion() == 1.0:
                self.param.append(self.data_dict[dxl_id][2])  # LEN
                self.param.append(dxl_id)  # ID
                self.param.append(self.data_dict[dxl_id][1])  # ADDR
            else:
                self.param.append(dxl_id)  # ID
                self.param.append(DXL_LOBYTE(self.data_dict[dxl_id][1]))  # ADDR_L
                self.param.append(DXL_HIBYTE(self.data_dict[dxl_id][1]))  # ADDR_H
                self.param.append(DXL_LOBYTE(self.data_dict[dxl_id][2]))  # LEN_L
                self.param.append(DXL_HIBYTE(self.data_dict[dxl_id][2]))  # LEN_H

    def addParam(self, dxl_id, start_address, data_length):
        if dxl_id in self.data_dict:  # dxl_id already exist
            return False

        data = []  # [0] * data_length
        self.data_dict[dxl_id] = [data, start_address, data_length]

        self.is_param_changed = True
        return True

    def removeParam(self, dxl_id):
        if dxl_id not in self.data_dict:  # NOT exist
            return

        del self.data_dict[dxl_id]

        self.is_param_changed = True

    def clearParam(self):
        self.data_dict.clear()
        return

    def txPacket(self):
        if len(self.data_dict.keys()) == 0:
            return COMM_NOT_AVAILABLE

        if self.is_param_changed is True or not self.param:
            self.makeParam()

        if self.ph.getProtocolVersion() == 1.0:
            return self.ph.bulkReadTx(self.port, self.param, len(self.data_dict.keys()) * 3)
        else:
            return self.ph.bulkReadTx(self.port, self.param, len(self.data_dict.keys()) * 5)

    def rxPacket(self):
        self.last_result = False

        result = COMM_RX_FAIL

        if len(self.data_dict.keys()) == 0:
            return COMM_NOT_AVAILABLE

        for dxl_id in self.data_dict:
            self.data_dict[dxl_id][PARAM_NUM_DATA], result, _ = self.ph.readRx(self.port, dxl_id,
                                                                               self.data_dict[dxl_id][PARAM_NUM_LENGTH])
            if result != COMM_SUCCESS:
                return result

        if result == COMM_SUCCESS:
            self.last_result = True

        return result

    def txRxPacket(self):
        result = self.txPacket()
        if result != COMM_SUCCESS:
            return result

        return self.rxPacket()

    def isAvailable(self, dxl_id, address, data_length):
        if self.last_result is False or dxl_id not in self.data_dict:
            return False

        start_addr = self.data_dict[dxl_id][PARAM_NUM_ADDRESS]

        if (address < start_addr) or (start_addr + self.data_dict[dxl_id][PARAM_NUM_LENGTH] - data_length < address):
            return False

        return True

    def getData(self, dxl_id, address, data_length):
        if not self.isAvailable(dxl_id, address, data_length):
            return 0

        start_addr = self.data_dict[dxl_id][PARAM_NUM_ADDRESS]

        if data_length == 1:
            return self.data_dict[dxl_id][PARAM_NUM_DATA][address - start_addr]
        elif data_length == 2:
            return DXL_MAKEWORD(self.data_dict[dxl_id][PARAM_NUM_DATA][address - start_addr],
                                self.data_dict[dxl_id][PARAM_NUM_DATA][address - start_addr + 1])
        elif data_length == 4:
            return DXL_MAKEDWORD(DXL_MAKEWORD(self.data_dict[dxl_id][PARAM_NUM_DATA][address - start_addr + 0],
                                              self.data_dict[dxl_id][PARAM_NUM_DATA][address - start_addr + 1]),
                                 DXL_MAKEWORD(self.data_dict[dxl_id][PARAM_NUM_DATA][address - start_addr + 2],
                                              self.data_dict[dxl_id][PARAM_NUM_DATA][address - start_addr + 3]))
        else:
            return 0
