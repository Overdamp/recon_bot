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



class GroupSyncRead:
    def __init__(self, port, ph, start_address, data_length):
        self.port = port
        self.ph = ph
        self.start_address = start_address
        self.data_length = data_length

        self.last_result = False
        self.is_param_changed = False
        self.param = []
        self.data_dict = {}

        self.clearParam()

    def makeParam(self):
        if self.ph.getProtocolVersion() == 1.0:
            return

        if not self.data_dict:  # len(self.data_dict.keys()) == 0:
            return

        self.param = []

        for dxl_id in self.data_dict:
            self.param.append(dxl_id)

    def addParam(self, dxl_id):
        if self.ph.getProtocolVersion() == 1.0:
            return False

        if dxl_id in self.data_dict:  # dxl_id already exist
            return False

        self.data_dict[dxl_id] = []  # [0] * self.data_length

        self.is_param_changed = True
        return True

    def removeParam(self, dxl_id):
        if self.ph.getProtocolVersion() == 1.0:
            return

        if dxl_id not in self.data_dict:  # NOT exist
            return

        del self.data_dict[dxl_id]

        self.is_param_changed = True

    def clearParam(self):
        if self.ph.getProtocolVersion() == 1.0:
            return

        self.data_dict.clear()

    def txPacket(self):
        if self.ph.getProtocolVersion() == 1.0 or len(self.data_dict.keys()) == 0:
            return COMM_NOT_AVAILABLE

        if self.is_param_changed is True or not self.param:
            self.makeParam()

        return self.ph.syncReadTx(self.port, self.start_address, self.data_length, self.param,
                                  len(self.data_dict.keys()) * 1)

    def rxPacket(self):
        self.last_result = False

        if self.ph.getProtocolVersion() == 1.0:
            return COMM_NOT_AVAILABLE

        result = COMM_RX_FAIL

        if len(self.data_dict.keys()) == 0:
            return COMM_NOT_AVAILABLE

        for dxl_id in self.data_dict:
            self.data_dict[dxl_id], result, _ = self.ph.readRx(self.port, dxl_id, self.data_length)
            if result != COMM_SUCCESS:
                return result

        if result == COMM_SUCCESS:
            self.last_result = True

        return result

    def txRxPacket(self):
        if self.ph.getProtocolVersion() == 1.0:
            return COMM_NOT_AVAILABLE

        result = self.txPacket()
        if result != COMM_SUCCESS:
            return result

        return self.rxPacket()

    def isAvailable(self, dxl_id, address, data_length):
        if self.ph.getProtocolVersion() == 1.0 or self.last_result is False or dxl_id not in self.data_dict:
            return False

        if (address < self.start_address) or (self.start_address + self.data_length - data_length < address):
            return False

        return True

    def getData(self, dxl_id, address, data_length):
        if not self.isAvailable(dxl_id, address, data_length):
            return 0

        if data_length == 1:
            return self.data_dict[dxl_id][address - self.start_address]
        elif data_length == 2:
            return DXL_MAKEWORD(self.data_dict[dxl_id][address - self.start_address],
                                self.data_dict[dxl_id][address - self.start_address + 1])
        elif data_length == 4:
            return DXL_MAKEDWORD(DXL_MAKEWORD(self.data_dict[dxl_id][address - self.start_address + 0],
                                              self.data_dict[dxl_id][address - self.start_address + 1]),
                                 DXL_MAKEWORD(self.data_dict[dxl_id][address - self.start_address + 2],
                                              self.data_dict[dxl_id][address - self.start_address + 3]))
        else:
            return 0
