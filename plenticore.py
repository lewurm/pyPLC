import pymodbus.client as ModbusClient
from pymodbus.payload import BinaryPayloadDecoder
from pymodbus.constants import Endian

# pymodbus_apply_logging_config("DEBUG")

# framer=FramerType.SOCKET
client = ModbusClient.ModbusTcpClient('192.168.44.70', port = "1502")
# print("connect to inverter...")
client.connect()

reg_dump_count = 0
firstRun = True

def clear_line():
    global reg_dump_count
    reg_dump_count += 1
    if not firstRun:
        print("                                                                          ")
        print("")
        print('\033[3A')

def reg_sint16(reg, desc, unit=''):
    clear_line()
    rr = client.read_holding_registers(reg, count=2, slave=71)
    decoder = BinaryPayloadDecoder.fromRegisters(rr.registers, byteorder=Endian.BIG, wordorder=Endian.LITTLE)
    value_sint16= decoder.decode_16bit_int()
    print(f"({reg:4d}) {desc}: {value_sint16} {unit}")

def reg_uint16(reg, desc, unit=''):
    clear_line()
    rr = client.read_holding_registers(reg, count=2, slave=71)
    decoder = BinaryPayloadDecoder.fromRegisters(rr.registers, byteorder=Endian.BIG, wordorder=Endian.LITTLE)
    value_uint16= decoder.decode_16bit_uint()
    print(f"({reg:4d}) {desc}: {value_uint16} {unit}")

def reg_uint32(reg, desc, unit=''):
    clear_line()
    rr = client.read_holding_registers(reg, count=2, slave=71)
    decoder = BinaryPayloadDecoder.fromRegisters(rr.registers, byteorder=Endian.BIG, wordorder=Endian.LITTLE)
    value_uint32= decoder.decode_32bit_uint()
    print(f"({reg:4d}) {desc}: {value_uint32} {unit}")

def reg_float32(reg, desc, unit=''):
    clear_line()
    rr = client.read_holding_registers(reg, count=2, slave=71)
    decoder = BinaryPayloadDecoder.fromRegisters(rr.registers, byteorder=Endian.BIG, wordorder=Endian.LITTLE)
    value_float32 = decoder.decode_32bit_float()
    print(f"({reg:4d}) {desc}: {value_float32:.1f} {unit}")

while True:
    reg_dump_count = 0
    reg_uint32 ( 56,  'Inverter state                           ', '')
    reg_float32(120,  'Isolation resistance                     ', 'Ohm')
    reg_float32(190,  'Battery charge current                   ', 'A')
    reg_float32(194,  'Number of battery cycles                 ', '')
    reg_float32(200,  'Actual battery (dis)charge current       ', 'A')
    reg_float32(208,  'Battery ready flag                       ', '')
    reg_float32(210,  'Act. state of charge                     ', '%')
    reg_float32(214,  'Battery temperature                      ', '°C')
    reg_float32(216,  'Battery voltage                          ', 'V')
    reg_float32(278,  'Current DC3                              ', 'A')
    reg_float32(280,  'Power   DC3                              ', 'W')
    reg_float32(286,  'Voltage DC3                              ', 'V')
    reg_uint16 (512,  'Battery gross capacity (uint16)          ', 'Ah')
    reg_uint32 (512,  'Battery gross capacity (uint32)          ', 'Ah')
    reg_float32(512,  'Battery gross capacity (float32)         ', 'Ah')
    reg_float32(514,  'Battery actual SOC                       ', '%')
    reg_uint32 (525,  'Battery Model ID                         ', '')
    reg_uint32 (527,  'Battery Serial Number                    ', '')
    reg_uint32 (529,  'Work Capacity                            ', 'Wh')
    reg_sint16 (582,  'Actual battery (dis)charge power         ', 'W')

    reg_float32(1032, 'Battery charge current (DC) setpoint, abs', 'A')
    reg_float32(1034, 'Battery charge power (DC) setpoint, abs  ', 'W')
    reg_float32(1036, 'Battery charge power (DC) setpoint, rel  ', '%')
    reg_float32(1038, 'Battery max. charge power limit, abs     ', 'W')
    reg_float32(1040, 'Battery max. discharge power limit, abs  ', 'W')
    reg_float32(1076, 'Maximum charge power limit (from bat)    ', 'W')
    reg_float32(1078, 'Maximum discharge power limit (from bat) ', 'W')
    reg_float32(1042, 'Minimum SOC                              ', '%')
    reg_float32(1044, 'Maximum SOC                              ', '%')
    reg_float32(1068, 'Battery work capacity                    ', 'Wh')

    firstRun = False

    while reg_dump_count > 0:
        reg_dump_count -= 1
        print('\033[3A')

client.close()
