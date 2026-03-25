from pymodbus.server.sync import StartTcpServer
from pymodbus.datastore import ModbusSlaveContext, ModbusServerContext
from pymodbus.datastore import ModbusSequentialDataBlock

store = ModbusSlaveContext(
    hr=ModbusSequentialDataBlock(0, [0] * 100)
)

context = ModbusServerContext(slaves=store, single=True)

print("Starting fake Modbus server on 0.0.0.0:502")
StartTcpServer(context, address=("0.0.0.0", 1234))