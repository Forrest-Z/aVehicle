import pyads

plc = pyads.Connection('5.63.79.44.1.1', pyads.PORT_TC3PLC1, '192.168.214.103')
plc.open()

ncounter = plc.read_by_name("MAIN.nCounter",pyads.PLCTYPE_INT)
print('ncounter: {}'.format(ncounter))
