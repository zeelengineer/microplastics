import board
import canio

can = canio.CAN(rx=board.RX, tx=board.TX, baudrate=250_000, auto_restart=True)

# Send a message
message = canio.Message(id=0x123, data=b"Hello CAN")
can.send(message)

# Receive a message
reader = can.listen(timeout=1.0)
message = reader.receive()
if message is None:
    print("No message received")
else:
    print(f"Received message with id {message.id} and data {message.data}")
