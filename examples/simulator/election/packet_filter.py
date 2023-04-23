counted_addrs = [
    b'\xff\x02\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\xee\x00\xda',
    b'\xfc\x07\x00\x00\x00\x00\x00\x04\x00\x00\x00\x00\x00\x00\x00\x01',
    b'\xfc\x07\x00\x00\x00\x00\x00\x03\x00\x00\x00\x00\x00\x00\x00\x01',
    b'\xfc\x07\x00\x00\x00\x00\x00\x02\x00\x00\x00\x00\x00\x00\x00\x01',
    b'\xfc\x07\x00\x00\x00\x00\x00\x01\x00\x00\x00\x00\x00\x00\x00\x01',
    b'\xfc\x07\x00\x00\x00\x00\x00\x05\x00\x00\x00\x00\x00\x00\x00\x01',
    b'\xfc\x07\x00\x00\x00\x00\x00\x06\x00\x00\x00\x00\x00\x00\x00\x01',
    b'\xff\x02\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\xea\x00\xea'
]

routing_addrs = [
    b'\xff\x02\x00\x00\x00\x00\x00\x00\x00\n\x00\x0b\x00\x0b\x00\xae'
]

total_counter: int = 0

def filter( packet, sender, receiver ):
    global total_counter, counted_addrs, routing_addrs
    pack_array = bytearray(packet)
    # print(pack_array[24:40]) = The address we want to check
    if pack_array[24:40] in counted_addrs:
        total_counter += 1
        print(total_counter)
        print(f"sender: {sender.module_id}, connector: { sender.connector_idx }")
        print(f"receiver: {receiver.module_id}, connector: { receiver.connector_idx }")

    if pack_array[24:40] in routing_addrs:
        print(f"Routing Message -> sender: {sender.module_id}, connector: { sender.connector_idx }")
        print(f"Routing Message -> receiver: {receiver.module_id}, connector: { receiver.connector_idx }")
    return packet, 0