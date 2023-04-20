import encoders_driver_v2 as encdrv

if __name__ == "__main__":
    encoddrv = encdrv.EncoderIO()
        
    while True:
        encoddrv.get_sync()
        while True:
            sync,data_encoders = encoddrv.read_packet(debug=True)
            if not sync:
                break

