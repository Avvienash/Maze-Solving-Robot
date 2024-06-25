import spidev
import time

def spi_setup():
    print("Initializing SPI")
    spi = spidev.SpiDev()
    spi.open(0, 0)
    spi.max_speed_hz = 40000000
    time.sleep(0.1)
    print("SPI Initialized")
    return spi

def send_data(spi, sign_0, value_0, sign_1, value_1):
    # Constructing 16-bit data
    data = (sign_0 & 0x01) << 7  # Sign 0 (1 bit)
    data |= (sign_1 & 0x01) << 6  # Sign 1 (1 bit)
    data |= (value_0 & 0x3F)  # Value 0 (6 bits)
    data <<= 6  # Shift for Value 1 (6 bits)
    data |= (value_1 & 0x3F)  # Value 1 (6 bits)
    
    # Append 00 as the first two bits
    data <<= 2
    
    # Convert to bytes
    bytes_data = [(data >> 8) & 0xFF, data & 0xFF]
    
    print("Sending:", bytes_data)
    print("data: ", data)
    spi.xfer2(bytes_data)

def main():
    spi = spi_setup()
    send_data(spi, 1, 0, 1, 0)  # Sending values with their signs
    spi.close()

if __name__ == "__main__":
    main()
