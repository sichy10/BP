import unittest

# Constants from DallasTemperature.h
DEVICE_DISCONNECTED_C = -127
DEVICE_DISCONNECTED_F = -196.6
DEVICE_DISCONNECTED_RAW = -7040

# Conversion functions from DallasTemperature.c

def dt_to_fahrenheit(celsius: float) -> float:
    return (celsius * 1.8) + 32


def dt_to_celsius(fahrenheit: float) -> float:
    return (fahrenheit - 32) * 0.555555556


def dt_raw_to_celsius(raw: int) -> float:
    if raw <= DEVICE_DISCONNECTED_RAW:
        return DEVICE_DISCONNECTED_C
    return raw * 0.0078125


def dt_raw_to_fahrenheit(raw: int) -> float:
    if raw <= DEVICE_DISCONNECTED_RAW:
        return DEVICE_DISCONNECTED_F
    return (raw * 0.0140625) + 32


def dt_calculate_temperature(device_address, scratch_pad) -> int:
    fp_temperature = ((scratch_pad[1] << 11) | (scratch_pad[0] << 3))
    DS18S20MODEL = 0x10
    COUNT_REMAIN = 6
    COUNT_PER_C = 7
    if device_address[0] == DS18S20MODEL:
        fp_temperature = ((fp_temperature & 0xfff0) << 3) - 16 + (((scratch_pad[COUNT_PER_C] - scratch_pad[COUNT_REMAIN]) << 7) // scratch_pad[COUNT_PER_C])
    return fp_temperature

# CRC functions from OneWire.c
DSCRC_TABLE = [
        0x00, 0x5E, 0xBC, 0xE2, 0x61, 0x3F, 0xDD, 0x83,
        0xC2, 0x9C, 0x7E, 0x20, 0xA3, 0xFD, 0x1F, 0x41,
        0x00, 0x9D, 0x23, 0xBE, 0x46, 0xDB, 0x65, 0xF8,
        0x8C, 0x11, 0xAF, 0x32, 0xCA, 0x57, 0xE9, 0x74
]


def ow_crc8(data) -> int:
    crc = 0
    for b in data:
        crc = b ^ crc
        crc = DSCRC_TABLE[crc & 0x0f] ^ DSCRC_TABLE[16 + ((crc >> 4) & 0x0f)]
    return crc & 0xFF

ODD_PARITY = [0, 1, 1, 0, 1, 0, 0, 1, 1, 0, 0, 1, 0, 1, 1, 0]

def ow_crc16(data, crc=0) -> int:
    for b in data:
        cdata = b
        cdata = (cdata ^ crc) & 0xFF
        crc >>= 8
        if ODD_PARITY[cdata & 0x0F] ^ ODD_PARITY[cdata >> 4]:
            crc ^= 0xC001
        cdata <<= 6
        crc ^= cdata
        cdata <<= 1
        crc ^= cdata
    return crc & 0xFFFF


def ow_check_crc16(data, inverted_crc, crc=0) -> bool:
    crc = (~ow_crc16(data, crc)) & 0xFFFF
    return ((crc & 0xFF) == inverted_crc[0]) and ((crc >> 8) == inverted_crc[1])


class TestConversions(unittest.TestCase):
    def test_temp_conversions(self):
        self.assertAlmostEqual(dt_to_fahrenheit(0), 32)
        self.assertAlmostEqual(dt_to_fahrenheit(100), 212)
        self.assertAlmostEqual(dt_to_celsius(32), 0)
        self.assertAlmostEqual(dt_to_celsius(212), 100, places=6)

    def test_raw_conversions(self):
        self.assertEqual(dt_raw_to_celsius(0), 0)
        self.assertEqual(dt_raw_to_celsius(128), 1)
        self.assertEqual(dt_raw_to_celsius(DEVICE_DISCONNECTED_RAW), DEVICE_DISCONNECTED_C)
        self.assertEqual(dt_raw_to_fahrenheit(0), 32)
        self.assertAlmostEqual(dt_raw_to_fahrenheit(128), 33.8, places=1)
        self.assertEqual(dt_raw_to_fahrenheit(DEVICE_DISCONNECTED_RAW), DEVICE_DISCONNECTED_F)

    def test_calculate_temperature(self):
        # Example DS18B20 scratchpad for 85C
        device = [0x28]
        scratch = [0x50, 0x05, 0, 0, 0, 0, 0, 0, 0]
        raw = dt_calculate_temperature(device, scratch)
        self.assertEqual(raw, 10880)
        self.assertAlmostEqual(dt_raw_to_celsius(raw), 85.0)

    def test_crc8(self):
        data = [0x28, 0xFF, 0x8C, 0x5C, 0x60, 0x16, 0x05]
        self.assertEqual(ow_crc8(data), 0xB4)

    def test_crc16(self):
        data = [1,2,3,4,5,6,7,8,9]
        crc = ow_crc16(data)
        self.assertEqual(crc, 0x4204)
        inverted = [~crc & 0xFF, (~crc >> 8) & 0xFF]
        self.assertTrue(ow_check_crc16(data, inverted))


if __name__ == '__main__':
    unittest.main()
