import sys
import argparse

parser = argparse.ArgumentParser(prog='Test', description='Test formulas and methods')
parser.add_argument('-c', '--co2_level', type=int)
args = parser.parse_args()


def co2_hue_interpolation_test():
    """Test HUE interpolation based on CO2 level from 1 to 3000

    https://www.w3schools.com/colors/colors_hsl.asp
    """
    co2 = args.co2_level
    co2_MAX = 2000
    print(f"CO2 level initial: {co2}")
    t = (co2 - 440) / (co2_MAX - 440)
    print(f"CO2 t: {t}")
    hue_float = (1 - t) * 96
    print(f"CO2 hue: {hue_float}")
    return None


if __name__ == "__main__":
    co2_hue_interpolation_test()