import fire
from blueprints.nba.default import geometry

def get_gcp_coordinates(*pointNames):
    for pointName in pointNames:
        x, y, z = geometry["points"][pointName]["xyz"]
        print(f"{pointName}: {x}, {y}, {z}")

if __name__ == "__main__":
    fire.Fire(get_gcp_coordinates)