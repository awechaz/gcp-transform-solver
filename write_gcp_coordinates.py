import fire
from blueprints.nba.default import geometry

def write_gcp_coordinates(outputFilename, *pointNames):
    with open(outputFilename, "w") as f:
        for pointName in pointNames:
            x, y, z = geometry["points"][pointName]["xyz"]
            f.write(f"{x} {y} {z}\n")

    print("Point coordinates written to", outputFilename)

if __name__ == "__main__":
    fire.Fire(write_gcp_coordinates)