import sys, os
current_dir = os.path.dirname(os.path.abspath(__file__))
sys.path.append(current_dir)

class Coordinate():
    def __init__(self):
        self.__latitude = 0.0
        self.__longitude = 0.0
        self.__altitude = 0.0
    
    def getLatitude(self) -> float:
        return self.__latitude
    def getLongitude(self) -> float:
        return self.__longitude
    def getAltitude(self) -> float:
        return self.__altitude
    def setCoordinate(self, latitude: float, longitude: float, altitude: float) -> None:
        self.__latitude = latitude
        self.__longitude = longitude
        self.__altitude = altitude

    
class Position():
    def __init__(self):
        self.__x = 0.0
        self.__y = 0.0
        self.__z = 0.0
    
    def getX(self) -> float:
        return self.__x
    def getY(self) -> float:
        return self.__y
    def getZ(self) -> float:
        return self.__z
    def setPosition(self, x: float, y: float, z: float) -> None:
        self.__x = x
        self.__y = y
        self.__z = z

class Orientation():
    def __init__(self):
        self.__x = 0.0
        self.__y = 0.0
        self.__z = 0.0
        self.__w = 0.0
    
    def getX(self) -> float:
        return self.__x
    def getY(self) -> float:
        return self.__y
    def getZ(self) -> float:
        return self.__z
    def getW(self) -> float:
        return self.__w
    def setPosition(self, x: float, y: float, z: float, w: float) -> None:
        self.__x = x
        self.__y = y
        self.__z = z
        self.__w = w