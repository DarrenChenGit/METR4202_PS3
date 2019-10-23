import math as m

class Map:
    def __init__(self, x_length, y_length):
        self.x_length = x_length
        self.y_length = y_length
        self.grids = [[' ' for x in range(0, x_length, 1)] for y in range(0, y_length, 1)]

    #becareful of indexing, it is y, x.
    def display_map(self):
        for y in range(0, self.y_length, 1):
            
            for x in range(0, self.x_length, 1):
                print('|' + str(self.grids[self.y_length - 1 - y][x]), end = '')
            
                if x == (self.x_length - 1):
                    print('|')
        
                    
    def mark_location(self, x, y, icon):
        self.grids[y][x] = icon

    def mark_relative_location(self, currentX, currentY, distance, angle, icon):
        locationX = int(currentX + distance*m.cos(angle))
        locationY = int(currentY + distance*m.sin(angle))
        self.mark_location(locationX, locationY, icon)
