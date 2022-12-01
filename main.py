import random
import threading

class Map :
    
    def __init__(self, vert, horz, blocked) :
        self.data = [ [0] * horz for i in range(vert) ]
        while blocked > 0 :
            x, y = random.randint(0, vert - 1), random.randint(0, horz - 1)
            if self.data[x][y] != 1 :
                self.data[x][y] = 1
                blocked -= 1
        x, y = random.randint(0, vert - 1), random.randint(0, horz - 1)
        self.data[x][y] = 2
        param = (vert + horz) / 2
        while True :
            x2, y2 = random.randint(0, vert - 1), random.randint(0, horz - 1)
            if distance(x, y, x2, y2) > param :
                self.data[x2][y2] = 3
                break
            else :
                param *= 0.9

    
class Trace :
    
    def __init__(self, parent, x, y, cost, heuristic) :
        self.parent = parent
        self.x, self.y = x, y
        self.cost = cost
        self.heuristic = heuristic
        self.closed = False
        
    def close(self) :
        self.closed = True
    
    def changeWay(self, cost, parent) :
        if self.cost > cost :
            self.cost = cost
            self.parent = parent
    
    def totalCost(self) :
        return self.cost + self.heuristic
        
        
def distance(x1, y1, x2, y2) :
    return ((x2 - x1)**2 + (y2 - y1)**2)**0.50

def checkRange(s, e, n) :
    return not (s > n or e < n)


def astar(mapdata, hfunc, taximove) :
    data = mapdata.data
    steps = 0
    vert, horz = len(data), len(data[0])
    sx, sy = 0, 0
    ex, ey = 0, 0
    for x in range(horz) :
        for y in range(vert) :
            if data[x][y] == 2 :
                sx, sy = x, y
            if data[x][y] == 3 :
                ex, ey = x, y
                
    curr = Trace(None, sx, sy, 0, 0)
    curr.close()
    vL = [ curr ]

    while True :
        for dx in range(-1, 2) :
            for dy in range(-1, 2) :
                steps += 1
                if taximove and dx * dy != 0 :
                    continue
                if dx == dy == 0 :
                    continue
                nx, ny = curr.x + dx, curr.y + dy
                if not (checkRange(0, horz - 1, nx) and checkRange(0, vert - 1, ny)) :
                    continue
                if data[nx][ny] == 1 :
                    continue
                if dx * dy != 0 and data[curr.x][ny] == \
                    data[nx][curr.y] == 1 :
                    continue
                
                hcost = hfunc(nx, ny, ex, ey)
                cost = curr.cost + distance(0, 0, dx, dy)
                exist = False
                for node in vL :
                    if node.x == nx and node.y == ny :
                        exist = True
                        if node.closed :    
                            break
                        node.changeWay(cost, curr)
                        break
                if not exist :
                    vnode = Trace(curr, nx, ny, cost, hcost)
                    vL.append(vnode)
        dmin = None
        for node in vL :
            if node.closed :
                continue
            if dmin == None or node.totalCost() < dmin :
                dmin = node.totalCost()
        
        if dmin == None :
            return None
                
        hmin = None
        for node in vL :
            if node.totalCost() > dmin :
                continue
            if node.closed :
                continue
            if hmin == None or node.heuristic < hmin.heuristic :
                hmin = node
        curr = hmin
        curr.close()
        if curr.x == ex and curr.y == ey :
            return [curr.cost, steps]
        
##########################################################
#heuristics

def dijkstra_h(x, y, ex, ey) :
    return 0

def euclid_h(x, y, ex, ey) :
    return distance(x, y, ex, ey)

def taxi_h(x, y, ex, ey) :
    return abs(x + y - ex - ey)

def diagonal_h(x, y, ex, ey) :
    dx, dy = abs(ex - x), abs(ey - y)
    return min(dx, dy) * (2 ** 0.5) + abs(dx - dy)


##########################################################      

def main(hfunc, vert, horz, blocked, loop, log) :
    i = loop
    acc, effc = 0, 0
    acc_taxi, effc_taxi = 0, 0
    while i > 0 :
        m = Map(vert, horz, blocked)
        control = astar(m, dijkstra_h, False)
        if control == None :
            continue
        dist = astar(m, hfunc, False)
        acc += (control[0] / dist[0]) / loop
        effc += (control[1] / dist[1]) / loop
        
        control = astar(m, dijkstra_h, True)
        if control == None :
            continue
        dist = astar(m, hfunc, True)
        acc_taxi += (control[0] / dist[0]) / loop
        effc_taxi += (control[1] / dist[1]) / loop
        
        if (i - 1) % log == 0 :
            print("Process number " + str(loop - i + 1) + " completed")
        del(m)
        i -= 1
    
    print(f"RESULT ({hfunc.__name__}, {vert}, {horz}, {blocked}, {loop})")
    print("Accuracy in Euclid : " + str(acc))
    print("Efficiency in Euclid : " + str(effc))
    print("Accuracy in Taxi : " + str(acc_taxi))
    print("Efficiency in Taxi : " + str(effc_taxi))