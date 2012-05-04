#!/usr/bin/env python

if __name__ == "__main__":
    from brushfire import BrushFire

    brush = BrushFire((0,0),(10,10),20,5)
    obstacles = [(0,0),(1,1),(2,2),(3,3),(4,4)]
    brush.updateGlobalGrid(obstacles)
    obstacles = [(.5,.5),(1.5,1.5),(2.5,2.5),(3.5,3.5),(4.5,4.5)]
    brush.updateGlobalGrid(obstacles)
    brush.extractLocal(5,5)
    brush.brushfire()
    print brush


