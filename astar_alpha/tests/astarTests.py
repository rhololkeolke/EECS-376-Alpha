'''
Created on May 1, 2012

@author: Devin Schwab
'''
import unittest

from astar import Astar

class Test(unittest.TestCase):

    def setUp(self):
        self.searcher1 = Astar((0,0),(10,10),10)
        self.searcher2 = Astar((-5,-5),(5,5),10)
        self.searcher3 = Astar((0,0),(10,10),100)

    def test_Astar(self):
        
        searcher = self.searcher1
        # test the grid size
        for row in searcher.grid:
            self.assertEqual(len(row), 10)
            
        self.assertEqual(searcher.c1, (0,0))
        self.assertEqual(searcher.c2, (10,10))
        self.assertEqual(searcher.numCells, 10)
        
        self.assertEqual(searcher.start, None)
        self.assertEqual(searcher.goal, None)
        self.assertEqual(searcher.path, [])
        self.assertEqual(len(searcher._Astar__pathDict),0)
        
        searcher = self.searcher2
        for row in searcher.grid:
            self.assertEqual(len(row), 10)
            
        self.assertEqual(searcher.c1, (-5,-5))
        self.assertEqual(searcher.c2, (5,5))
        self.assertEqual(searcher.numCells, 10)
        
        self.assertEqual(searcher.start, None)
        self.assertEqual(searcher.goal, None)
        self.assertEqual(searcher.path, [])
        self.assertEqual(len(searcher._Astar__pathDict),0)
        
        searcher = self.searcher3
        for row in searcher.grid:
            self.assertEqual(len(row), 100)
            
        self.assertEqual(searcher.c1, (0,0))
        self.assertEqual(searcher.c2, (10,10))
        self.assertEqual(searcher.numCells, 100)
        
        self.assertEqual(searcher.start, None)
        self.assertEqual(searcher.goal, None)
        self.assertEqual(searcher.path, [])
        self.assertEqual(len(searcher._Astar__pathDict),0)
        
    def test_createGrid(self):
        searcher = self.searcher1
        
        grid = searcher.createGrid()
        for row in grid:
            self.assertEqual(len(row), 10)
            
        searcher = self.searcher2
        
        grid = searcher.createGrid()
        for row in grid:
            self.assertEqual(len(row), 10)

        searcher = self.searcher3
        
        grid = searcher.createGrid()
        for row in grid:
            self.assertEqual(len(row), 100)
        
    def test_transformMapToGrid(self):
        searcher = self.searcher1
        
        try:
            result = searcher.transformMapToGrid((-5,-5))
            self.fail("Expected an exception")
        except IndexError:
            pass

        try:
            result = searcher.transformMapToGrid((0,0))
            self.assertEqual(len(result),2)
            self.assertEquals(result[0],0)
            self.assertEquals(result[1],0)
        except:
            self.fail("Unexpected Exception")
            
        try:
            result = searcher.transformMapToGrid((5,5))
            self.assertEqual(len(result),2)
            self.assertEquals(result[0],5)
            self.assertEquals(result[1],5)
        except:
            self.fail("Unexpected Exception")
            
        try:
            result = searcher.transformMapToGrid((9.5,9.5))
            self.assertEqual(len(result),2)
            self.assertEquals(result[0],9)
            self.assertEquals(result[1],9)
        except:
            self.fail("Unexpected Exception")
            
        try:
            result = searcher.transformMapToGrid((10,10))
            self.fail("Expected an exception")
        except IndexError:
            pass
        
        try:
            result = searcher.transformMapToGrid((15,15))
            self.fail("Expected an exception")
        except IndexError:
            pass
        
        searcher = self.searcher2
        
        try:
            result = searcher.transformMapToGrid((-10,-10))
            self.fail("Expected an exception")
        except IndexError:
            pass

        try:
            result = searcher.transformMapToGrid((-5,-5))
            self.assertEqual(len(result),2)
            self.assertEquals(result[0],0)
            self.assertEquals(result[1],0)
        except:
            self.fail("Unexpected Exception")
            
        try:
            result = searcher.transformMapToGrid((0,0))
            self.assertEqual(len(result),2)
            self.assertEquals(result[0],5)
            self.assertEquals(result[1],5)
        except:
            self.fail("Unexpected Exception")
            
        try:
            result = searcher.transformMapToGrid((4.5,4.5))
            self.assertEqual(len(result),2)
            self.assertEquals(result[0],9)
            self.assertEquals(result[1],9)
        except:
            self.fail("Unexpected Exception")
            
        try:
            result = searcher.transformMapToGrid((5,5))
            self.fail("Expected an exception")
        except IndexError:
            pass
        
        try:
            result = searcher.transformMapToGrid((10,10))
            self.fail("Expected an exception")
        except IndexError:
            pass
        
        searcher = self.searcher3
        
        try:
            result = searcher.transformMapToGrid((-5,-5))
            self.fail("Expected an exception")
        except:
            pass

        try:
            result = searcher.transformMapToGrid((0,0))
            self.assertEqual(len(result),2)
            self.assertEquals(result[0],0)
            self.assertEquals(result[1],0)
        except:
            self.fail("Unexpected Exception")
            
        try:
            result = searcher.transformMapToGrid((5,5))
            self.assertEqual(len(result),2)
            self.assertEquals(result[0],50)
            self.assertEquals(result[1],50)
        except:
            self.fail("Unexpected Exception")
            
        try:
            result = searcher.transformMapToGrid((9.5,9.5))
            self.assertEqual(len(result),2)
            self.assertEquals(result[0],95)
            self.assertEquals(result[1],95)
        except:
            self.fail("Unexpected Exception")
            
        try:
            result = searcher.transformMapToGrid((10,10))
            self.fail("Expected an exception")
        except IndexError:
            pass
        
        try:
            result = searcher.transformMapToGrid((15,15))
            self.fail("Expected an exception")
        except IndexError:
            pass
            
    
    def test_transformGridToMap(self):
        searcher = self.searcher1
        
        try:
            (x,y) = searcher.transformGridToMap((-5,-5))
            self.fail("Expected an exception")
        except IndexError:
            pass
        
        try:
            (x,y) = searcher.transformGridToMap((0,0))
            self.assertAlmostEquals(x,0.0,delta=.0001)
            self.assertAlmostEquals(y,0.0,delta=.0001)
        except:
            self.fail("Unexpected Exception")
            
        try:
            (x,y) = searcher.transformGridToMap((5,5))
            self.assertAlmostEquals(x,5.0,delta=.0001)
            self.assertAlmostEquals(y,5.0,delta=.0001)
        except:
            self.fail("Unexpected Exception")
            
        try:
            (x,y) = searcher.transformGridToMap((9,9))
            self.assertAlmostEquals(x,9.0,delta=.0001)
            self.assertAlmostEquals(y,9.0,delta=.0001)
        except:
            self.fail("Unexpected Exception")
            
        try:
            (x,y) = searcher.transformGridToMap((10,10))
            self.fail("Expected an exception")
        except IndexError:
            pass
        
        try:
            (x,y) = searcher.transformGridToMap((15,15))
            self.fail("Expected an exception")
        except IndexError:
            pass
        
        searcher = self.searcher2
        
        try:
            (x,y) = searcher.transformGridToMap((-5,-5))
            self.fail("Expected an exception")
        except IndexError:
            pass
        
        try:
            (x,y) = searcher.transformGridToMap((0,0))
            self.assertAlmostEquals(x,-5.0,delta=.0001)
            self.assertAlmostEquals(y,-5.0,delta=.0001)
        except:
            self.fail("Unexpected Exception")
            
        try:
            (x,y) = searcher.transformGridToMap((5,5))
            self.assertAlmostEquals(x,0.0,delta=.0001)
            self.assertAlmostEquals(y,0.0,delta=.0001)
        except:
            self.fail("Unexpected Exception")
            
        try:
            (x,y) = searcher.transformGridToMap((9,9))
            self.assertAlmostEquals(x,4.0,delta=.0001)
            self.assertAlmostEquals(y,4.0,delta=.0001)
        except:
            self.fail("Unexpected Exception")
            
        try:
            (x,y) = searcher.transformGridToMap((10,10))
            self.fail("Expected an exception")
        except IndexError:
            pass
        
        try:
            (x,y) = searcher.transformGridToMap((15,15))
            self.fail("Expected an exception")
        except IndexError:
            pass
        
        searcher = self.searcher3
        
        try:
            (x,y) = searcher.transformGridToMap((-5,-5))
            self.fail("Expected an exception")
        except IndexError:
            pass
        
        try:
            (x,y) = searcher.transformGridToMap((0,0))
            self.assertAlmostEquals(x,0.0,delta=.0001)
            self.assertAlmostEquals(y,0.0,delta=.0001)
        except:
            self.fail("Unexpected Exception")
            
        try:
            (x,y) = searcher.transformGridToMap((5,5))
            self.assertAlmostEquals(x,0.5,delta=.0001)
            self.assertAlmostEquals(y,0.5,delta=.0001)
        except:
            self.fail("Unexpected Exception")
            
        try:
            (x,y) = searcher.transformGridToMap((93,93))
            self.assertAlmostEquals(x,9.3,delta=.0001)
            self.assertAlmostEquals(y,9.3,delta=.0001)
        except:
            self.fail("Unexpected Exception")
            
        try:
            (x,y) = searcher.transformGridToMap((100,100))
            self.fail("Expected an exception")
        except IndexError:
            pass
        
        try:
            (x,y) = searcher.transformGridToMap((150,150))
            self.fail("Expected an exception")
        except IndexError:
            pass
        
    
    def test_getNeighbors(self):
        searcher = self.searcher1
        
        neighbors = searcher.getNeighbors((5,5))
        self.assertEquals(len(neighbors),8)
        self.assertTrue((5,6) in neighbors)
        self.assertTrue((6,6) in neighbors)
        self.assertTrue((6,5) in neighbors)
        self.assertTrue((6,4) in neighbors)
        self.assertTrue((5,4) in neighbors)
        self.assertTrue((4,4) in neighbors)
        self.assertTrue((4,5) in neighbors)
        self.assertTrue((4,6) in neighbors)
        
        neighbors = searcher.getNeighbors((0,0))
        self.assertEquals(len(neighbors),3)
        self.assertTrue((0,1) in neighbors)
        self.assertTrue((1,1) in neighbors)
        self.assertTrue((1,0) in neighbors)
        
        neighbors = searcher.getNeighbors((9,9))
        self.assertEquals(len(neighbors),3)
        self.assertTrue((8,9) in neighbors)
        self.assertTrue((8,8) in neighbors)
        self.assertTrue((9,8) in neighbors)
        
    def test_populateGrid(self):
        searcher = self.searcher1
        
        closedList = [(-5,-5),(0,.1),(1,1.5),(2,2),(3,3.9),(4.9,4),(3.2,3.2)]
        closedGrid = [(0,0),(1,1),(2,2),(3,3),(4,4)]
        (conflict, newGrid) = searcher.populateGrid(closedList)
        self.assertFalse(conflict)
        
        numClosed = 0
        for i,row in enumerate(newGrid):
            for j,cell in enumerate(row):
                if(cell == -1):
                    numClosed += 1
                    self.assertTrue((i,j) in closedGrid)
        self.assertEqual(numClosed, len(closedGrid))
        
        searcher._Astar__pathDict[(0,0)] = True
        
        (conflict, newGrid) = searcher.populateGrid(closedList)
        self.assertTrue(conflict)
        
        numClosed = 0
        for i,row in enumerate(newGrid):
            for j,cell in enumerate(row):
                if(cell == -1):
                    numClosed += 1
                    self.assertTrue((i,j) in closedGrid)
        self.assertEqual(numClosed, len(closedGrid))
        
    
    def test_updateClosedList(self):
        searcher = self.searcher1
        
        closedList = [(-5,-5),(0,.1),(1,1.5),(2,2),(3,3.9),(4.9,4),(3.2,3.2)]
        closedGrid = [(0,0),(1,1),(2,2),(3,3),(4,4)]
        result = searcher.updateClosedList(closedList, recompute=False)
        
        self.assertFalse(result)
        
        newGrid = searcher.grid
        
        numClosed = 0
        for i,row in enumerate(newGrid):
            for j,cell in enumerate(row):
                if(cell == -1):
                    numClosed += 1
                    self.assertTrue((i,j) in closedGrid)
        self.assertEqual(numClosed, len(closedGrid))
        
        searcher._Astar__pathDict[(0,0)] = True
        
        result = searcher.updateClosedList(closedList, recompute=False)
        
        self.assertFalse(result)
        
        newGrid = searcher.grid
        
        numClosed = 0
        for i,row in enumerate(newGrid):
            for j,cell in enumerate(row):
                if(cell == -1):
                    numClosed += 1
                    self.assertTrue((i,j) in closedGrid)
        self.assertEqual(numClosed, len(closedGrid))
        
        searcher._Astar__pathDict = dict()
        
        result = searcher.updateClosedList(closedList)
        
        self.assertFalse(result)
        
        newGrid = searcher.grid
        
        numClosed = 0
        for i,row in enumerate(newGrid):
            for j,cell in enumerate(row):
                if(cell == -1):
                    numClosed += 1
                    self.assertTrue((i,j) in closedGrid)
        self.assertEqual(numClosed, len(closedGrid))
        
        searcher._Astar__pathDict[(0,0)] = True
        
        result = searcher.updateClosedList(closedList)
        
        self.assertTrue(result)
        
        newGrid = searcher.grid
        
        numClosed = 0
        for i,row in enumerate(newGrid):
            for j,cell in enumerate(row):
                if(cell == -1):
                    numClosed += 1
                    self.assertTrue((i,j) in closedGrid)
        self.assertEqual(numClosed, len(closedGrid))
        
        
    
    def test_updateGoal(self):
        searcher = self.searcher1
        
        result = searcher.updateGoal((5,5), recompute=False)
        self.assertFalse(result)
        self.assertEqual(searcher.goal, (5,5))
        
        result = searcher.updateGoal((5,5))
        self.assertTrue(result)
        self.assertEqual(searcher.goal, (5,5))
    
    def test_computePath(self):
        searcher = self.searcher1
        
        searcher.computePath((0,0), (1.5,1))
        self.assertEqual(searcher.path,[(0,0),(1,1)])
        
        searcher.computePath((0,0),(9.5,9.5))
        self.assertEqual(len(searcher.path),10)
        self.assertTrue((0,0) in searcher.path)
        self.assertTrue((1,1) in searcher.path)
        self.assertTrue((2,2) in searcher.path)
        self.assertTrue((3,3) in searcher.path)
        self.assertTrue((4,4) in searcher.path)
        self.assertTrue((5,5) in searcher.path)
        self.assertTrue((6,6) in searcher.path)
        self.assertTrue((7,7) in searcher.path)
        self.assertTrue((8,8) in searcher.path)
        self.assertTrue((9,9) in searcher.path)
        
        searcher.path = []
        closedGrid = [[ 0,-1, 0, 0, 0, 0, 0, 0, 0, 0],
                      [ 0,-1, 0, 0, 0, 0, 0, 0, 0, 0],
                      [ 0,-1, 0, 0, 0, 0, 0, 0, 0, 0],
                      [ 0,-1, 0, 0, 0, 0, 0, 0, 0, 0],
                      [ 0,-1, 0, 0, 0, 0, 0, 0, 0, 0],
                      [ 0,-1, 0, 0, 0, 0, 0, 0, 0, 0],
                      [ 0,-1, 0, 0, 0, 0, 0, 0, 0, 0],
                      [ 0,-1, 0, 0, 0, 0, 0, 0, 0, 0],
                      [ 0,-1, 0, 0, 0, 0, 0, 0, 0, 0],
                      [ 0,-1, 0, 0, 0, 0, 0, 0, 0, 0]]
        
        searcher.grid = closedGrid
        searcher.computePath((0,0),(0,2.5))
        self.assertEqual(searcher.path,[])
        
        searcher.path = []
        closedGrid = [[ 0,-1, 0, 0, 0, 0, 0, 0, 0, 0],
                      [ 0,-1, 0, 0, 0, 0, 0, 0, 0, 0],
                      [ 0,-1, 0, 0, 0, 0, 0, 0, 0, 0],
                      [ 0,-1, 0, 0, 0, 0, 0, 0, 0, 0],
                      [ 0,-1, 0, 0, 0, 0, 0, 0, 0, 0],
                      [ 0,-1, 0, 0, 0, 0, 0, 0, 0, 0],
                      [ 0,-1, 0, 0, 0, 0, 0, 0, 0, 0],
                      [ 0,-1, 0, 0, 0, 0, 0, 0, 0, 0],
                      [ 0,-1, 0, 0, 0, 0, 0, 0, 0, 0],
                      [ 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]]
        
        searcher.grid = closedGrid
        searcher.computePath((0,0),(0.1,2.5))
        self.assertEqual(len(searcher.path),19)
        self.assertTrue((0,0) in searcher.path)
        self.assertTrue((1,0) in searcher.path)
        self.assertTrue((2,0) in searcher.path)
        self.assertTrue((3,0) in searcher.path)
        self.assertTrue((4,0) in searcher.path)
        self.assertTrue((5,0) in searcher.path)
        self.assertTrue((6,0) in searcher.path)
        self.assertTrue((7,0) in searcher.path)
        self.assertTrue((8,0) in searcher.path)
        self.assertTrue((9,1) in searcher.path)
        self.assertTrue((8,2) in searcher.path)
        self.assertTrue((7,2) in searcher.path)
        self.assertTrue((6,2) in searcher.path)
        self.assertTrue((5,2) in searcher.path)
        self.assertTrue((4,2) in searcher.path)
        self.assertTrue((3,2) in searcher.path)
        self.assertTrue((2,2) in searcher.path)
        self.assertTrue((1,2) in searcher.path)
        self.assertTrue((0,2) in searcher.path)
        

        
if __name__ == "__main__":
    #import sys;sys.argv = ['', 'Test.testName']
    unittest.main()