'''
Created on May 1, 2012

@author: Devin Schwab
'''
import unittest
from space import Space

class Test(unittest.TestCase):

    def setUp(self):
        self.root = Space((0,0),(10,10))
        self.child = Space((1,1),(10,10),self.root)

    def test_Space(self):
        from math import sqrt
        
        root = self.root
        child = self.child
        
        self.assertEqual(root.point, (0,0))
        self.assertEqual(root.g, 0)
        self.assertAlmostEqual(root.h, sqrt(200), delta=.0001)
        
        self.assertEqual(child.point, (1,1))
        self.assertEqual(child.g, root.g+1)
        self.assertAlmostEqual(child.h, sqrt(2*81), delta=.0001)
    
    def test_f(self):
        root = self.root
        child = self.child
        
        self.assertAlmostEqual(root.f(),root.h+root.g)
        self.assertAlmostEqual(child.f(), child.h+child.g)
    
    def test_lt(self):
        root = self.root
        child = self.child
        
        self.assertFalse(root.__lt__(child))
        self.assertTrue(child.__lt__(root))


if __name__ == "__main__":
    #import sys;sys.argv = ['', 'Test.testName']
    unittest.main()