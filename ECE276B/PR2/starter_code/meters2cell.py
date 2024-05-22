import numpy as np

np.set_printoptions(precision=3, suppress = True)

def meters2cells(x, m, r):
    """
    x: Coords of the origin
    m: minimum continuous metric coordinates contained in the grid
    r: grid resolution
    
    return:
        the coords of the point in cell
    """
    return np.floor((x-m)/r)

def cells2meters(x, m, r):
    """ 
    return:
        The coords of the point in meterss
    """
    return (x+0.5)*r+m

def main():
    print("Example implementation converting from meters to a grid and back.")
  
    x = np.random.rand(3)
    z = np.zeros(3)
    r = 0.1*np.ones(3)
    xc = meters2cells(x,z,r)
    xm = cells2meters(xc,z,r)
    
    
    print("The grid resolution is")
    print(r)
    print("The minimum continuous metric coordinates contained in the grid is ")
    print(z)
    
    print("\n")
    print("The original point is")
    print(x)

    print("The point in cells is")
    print(xc)

    print("The point converted back to meters is")
    print(xm)  

if __name__ == '__main__':
    main()
  