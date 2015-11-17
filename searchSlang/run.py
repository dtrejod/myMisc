#!/usr/bin/env python
import os
import sys

# Import Class
#
from slangClass import slangSearch


def main(argv):

    # Create class object
    #
    mySlang = slangSearch() 
    
    # Read in the slang word list from a csv file
    #
    mySlang.read(argv[1])
    mySlang.myPrint()
    mySlang.fillXColumns(15)    
    
    # Read in file
    #
    myList.saveDataCsv('trejo_devin_surverys.csv')
    
    # Exit Gracefully
    #
    print("Program Finished successfully.")
   
    
# Run main if tihs is ran as main function. 
if __name__ == "__main__":
    main(sys.argv)
