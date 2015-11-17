#!/usr/bin/env python
import os
import sys
from collections import Counter
import pandas as pd
from googlesearch import GoogleSearch
import re
import random

# wordCounts class
#
class slangSearch():

    # Initialize Variables
    #
    def __init__(self):
        # Define the column names
        #
        self.cols = ['Filename', 'Contents']
        
        # Initialize dataframe
        #
        self.massQ = pd.DataFrame()


    def read(self, fName):
        # Check if file exsits
        #
        try:
            file = open(fName, 'r')
        
        # Run this code if the open function throws an IOError
        #
        except IOError:
            print('There was an error opening the file \''+fName+'\'')
            sys.exit(1)
        
        # read in questions into a dataframe
        #
        self.massQ = pd.read_csv(fName, header=None) 
        
        # Clean up questions
        #
        self.massQ.replace({r'[a-zA-Z]\.': ''}, regex=True,
                inplace=True)
        self.massQ.replace({'[^a-zA-Z]+': ' '}, regex=True,\
                inplace=True)
        self.massQ.columns = ['Questions']
        self.massQ['Questions'] = self.massQ['Questions'].str.strip()
    
    # Print to console the contents of the dataFrame
    #
    def myPrint(self):
        print(self.massQ)
   
    def fillXColumns(self, X):
        # Define a dictionary to store our results
        #
        potentials = {}
        
        # Expand current dataframe by X columsn
        #
        for i in range(1,X):
            self.massQ[str(i)] = ""

        # Row index tracker (note column 0 are the questions)
        #
        row = 1

        # Loop through each question
        #
        for ques in self.massQ['Questions']:
            # We obtain 4 columns from google urban dictionary search
            #
            potentials['Urban'] = (self.__findUrban__(ques))

            # We obtain 4 most common word from a standard google search 
            #
            potentials['GoogleRaw'] = (self.__findCommonGoogle__(ques))
            
            # We obtain 4 most common words from a simlar google serach
            #
            words_ques = ques.split()
            # Append tilda in front of each word in the question
            #
            words_ques = ["~" + word for word in words_ques] 
            potentials['GoogleSim'] = (self.__findCommonGoogle__(\
                    " ".join(words_ques)))
            
            # Now from our results we populate the columns
            #
            for col in range(1,len(self.massQ.columns)):
                self.massQ.set_value(row, self.massQ.columns[col],\
                        random.choice(potentials.values()))
            row = row + 1

    # Creates a histogram and returns the most popular google search 
    # word in the results
    #
    def __findCommonGoogle__(self,phrase):
        # Get a list of all words from the top 4 results
        #
        all_words = self.__findGoogle__(phrase)
        all_words = self.__keepAlphaNum__(all_words)
        all_words = self.__removeArtPrepEct__(all_words)
        return self.__createHistogram__(all_words[:4])
    
    # Remove common words fromt the list
    #
    def __removeArtPrepEct__(self, words):
        words_str = " ".join(words)
        re.sub(r'(\s+)(a|an|and|the|of|to|be|in|that|have|'+\
            r'I|it|for|not|on|with|he|as|you|do|at|this|but|his|by|'+
            r'from|they|we|say|her|she|or|an|will|my|one|all|would|'+
            r'there)(\s+)','\1\3',words_str)
        return words_str.split()

    # Remove special Characters from list. Only return letters
    #
    def __keepAlphaNum__(self, words):
        words_str = " ".join(words)
        re.sub(r'[^a-zA-Z]+', ' ',words_str)
        return words_str.lower().split()
    
    # Returns top google results specifically from the site
    # urban dictionary
    def __findUrban__(self, phrase):
        # Define a list to store matches
        #
        gs = GoogleSearch(phrase + ' site:urbandictionary.com')
        
        # Create an empty list
        #
        matches = []
        # Loop over all the results
        #
        for hit in gs.top_results():
            # Append to our list the results removeing the phrase
            # 'Urban Dictionary: ' from the result
            #
            matches.append(hit['titleNoFormatting'].replace(\
                    'Urban Dictionary: ', ''))
        
        # return results from ubran dictionary google search
        #
        return matches


    # Returns top google results in general
    #
    def __findGoogle__(self, phrase):
        gs = GoogleSearch(phrase)
        matches = []
        for hit in gs.top_results():
            matches = hit['titleNoFormatting']
        return matches

            
    # Return the most common words from a list of words
    #
    def __createHistogram__(self, words):
        # Use counter to count word frequency
        #
        word_counts = Counter(words)

        # We convert the dictionary back to a dataframe allowing us to 
        # sort by frequency
        #
        df = pd.DataFrame.from_dict(word_counts, \
                orient='index').reset_index()
        df.sort(columns=[df.columns[1], df.columns[0]],\
                inplace=True, ascending=False)
        listdf = df[df.columns[0]].tolist()
        return listdf
    
    # Save the contents to a csv_files
    #
    def saveDataCsv(self, fName):
        self.massQ.to_csv(fName)

    # Save the contents of the dataframe to a file
    #
    def saveDataPkl(self, fName):
        self.massWord.to_pickle(fName)
    
    # Load the contents of a previous run into dataframe
    #
    def loadDataPkl(self,fName):
        self.massWord = pd.read_pickle(fName)


