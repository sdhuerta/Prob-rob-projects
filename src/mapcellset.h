#ifndef SDSMT_MAPCELLSET
#define SDSMT_MAPCELLSET

#include <map.h>
#include <vector>
using namespace std;

class CellTreeIterator;
class CellTree;

class MapCellSet{
 private:
  CellTree *tree;
  int nitems;
  
 public:
  MapCellSet(){nitems=0;tree=NULL;}
  MapCellSet(MapCellSet &o);
  ~MapCellSet();


  /* we have an iterator to iterate through all cells in the set*/
  typedef CellTreeIterator iterator;

  /* these iterators are used to control loops                  */
  iterator begin();
  iterator end();

  /* getValue retrieves the current value for the given cell */
  /* If cell is not in the set, then it returns default_value  */
  double getValue(MapCoord cell, double default_value);
  
  /* setValue sets the current value for the given cell      */
  /* If cell is not in the set, then it is added               */
  void setValue(MapCoord cell, double value);  

  /* clear deletes all cells from the set                      */
  void clear();

  MapCellSet &operator =(MapCellSet &r);

  int nItems(){return nitems;}
  
};


class CellTreeIterator{
  vector<CellTree*> ptrs;
  //  CellTree **ptrs;
  int nptrs;
  int currptr;
  MapCell* rptr;
  
  int fillptrs(CellTree *root,vector<CellTree*> &ptrs,int pos);
  
 public:
  CellTreeIterator();
  CellTreeIterator(CellTree *root);
  //~CellTreeIterator(){if(ptrs != NULL)delete ptrs;}
  
  CellTreeIterator &operator ++();
  CellTreeIterator operator ++(int);
  
  MapCell &operator *(){return *rptr;}  
  MapCell* operator ->(){return rptr;}
  
  int operator !=(CellTreeIterator r){return rptr!=r.rptr;}
  
}; 


#endif
