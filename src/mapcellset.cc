
#include <mapcellset.h>

#define MAX(x,y) (x<y?y:x)


/*********************************************************/
/* The CellTree class holds a balanced binary tree of    */
/* MapCell structs                                       */
class CellTree{
private:
  MapCell cell;
  int height;
  CellTree *left, *right;
  int refs;

  static int getHeight(CellTree *node);
  static int getBalance(CellTree *node);
  static CellTree* rotate_left(CellTree* rt);
  static CellTree* rotate_right(CellTree* rt);
  
public:
  CellTree(MapCoord newcell, double newvalue);
  ~CellTree();

  void add_ref(){refs++;}
  int del_ref(){return --refs;}
  
  static CellTree* insert(CellTree *root, CellTree *node);
  static CellTree* find(CellTree *root, MapCoord cell);

  void setValue(double b){cell.value=b;}
  double getValue(){return cell.value;}

  typedef CellTreeIterator iterator;
  
  iterator begin() { return iterator(this); }
  iterator end() { return iterator(NULL); }

  friend class CellTreeIterator;

  void dump()
  {
    int i;
    if(left != NULL)
      left->dump();
    for(i=0;i<height;i++)
      printf("  ");
    print();
    if(right!=NULL)
      right->dump();
  }

  void print(){printf("%d %d %d %lf\n",
		      cell.coord.row,
		      cell.coord.col,
		      cell.coord.angle,
		      cell.value);}

};



CellTree::CellTree(MapCoord newcell,double newvalue)
{
  cell.coord=newcell;
  cell.value=newvalue;
  left = NULL;
  right = NULL;
  height = 1;
  refs = 1;
}

CellTree::~CellTree()
{
  
  if(left != NULL)
    delete left;
  if(left != NULL)
    delete right;
}

/*********************************************************/
/* find is used  to search the tree of cells. It         */
/* returns a pointer to the treenode. If the cell is     */
/* not in the tree, then it returns NULL.                */
CellTree* CellTree::find(CellTree *root, MapCoord cell)
{
  if(root != NULL)
    {
      if(cell.key < root->cell.coord.key)
	root = find(root->left,cell);
      else
	if(cell.key > root->cell.coord.key)
	  root = find(root->right,cell);
    }
  return root;
}



/**********************************************************/
/* tn_height finds the height of a node and returns      */
/* zero if the pointer is NULL.                           */
int CellTree::getHeight(CellTree *node)
{
  if(node == NULL)
    return 0;
  return node->height;
}

/**********************************************************/
/* find the balance factor of a node.                     */
/* returns zero if the pointer is NULL.                   */
int CellTree::getBalance(CellTree *node)
{
  if (node == NULL)
    return 0;
  return getHeight(node->left) - getHeight(node->right);
}

/**********************************************************/
/* rotate_left rotates counterclockwise                   */
CellTree* CellTree::rotate_left(CellTree* rt)
{
  CellTree* nrt = rt->right;
  rt->right = nrt->left;
  nrt->left = rt;
  nrt->refs = rt->refs;
  rt->refs = 1;
  rt->height =
    MAX(getHeight(rt->left),getHeight(rt->right)) + 1;
  nrt->height =
    MAX(getHeight(nrt->left),getHeight(nrt->right)) + 1;
  return nrt;
}

/**********************************************************/
/* rotate_right rotates clockwise                         */
CellTree* CellTree::rotate_right(CellTree* rt)
{
  CellTree* nrt = rt->left;
  rt->left = nrt->right;
  nrt->right = rt;
  nrt->refs = rt->refs;
  rt->refs = 1;
  rt->height =
    MAX(getHeight(rt->left),getHeight(rt->right)) + 1;
  nrt->height =
    MAX(getHeight(nrt->left),getHeight(nrt->right)) + 1;
  return nrt;
}

/**********************************************************/
/* insert performs a tree insertion and re-balances       */
CellTree * CellTree::insert(CellTree *root, CellTree *node)
{
  int bf;
  if (root == NULL)
    /* handle case where tree is empty, or we reached a leaf */
    root = node;
  else
    {
      /* Recursively search for insertion point, and perform the
	 insertion. */
      if(node->cell.coord.key < root->cell.coord.key)
	root->left  = insert(root->left, node);
      else
	root->right = insert(root->right, node);

      /* As we return from the recursive calls, recalculate the heights
         and perform rotations as necessary to re-balance the tree */
      root->height = MAX(getHeight(root->left),
			 getHeight(root->right)) + 1;

      /* Calculate the balance factor */
      bf = getBalance(root);
      if (bf > 1)
	{
	  /* the tree is deeper on the left than on the right) */
	  if(node->cell.coord.key <= root->left->cell.coord.key)
	    root = rotate_right(root);
	  else
	    {
	      root->left = rotate_left(root->left);
	      root = rotate_right(root);
	    };
	}
      else
	if(bf < -1)
	  {
	    /* the tree is deeper on the right than on the left) */
	    if(node->cell.coord.key >= root->right->cell.coord.key)
	      root = rotate_left(root);
	    else
	      {
		root->right = rotate_right(root->right);
		root = rotate_left(root);
	      }
	  }
    }
  return root;
}

int CellTreeIterator::fillptrs(CellTree *root,vector<CellTree*> &ptrs,int pos)
{
  if(root==NULL)
    return pos;
  pos = fillptrs(root->left,ptrs,pos);
  ptrs[pos++] = root;
  pos = fillptrs(root->right,ptrs,pos);
  return pos;
}

CellTreeIterator::CellTreeIterator()
{
  nptrs=0;
  currptr=0;
  rptr = NULL;
}


CellTreeIterator::CellTreeIterator(CellTree *root)
{
  if(root==NULL)
    {
      nptrs=0;
      currptr=0;
      rptr=NULL;
    }
  else
    {
      nptrs = 1<<(root->height);
      currptr = 0;
      ptrs.resize(nptrs);
      nptrs=fillptrs(root,ptrs,0);
      rptr=&(ptrs[0]->cell);
    }
}



// prefix case
CellTreeIterator &CellTreeIterator::operator ++()
{
  ++currptr;
  if(currptr >= nptrs)
    rptr=NULL;
  else
    rptr=&(ptrs[currptr]->cell);
  return *this;
}


// postfix case
CellTreeIterator CellTreeIterator::operator ++(int) 
{
  CellTreeIterator clone(*this); // clone is a shallow copy
  ++currptr;
  if(currptr >= nptrs)
    rptr=NULL;
  else
    rptr=&(ptrs[currptr]->cell);
  return clone;
}



MapCellSet::MapCellSet(MapCellSet &o)
{
  nitems = o.nitems;
  tree=o.tree;
  tree->add_ref();
}

MapCellSet::~MapCellSet()
{
  if(tree!=NULL && tree->del_ref() == 0)
    delete tree;
}


double MapCellSet::getValue(MapCoord cell, double default_value)
{
  CellTree *node;
  if(tree == NULL)
    return default_value;
  if((node = CellTree::find(tree,cell)) == NULL)
    return default_value;
  return node->getValue();
}


void MapCellSet::setValue(MapCoord cell, double value)
{
  CellTree *node;
  if(tree == NULL)
    tree = new CellTree(cell,value);
  else
    if((node = CellTree::find(tree,cell)) == NULL)
      {
	node = new CellTree(cell,value);
	tree = CellTree::insert(tree,node);
	nitems++;
      }
    else
      node->setValue(value);
}


void MapCellSet::clear()
{
  if(tree!=NULL && tree->del_ref() == 0)
    delete tree;
  tree = NULL;
  nitems = 0;
}


MapCellSet &MapCellSet::operator =(MapCellSet &r)
{
  nitems = r.nitems;
  tree = r.tree;
  tree->add_ref();
  return *this;
}


MapCellSet::iterator  MapCellSet::begin()
{
  return tree->begin();
}


MapCellSet::iterator MapCellSet::end()
{
  return tree->end();
}


// #include <time.h>

// int main()
// {
//   MapCellSet cellset;
//   int i;
//   uint32_t x,y,theta;
//   MapCoord cell;
//   srand(time(NULL));
  
//   for(i=0;i<100;i++)
//     {

//       cell.row=rand() & 0x1FFF;
//       cell.col=rand() & 0x1FFF;
//       cell.angle=rand() & 0x3F;
      
//       cellset.setValue(cell,0.0);
//     }
  
//   MapCellSet::iterator iter;

//   for(iter=cellset.begin();iter != cellset.end();iter++)
//     {
//       (*iter).print();
//     }
  
//   return 0;
// }


