
#include <stdio.h>
#include <transmodel.h>

void TransitionModel::deallocate()
{
  int act;
  if(actions != NULL)
    delete[] actions;
  if(transitions != NULL)
    {
      for(act=0;act<nActions;act++)
	delete[] transitions[act];
    }
}

void TransitionModel::allocate()
{
  int act;
  actions=new ActionDef[nActions];
  transitions=new vector<MapCell>*[nActions];
  for(act=0;act<nActions;act++)
    transitions[act]=new vector<MapCell>[nAngles];
}

TransitionModel::TransitionModel(int num_actions, int num_angles)
{
  nActions=num_actions;
  nAngles=num_angles;
  allocate();
}

TransitionModel::~TransitionModel()
{
  deallocate();
}

vector<MapCell> &TransitionModel::getTransitions(const MapCoord &s, int const action)
{
  return transitions[action][s.angle];
}

void TransitionModel::addTransition(MapCoord s, int a, MapCoord sp, double prob)
{
  MapCell cell;
  cell.coord = sp;
  cell.value = prob;
  transitions[a][s.angle].push_back(cell);
}


#define THRESHOLD 127


static int checkmap(const MapStruct *map,int x,int y)
{
  int tmp=map->rows[y][x] ;
#ifdef SINGLERAY
  map->rows[y][x]=128; 
#endif
  if (tmp < THRESHOLD)
    return 1;
  return 0;
}

// see if you can get from start to end without hitting anything on the map
// if there is a collision, change end to the last safe coordinate and
// return true
static int collision(const MapCoord &start, MapCoord &end,const MapStruct *map)
{
  int
    x0=start.col,
    x1=end.col,
    y0=start.row,
    y1=end.row,
    dx = abs(x1-x0),
    sx = (x0<x1 ? 1 : -1),
    dy = abs(y1-y0),
    sy = (y0<y1 ? 1 : -1),
    err = (dx>dy ? dx : -dy)/2,
    e2,hit,prevx=start.col,prevy=start.row;

  //printf("Checking from %d,%d to %d,%d\n",y0,x0,y1,x1);

  // if(x0==x1 && y0==y1)
  //   return 0;
  
  while((x0!=x1 || y0!=y1) &&
	x0>0 &&
	x0<map->width &&
	y0>0 &&
	y0<map->height &&
	!(hit=checkmap(map,x0,y0)))
    {
      prevx = x0;
      prevy = y0;
      e2 = err;
      if (e2 >-dx)
	{
	  err -= dy;
	  x0 += sx;
	}
      if (e2 < dy)
	{
	  err += dx;
	  y0 += sy;
	}
    }
  hit=checkmap(map,x0,y0);
  if(hit || x0!=end.col || y0!=end.row) 
    {
      // printf("Changing from %d,%d to %d,%d\n",end.row,end.col,prevy,prevx);
      end.row = prevy;
      end.col = prevx;
      hit = 1;
    }
  return hit;
}

// getReward may change sp if there is a collision.
double TransitionModel::getReward(const MapCoord &s, const int a,
				   MapCoord &sp, const MapStruct *m,
				  const MapCell *goals, const int nGoals)
{
  int i;
  if(collision(s,sp,m))
    return -10.0;
  for(i=0;i<nGoals;i++)
    if(sp.row == goals[i].coord.row &&
       sp.col == goals[i].coord.col &&
       sp.angle == goals[i].coord.angle)
      return goals[i].value;
  // small negative reward forces search for goal  
  return -1.0;  
}
  
void TransitionModel::read(char *filename)
{
  FILE *f;
  int act,ang,i,size;
  MapCell cell;
  deallocate();
  if((f=fopen(filename,"r"))==NULL)
    {
      perror("TransitionModel::read : ");
      exit(100);
    }

  fread(&nActions,sizeof(nActions),1,f);
  fread(&nAngles,sizeof(nAngles),1,f);
  allocate();
  fread(actions,sizeof(ActionDef),nActions,f);
  for(act=0;act<nActions;act++)
    for(ang=0;ang<nAngles;ang++)
      {
	fread(&size,sizeof(size),1,f);
	transitions[act][ang].reserve(size);
	for(i=0;i<size;i++)
	  {
	    fread(&cell,sizeof(cell),1,f);
	    //	    transitions[act][ang][i]=cell;
	    transitions[act][ang].push_back(cell);
	  }
      }
  
  fclose(f);
}


void TransitionModel::write(char *filename)
{
  FILE *f;
  int act,ang,i,size;
  if((f=fopen(filename,"w"))==NULL)
    {
      perror("TransitionModel::write : ");
      exit(100);
    }
  fwrite(&nActions,sizeof(nActions),1,f);
  fwrite(&nAngles,sizeof(nAngles),1,f);
  fwrite(actions,sizeof(ActionDef),nActions,f);
  for(act=0;act<nActions;act++)
    for(ang=0;ang<nAngles;ang++)
      {
	size = transitions[act][ang].size();
	fwrite(&size,sizeof(size),1,f);
	for(i=0;i<size;i++)
	  fwrite(&(transitions[act][ang][i]),sizeof(MapCell),1,f);
      }
  fclose(f);
}


