

/* system headers */
#include <png.h>
#include <stdlib.h>
#include <stdio.h>

/* my headers */
#include <map.h>

/* the following was used to for testing */
void asciidumpmap(MapStruct *m)
{
  int r,c;
  for(r=0;r<m->height;r++)
    {
      for(c=0;c<m->width;c++)
	if(m->rows[r][c])
	  printf("1");
	else
	  printf("0");
      printf("\n");
    }
}
 

/* read a PNG greyscale image to use as a map */
MapStruct* readmap(char **argv,int arg,int nAngles,double res)
{

  FILE *mapfile;
  png_structp png_ptr;
  png_infop info_ptr;
  unsigned char header[8]; // used to check file type
  MapStruct *map;
  
  /* open mapfile */
  if((mapfile = fopen(argv[arg], "r"))==NULL)
    {
      fprintf(stderr,"%s: Unable to open '%s' for input\n",argv[0],argv[arg]);
      perror(NULL);
      exit(1);
    }
  /* make sure mapfile is a png file */
  if (fread(header, 1, 8, mapfile) != 8)
    {
      fprintf(stderr,"%s: Unable to read from '%s'\n",argv[0],argv[arg]);
      perror(NULL);
      exit(2);
    }
  if (png_sig_cmp(header, 0, 8))
    {
      fprintf(stderr,"%s: '%s' is not a PNG file\n",argv[0],argv[arg]);
      exit(3);
    }
  png_ptr = png_create_read_struct(PNG_LIBPNG_VER_STRING, NULL, NULL, NULL);

  if (!png_ptr)
    {
      fprintf(stderr,"%s: Unable to allocate space for image\n",argv[0]);
      perror(NULL);
      exit(4);
    }
  info_ptr = png_create_info_struct(png_ptr);
  if (!info_ptr)
    {
      fprintf(stderr,"%s: Unable to allocate space for inmage info\n",argv[0]);
      perror(NULL);
      png_destroy_read_struct(&png_ptr,(png_infopp)NULL, (png_infopp)NULL);
      exit(5);
    }
  png_init_io(png_ptr, mapfile);
  png_set_sig_bytes(png_ptr, 8);
  png_set_compression_buffer_size(png_ptr, 8192);
  png_set_crc_action(png_ptr,PNG_CRC_NO_CHANGE,PNG_CRC_NO_CHANGE);
  png_set_user_limits(png_ptr, 1024*1024, 1024*1024);
  png_read_png(png_ptr, info_ptr, PNG_TRANSFORM_IDENTITY , NULL);

  if(png_get_color_type(png_ptr,info_ptr) != PNG_COLOR_TYPE_GRAY)
    {
      fprintf(stderr,"%s: '%s' is not a grayscale image\n",argv[0],argv[arg]);
      exit(6);
    }
  if(png_get_bit_depth(png_ptr,info_ptr) != 8)
    {
      fprintf(stderr,"%s: '%s' is not an 8-bit image\n",argv[0],argv[arg]);
      exit(7);
    }

  if((map = (MapStruct*)malloc(sizeof(MapStruct)))==NULL)
    {
      fprintf(stderr,"%s: Unable to allocate space for map\n",argv[0]);
      perror(NULL);
      exit(8);
    }
  
  if(fclose(mapfile))
    {
      fprintf(stderr,"%s: Error closing '%s'\n",argv[0],argv[arg]);
      perror(NULL);
      exit(9);
    }
  
  map->width = png_get_image_width(png_ptr, info_ptr);
  map->height = png_get_image_height(png_ptr, info_ptr);
  map->rows = png_get_rows(png_ptr,info_ptr);
  map->png_ptr = png_ptr;
  map->info_ptr = info_ptr;
  map->nAngles = nAngles;
  map->res = res;
  return map;
}


/* Convert Pose to map cell */
MapCoord PoseToCell(MapStruct *map, Pose2D &pose)
{
  double tt=pose.theta;
  MapCoord cell;
  while(tt < 0.0)
    tt += 2.0*M_PI;
  while(tt > 2.0*M_PI)
    tt-=2.0*M_PI;
  cell.angle = round(tt * map->nAngles / (2.0 * M_PI));
  if(cell.angle >= map->nAngles)
    cell.angle=0;
  cell.col = round(pose.x/map->res-0.5);
  cell.row = round(pose.y/map->res-0.5);
  if(cell.col >= map->width)
    cell.col = map->width-1;
  if(cell.row >= map->height)
    cell.row = map->height-1;
  if(cell.col < 0) 
    cell.col = 0;
  if(cell.row < 0)
    cell.row = 0;
  return cell;
}

/* Convert map cell to Pose by finding the center point of the cell */
Pose2D CellToPose(MapStruct *map, MapCoord &cell)
{
  Pose2D pose;
  pose.x = cell.col * map->res + 0.5*map->res;
  pose.y = cell.row * map->res + 0.5*map->res;
  pose.theta = (cell.angle * 2.0 * M_PI)/map->nAngles;
  return pose;
}

/* These functions allocate and free a 2D array of floats */
float** allocate_float_map(int width, int height)
{
  int i;
  float** data;
  if((data=(float**)malloc(height*sizeof(float*)))==NULL)
    {
      perror("allocate_float_map");
      exit(10);
    }
  for(i=0;i<height;i++)
    if((data[i]=(float*)malloc(width*sizeof(float)))==NULL)
      {
	perror("allocate_float_map");
	exit(10);
      }
  return data;
}

void free_float_map(float** mapdata,int height)
{
  for(int i=0;i<height;i++)
    free(mapdata[i]);
  free(mapdata);
}
  

/* write_float_map scales the values in a float map to between 0 and
   255, then writes the data as a Portable Gray Map pgm file.
   Filename should end in ".pgm" when this function is called. */
void write_float_map(char *filename,float **data,int width,int height,int autoscale)
{
  int row,col;
  float max=1.0;
  float min=0.0;
  FILE *f;
  unsigned char *irow;

  if((f=fopen(filename,"w"))==NULL)
    {
      perror(filename);
      exit(11);
    }

  fprintf(f,"P5\n%d %d\n255\n",width,height);
  
  if((irow=(unsigned char*)malloc(width*sizeof(unsigned char)))==NULL)
    {
      perror("write_float_map");
      exit(12);
    }
  
  if(autoscale)
    {
      max=data[0][0];
      min=data[0][0];
      for(row=0;row<height;row++)
	for(col=0;col<width;col++)
	  {
	    if(data[row][col]>max)
	      max = data[row][col];
	    if(data[row][col]<min)
	      min = data[row][col];
	  }
    }
  
  for(row=0;row<height;row++)
    {
      for(col=0;col<width;col++)
	irow[col] = ((data[row][col]-min)/(max-min))*255.0;
      fwrite(irow,sizeof(unsigned char),width,f);
    }

  
  fclose(f);
  free(irow);
}

/* These functions allocate and free a 2D array of doubles */
double** allocate_double_map(int width, int height)
{
  int i;
  double** data;
  if((data=(double**)malloc(height*sizeof(double*)))==NULL)
    {
      perror("allocate_double_map");
      exit(10);
    }
  for(i=0;i<height;i++)
    if((data[i]=(double*)malloc(width*sizeof(double)))==NULL)
      {
	perror("allocate_double_map");
	exit(10);
      }
  return data;
}

void free_double_map(double** mapdata,int height)
{
  for(int i=0;i<height;i++)
    free(mapdata[i]);
  free(mapdata);
}
  

/* write_double_map scales the values in a double map to between 0 and
   255, then writes the data as a Portable Gray Map pgm file.
   Filename should end in ".pgm" when this function is called. */
void write_double_map(char *filename,double **data,int width,int height,int autoscale)
{
  int row,col;
  double max=1.0;
  double min=0.0;
  FILE *f;
  unsigned char *irow;

  if((f=fopen(filename,"w"))==NULL)
    {
      perror(filename);
      exit(11);
    }

  fprintf(f,"P5\n%d %d\n255\n",width,height);
  
  if((irow=(unsigned char*)malloc(width*sizeof(unsigned char)))==NULL)
    {
      perror("write_double_map");
      exit(12);
    }
  
  if(autoscale)
    {
      max=data[0][0];
      min=data[0][0];
      for(row=0;row<height;row++)
	for(col=0;col<width;col++)
	  {
	    if(data[row][col]>max)
	      max = data[row][col];
	    if(data[row][col]<min)
	      min = data[row][col];
	  }
    }
  
  for(row=0;row<height;row++)
    {
      for(col=0;col<width;col++)
	irow[col] = ((data[row][col]-min)/(max-min))*255.0;
      fwrite(irow,sizeof(unsigned char),width,f);
    }

  
  fclose(f);
  free(irow);
}

