
#ifndef SDFGEN_H_
#define SDFGEN_H_

#include <ft2build.h>
#include FT_OUTLINE_H
#include FT_BITMAP_H

FT_BEGIN_HEADER

  /* generate sdf from outline */
  /* input: library, outline   */
  /* output: abitmap           */
  FT_EXPORT( FT_Error )
  FT_Generate_SDF( FT_Library   library,
                   FT_Outline*  outline, 
                   FT_Bitmap   *abitmap );

  
  /* Private Stuff */

  /**************************************************************************
   *
   * SDF Outline implementation structs and enums.
   *
   */

  /* float vector */
  typedef struct  SDF_Vector_
  {
    float  x;
    float  y;

  } SDF_Vector;

  /* enumeration for the types of contour present in FT_Outline */
  typedef enum  SDF_Contour_Type_
  {
    SDF_CONTOUR_TYPE_NONE,
    SDF_CONTOUR_TYPE_LINE,              /* line type contour */
    SDF_CONTOUR_TYPE_QUADRATIC_BEZIER,  /* quadratic bezier type contour */
    SDF_CONTOUR_TYPE_CUBIC_BEZIER,      /* cubic bezier type contour */

    SDF_CONTOUR_TYPE_MAX

  } SDF_Contour_Type;

  /* structure to hold a contour */
  typedef struct  SDF_Contour_
  {
    SDF_Vector  start_pos;          /* all contour type has a start position */
    SDF_Vector  end_pos;            /* all contour type has a end position */
                                    
    SDF_Vector  control_point_a;    /* unused in line type contour */
    SDF_Vector  control_point_b;    /* unused in line and quadratic bezier type contour */

    SDF_Contour_Type contour_type;  /* contour identifier */
    
    struct SDF_Contour_* next;     /* used to create a linked list of contour */

  } SDF_Contour;

  /* structure represent a complete shape defined by FT_Outline */
  typedef struct SDF_Shape_
  {
    SDF_Vector current_pos;      /* used to store move_to position while decomposing FT_Outline */

    SDF_Contour* contour_head;  /* linked list of all the contours present in the FT_Outline */

    FT_ULong num_contours;       /* number of contours present in the contour linked list */

  } SDF_Shape;


  /**************************************************************************
   *
   * SDF Outline implementation functions.
   *
   */

  FT_LOCAL( void )
  SDF_Contour_Init( SDF_Contour  *contour );

  FT_LOCAL( void )
  SDF_Shape_Init( SDF_Shape  *shape );

  /* no need to create SDF_Contour_Done becuase            */
  /* SDF_Shape_Done will free all the created SDF_Contour  */
  FT_LOCAL( FT_Error )
  SDF_Shape_Done( FT_Library  library,
                  SDF_Shape  *shape );

FT_END_HEADER

#endif /* SDFGEN_H_ */

/* END */
