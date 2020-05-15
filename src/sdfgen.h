
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
    SDF_CONTOUR_TYPE_LINE,              /* line type contour             */
    SDF_CONTOUR_TYPE_QUADRATIC_BEZIER,  /* quadratic bezier type contour */
    SDF_CONTOUR_TYPE_CUBIC_BEZIER,      /* cubic bezier type contour     */

    SDF_CONTOUR_TYPE_MAX

  } SDF_Contour_Type;

  /* structure to hold a contour */
  typedef struct  SDF_Contour_
  {
    SDF_Vector            start_pos;        /* start position contour      */
    SDF_Vector            end_pos;          /* end position contour        */
    SDF_Vector            control_point_a;  /* unused in line type contour */
    SDF_Vector            control_point_b;  /* unused in line & quadratic  */

    SDF_Contour_Type      contour_type;     /* contour identifier          */

    struct SDF_Contour_*  next;             /* to create a linked list     */

  } SDF_Contour;

  /* structure represent a complete shape defined by FT_Outline */
  /* in a form of linked list                                   */
  typedef struct SDF_Shape_
  {
    SDF_Vector    current_pos;   /* to store move_to position       */

    SDF_Contour*  contour_head;  /* linked list of all the contours */

    FT_ULong      num_contours;  /* total number of contours        */

    FT_Memory     memory;        /* to allocate memory              */

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

  /* no need to create SDF_Contour_Done becuase              */
  /* SDF_Shape_Done will free all the allocated SDF_Contour  */
  FT_LOCAL( FT_Error )
  SDF_Shape_Done( SDF_Shape  *shape );

  FT_LOCAL( FT_Error )
  SDF_Decompose_Outline( FT_Outline* outline,
                         SDF_Shape  *shape );

FT_END_HEADER

#endif /* SDFGEN_H_ */

/* END */
