
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
    SDF_CONTOUR_TYPE_NONE              = -1,
    SDF_CONTOUR_TYPE_LINE              =  0,  /* line type             */
    SDF_CONTOUR_TYPE_QUADRATIC_BEZIER  =  1,  /* quadratic bezier type */
    SDF_CONTOUR_TYPE_CUBIC_BEZIER      =  2,  /* cubic bezier type     */

    SDF_CONTOUR_TYPE_MAX               =  3

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

    FT_Memory     memory;        /* to allocate/deallocate memory   */

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
  SDF_Decompose_Outline( FT_Outline*  outline,
                         SDF_Shape   *shape );

  /**************************************************************************
   *
   * Math functions
   *
   */

  /* function to clamp the input value between min and max */
  FT_LOCAL( float )
  clamp( float   input,
         float   min,
         float   max );

  /* returns the magnitude of a vector */
  FT_LOCAL( float )
  sdf_vector_length( SDF_Vector  vector );

  /* returns the squared magnitude of a vector */
  FT_LOCAL( float )
  sdf_vector_squared_length( SDF_Vector  vector );

  /* retruns component wise addition of `a' and `b' */
  FT_LOCAL( SDF_Vector )
  sdf_vector_add( SDF_Vector  a,
                  SDF_Vector  b );

  /* retruns component wise subtraction of `a' and `b' */
  FT_LOCAL( SDF_Vector )
  sdf_vector_sub( SDF_Vector  a,
                  SDF_Vector  b );

  /* retruns component wise multiplication by `scale' */
  FT_LOCAL( SDF_Vector )
  sdf_vector_scale( SDF_Vector  vector,
                    float       scale );

  FT_LOCAL( float )
  sdf_vector_dot( SDF_Vector  a, 
                  SDF_Vector  b );

  /* returns a normalized vector ( i.e. vector length = 1.0f ) */
  FT_LOCAL( SDF_Vector )
  sdf_vector_normalize( SDF_Vector  vector );

  /* function returns the minimum distance from a point to a contour.     */
  /* the sign of the distance is determined by the side at which the      */
  /* the point is of the contour. The side is determined by the cross     */
  /* product of minimum distance vector and the direction of the contour  */
  FT_LOCAL( FT_Error )
  get_min_distance( SDF_Contour*       contour,
                    const SDF_Vector   point,
                    float             *min_distance );

FT_END_HEADER

#endif /* SDFGEN_H_ */

/* END */
