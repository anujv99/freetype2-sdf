
#ifndef SDFGEN_H_
#define SDFGEN_H_

#include <ft2build.h>
#include FT_FREETYPE_H
#include FT_OUTLINE_H
#include FT_BITMAP_H

FT_BEGIN_HEADER

  /* data type used for computting signed distance fields */
  typedef float  SDF_DataType;

  /* generate sdf from outline */
  /* input: library, outline   */
  /* output: abitmap           */
  FT_EXPORT( FT_Error )
  Generate_SDF( FT_Library     library,
                FT_GlyphSlot   glyph,
                SDF_DataType   spread,
                FT_Bitmap     *abitmap );


  /* Private Stuff */

  /**************************************************************************
   *
   * SDF Outline implementation structs and enums.
   *
   */

  /* SDF_DataType vector */
  typedef struct  SDF_Vector_
  {
    SDF_DataType  x;
    SDF_DataType  y;

  } SDF_Vector;

  /* can be used to determine weather the point is */
  /* inside or outside the shape's outline         */
  typedef struct SDF_Signed_Distance_
  {
    SDF_Vector  nearest_point;  /* point on curve                    */
    SDF_Vector  direction;      /* direction of curve on above point */
  } SDF_Signed_Distance;

  /* enumeration for the types of edge present in FT_Outline */
  typedef enum  SDF_Edge_Type_
  {
    SDF_EDGE_TYPE_NONE              = -1,
    SDF_EDGE_TYPE_LINE              =  0,  /* line type             */
    SDF_EDGE_TYPE_QUADRATIC_BEZIER  =  1,  /* quadratic bezier type */
    SDF_EDGE_TYPE_CUBIC_BEZIER      =  2,  /* cubic bezier type     */

    SDF_EDGE_TYPE_MAX               =  3

  } SDF_Edge_Type;

  /* structure to hold a edge */
  typedef struct  SDF_Edge_
  {
    SDF_Vector            start_pos;        /* start position of edge      */
    SDF_Vector            end_pos;          /* end position of edge        */
    SDF_Vector            control_point_a;  /* unused in line type         */
    SDF_Vector            control_point_b;  /* unused in line & quadratic  */

    SDF_Edge_Type         edge_type;        /* edge identifier             */

    struct SDF_Edge_*     next;             /* to create a linked list     */

  } SDF_Edge;

  /* structure represent a complete shape defined by FT_Outline */
  /* in a form of linked list                                   */
  typedef struct SDF_Shape_
  {
    SDF_Vector    current_pos;   /* to store move_to position     */

    SDF_Edge*     head;          /* linked list of all the edges  */

    FT_ULong      num_edges;     /* total number of edges         */

    FT_Memory     memory;        /* to allocate/deallocate memory */

  } SDF_Shape;


  /**************************************************************************
   *
   * SDF Outline implementation functions.
   *
   */

  FT_LOCAL( void )
  SDF_Edge_Init( SDF_Edge  *edge );

  FT_LOCAL( void )
  SDF_Shape_Init( SDF_Shape  *shape );

  /* no need to create SDF_Edge_Done becuase              */
  /* SDF_Shape_Done will free all the allocated SDF_Edge  */
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
  FT_LOCAL( SDF_DataType )
  clamp( SDF_DataType  input,
         SDF_DataType  min,
         SDF_DataType  max );

  /* solve a quadratic equation ( ax^2 + bx + c = 0; ) and return */
  /* the number of roots. the roots are written to `out'.         */
  FT_LOCAL( FT_UShort )
  solve_quadratic_equation( SDF_DataType  a,
                            SDF_DataType  b,
                            SDF_DataType  c,
                            SDF_DataType  out[2] );

  /* solve a cubic equation ( ax^3 + bx^2 + cx + d = 0; ) and     */
  /* return the number of roots. the roots are written to `out'.  */
  FT_LOCAL( FT_UShort )
  solve_cubic_equation( SDF_DataType  a,
                        SDF_DataType  b,
                        SDF_DataType  c,
                        SDF_DataType  d,
                        SDF_DataType  out[3] );

  /* returns the magnitude of a vector */
  FT_LOCAL( SDF_DataType )
  sdf_vector_length( SDF_Vector  vector );

  /* returns the squared magnitude of a vector */
  FT_LOCAL( SDF_DataType )
  sdf_vector_squared_length( SDF_Vector  vector );

  /* returns component wise addition of `a' and `b' */
  FT_LOCAL( SDF_Vector )
  sdf_vector_add( SDF_Vector  a,
                  SDF_Vector  b );

  /* returns component wise subtraction of `a' and `b' */
  FT_LOCAL( SDF_Vector )
  sdf_vector_sub( SDF_Vector  a,
                  SDF_Vector  b );

  /* returns component wise multiplication by `scale' */
  FT_LOCAL( SDF_Vector )
  sdf_vector_scale( SDF_Vector    vector,
                    SDF_DataType  scale );

  /* dot/scalar product of two vector `a' and `b' */
  FT_LOCAL( SDF_DataType )
  sdf_vector_dot( SDF_Vector  a, 
                  SDF_Vector  b );

  /* cross/vector product of two vector `a' and `b'. */
  /* the cross product will be in the z-axis.        */
  FT_LOCAL( SDF_DataType )
  sdf_vector_cross( SDF_Vector  a,
                    SDF_Vector  b );

  /* returns a normalized vector ( i.e. vector length = 1.0f ) */
  FT_LOCAL( SDF_Vector )
  sdf_vector_normalize( SDF_Vector  vector );

  /* return 1 if `a' and `b' are equal component-wise */
  FT_LOCAL( FT_Bool )
  sdf_vector_equal( SDF_Vector  a,
                    SDF_Vector  b );

  /* returns the signed distance of a point on the curve `edge' */
  /* that is nearest to `point'                                 */
  FT_LOCAL( FT_Error )
  get_min_distance( SDF_Edge*             edge,
                    const SDF_Vector      point,
                    SDF_Signed_Distance  *out );

FT_END_HEADER

#endif /* SDFGEN_H_ */

/* END */
